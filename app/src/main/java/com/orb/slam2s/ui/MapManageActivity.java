/*
 * Copyright (C) 2026 Olsc <OlscStudio@outlook.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.orb.slam2s.ui;

import android.os.Bundle;
import android.text.format.Formatter;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowCompat;
import androidx.core.view.WindowInsetsCompat;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.orb.slam2s.R;
import com.orb.slam2s.util.MapManager;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

public class MapManageActivity extends AppCompatActivity {

    private MapManager mMapManager;
    private RecyclerView mRecyclerView;
    private View mEmptyLayout;
    private TextView mTvStorageStats;
    private TextView mTvStoragePath;
    private MapAdapter mAdapter;
    private final List<MapManager.MapInfo> mMapList = new ArrayList<>();
    private final SimpleDateFormat mDateFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault());

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_map_manage);

        WindowCompat.setDecorFitsSystemWindows(getWindow(), false);
        setupWindowInsets();

        mMapManager = new MapManager(this);

        initViews();
        loadMaps();
    }

    private void setupWindowInsets() {
        View rootLayout = findViewById(R.id.root_map_manage_layout);
        if (rootLayout == null) return;

        ViewCompat.setOnApplyWindowInsetsListener(rootLayout, (v, windowInsets) -> {
            Insets insets = windowInsets.getInsets(
                    WindowInsetsCompat.Type.systemBars() | WindowInsetsCompat.Type.displayCutout()
            );
            v.setPadding(insets.left, insets.top, insets.right, insets.bottom);
            return windowInsets;
        });
    }

    private void initViews() {
        ImageButton btnBack = findViewById(R.id.btn_back);
        ImageButton btnRefresh = findViewById(R.id.btn_refresh);
        mTvStorageStats = findViewById(R.id.tv_storage_stats);
        mTvStoragePath = findViewById(R.id.tv_storage_path);
        mEmptyLayout = findViewById(R.id.layout_empty);
        mRecyclerView = findViewById(R.id.rv_map_list);

        btnBack.setOnClickListener(v -> finish());
        btnRefresh.setOnClickListener(v -> {
            loadMaps();
            Toast.makeText(this, R.string.map_action_refresh, Toast.LENGTH_SHORT).show();
        });

        mRecyclerView.setLayoutManager(new LinearLayoutManager(this));
        mAdapter = new MapAdapter();
        mRecyclerView.setAdapter(mAdapter);

        File dir = mMapManager.getMapDirectory();
        if (dir != null) {
            mTvStoragePath.setText(getString(R.string.map_details_path, dir.getAbsolutePath()));
        }
    }

    private void loadMaps() {
        mMapList.clear();
        ArrayList<MapManager.MapInfo> maps = mMapManager.getAllMaps();
        if (maps != null) {
            mMapList.addAll(maps);
        }

        long totalSize = 0;
        for (MapManager.MapInfo info : mMapList) {
            totalSize += info.fileSize;
        }

        String formattedSize = Formatter.formatFileSize(this, totalSize);
        mTvStorageStats.setText(getString(R.string.map_manage_stats, mMapList.size(), formattedSize));

        if (mMapList.isEmpty()) {
            mEmptyLayout.setVisibility(View.VISIBLE);
            mRecyclerView.setVisibility(View.GONE);
        } else {
            mEmptyLayout.setVisibility(View.GONE);
            mRecyclerView.setVisibility(View.VISIBLE);
            mAdapter.notifyDataSetChanged();
        }
    }

    private void showMapDetailsDialog(MapManager.MapInfo mapInfo) {
        String formattedSize = Formatter.formatFileSize(this, mapInfo.fileSize);
        File mapFile = mMapManager.getMapFile(mapInfo.name);
        File arInfoFile = new File(mMapManager.getMapDirectory(), mapInfo.name + ".bin.arinfo");
        File metaFile = new File(mMapManager.getMapDirectory(), mapInfo.name + ".json");

        StringBuilder sb = new StringBuilder();
        sb.append(getString(R.string.map_details_name, mapInfo.name)).append("\n");
        sb.append(getString(R.string.map_details_keyframes, mapInfo.keyFrames)).append("\n");
        sb.append(getString(R.string.map_details_mappoints, mapInfo.mapPoints)).append("\n");
        sb.append(getString(R.string.map_details_size, mapInfo.fileSize / 1024)).append(" (").append(formattedSize).append(")\n");
        sb.append(getString(R.string.map_details_time, mDateFormat.format(new Date(mapInfo.createTime)))).append("\n");
        sb.append(getString(R.string.map_details_plane, mapInfo.hasPlane ? getString(R.string.map_details_plane_yes) : getString(R.string.map_details_plane_no))).append("\n\n");
        sb.append(getString(R.string.map_details_path, mapFile.getAbsolutePath())).append("\n");
        sb.append("AR Info: ").append(arInfoFile.exists() ? getString(R.string.map_details_plane_yes) : getString(R.string.map_details_plane_no)).append("\n");
        sb.append("Metadata JSON: ").append(metaFile.exists() ? getString(R.string.map_details_plane_yes) : getString(R.string.map_details_plane_no));

        new AlertDialog.Builder(this)
                .setTitle(getString(R.string.dialog_map_details))
                .setMessage(sb.toString())
                .setPositiveButton(getString(R.string.button_ok), null)
                .show();
    }

    private void confirmDeleteMap(MapManager.MapInfo mapInfo) {
        new AlertDialog.Builder(this)
                .setTitle(getString(R.string.dialog_confirm_delete))
                .setMessage(getString(R.string.dialog_confirm_delete_message, mapInfo.name))
                .setPositiveButton(getString(R.string.action_delete), (dialog, which) -> {
                    if (mMapManager.deleteMap(mapInfo.name)) {
                        Toast.makeText(MapManageActivity.this, getString(R.string.hint_map_deleted), Toast.LENGTH_SHORT).show();
                        loadMaps();
                    } else {
                        Toast.makeText(MapManageActivity.this, getString(R.string.hint_map_delete_failed), Toast.LENGTH_SHORT).show();
                    }
                })
                .setNegativeButton(getString(R.string.button_cancel), null)
                .show();
    }

    private class MapAdapter extends RecyclerView.Adapter<MapViewHolder> {

        @NonNull
        @Override
        public MapViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
            View view = LayoutInflater.from(parent.getContext()).inflate(R.layout.item_map_card, parent, false);
            return new MapViewHolder(view);
        }

        @Override
        public void onBindViewHolder(@NonNull MapViewHolder holder, int position) {
            MapManager.MapInfo info = mMapList.get(position);
            holder.bind(info);
        }

        @Override
        public int getItemCount() {
            return mMapList.size();
        }
    }

    private class MapViewHolder extends RecyclerView.ViewHolder {
        private final TextView tvName;
        private final TextView tvSize;
        private final TextView tvKeyframes;
        private final TextView tvMapPoints;
        private final TextView tvPlaneStatus;
        private final TextView tvTime;
        private final Button btnDetails;
        private final Button btnDelete;

        public MapViewHolder(@NonNull View itemView) {
            super(itemView);
            tvName = itemView.findViewById(R.id.tv_map_name);
            tvSize = itemView.findViewById(R.id.tv_map_size);
            tvKeyframes = itemView.findViewById(R.id.tv_keyframes);
            tvMapPoints = itemView.findViewById(R.id.tv_mappoints);
            tvPlaneStatus = itemView.findViewById(R.id.tv_plane_status);
            tvTime = itemView.findViewById(R.id.tv_map_time);
            btnDetails = itemView.findViewById(R.id.btn_details);
            btnDelete = itemView.findViewById(R.id.btn_delete);
        }

        public void bind(MapManager.MapInfo info) {
            tvName.setText(info.name);
            tvSize.setText(Formatter.formatFileSize(itemView.getContext(), info.fileSize));
            tvKeyframes.setText(getString(R.string.map_details_keyframes, info.keyFrames));
            tvMapPoints.setText(getString(R.string.map_details_mappoints, info.mapPoints));
            tvPlaneStatus.setText(getString(R.string.map_details_plane, info.hasPlane ? getString(R.string.map_details_plane_yes) : getString(R.string.map_details_plane_no)));
            tvTime.setText(mDateFormat.format(new Date(info.createTime)));

            btnDetails.setOnClickListener(v -> showMapDetailsDialog(info));
            btnDelete.setOnClickListener(v -> confirmDeleteMap(info));
        }
    }
}