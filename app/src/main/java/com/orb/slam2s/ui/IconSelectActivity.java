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
import android.view.LayoutInflater;
import android.view.View;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.RadioButton;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowCompat;
import androidx.core.view.WindowInsetsCompat;

import com.orb.slam2s.R;
import com.orb.slam2s.util.IconManager;

import java.util.ArrayList;
import java.util.List;

// 桌面图标选择 Activity：根据 IconManager 中登记的图标选项动态生成切换卡片，
// 新增图标无需修改本类，只需在 IconManager.OPTIONS 中追加配置即可。
public class IconSelectActivity extends AppCompatActivity {

    // 单个图标选项对应的视图引用
    private static final class OptionViewHolder {
        final IconManager.IconOption option;
        final RadioButton radio;
        final TextView badge;

        OptionViewHolder(IconManager.IconOption option, RadioButton radio, TextView badge) {
            this.option = option;
            this.radio = radio;
            this.badge = badge;
        }
    }

    private final List<OptionViewHolder> mOptionViews = new ArrayList<>();

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_icon_select);

        WindowCompat.setDecorFitsSystemWindows(getWindow(), false);
        setupWindowInsets();

        initViews();
        updateSelectionUI(IconManager.getCurrentIconType(this));
    }

    private void setupWindowInsets() {
        View rootLayout = findViewById(R.id.root_icon_select_layout);
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
        btnBack.setOnClickListener(v -> finish());

        LinearLayout container = findViewById(R.id.icon_options_container);
        LayoutInflater inflater = LayoutInflater.from(this);
        for (IconManager.IconOption option : IconManager.getOptions()) {
            View card = inflater.inflate(R.layout.item_icon_option, container, false);

            ImageView preview = card.findViewById(R.id.iv_icon_preview);
            TextView name = card.findViewById(R.id.tv_icon_name);
            TextView badge = card.findViewById(R.id.tv_icon_badge);
            RadioButton radio = card.findViewById(R.id.rb_icon);

            preview.setImageResource(option.previewRes);
            name.setText(option.nameRes);
            card.setOnClickListener(v -> applyIcon(option.type));

            container.addView(card);
            mOptionViews.add(new OptionViewHolder(option, radio, badge));
        }
    }

    private void applyIcon(IconManager.IconType targetType) {
        if (targetType == IconManager.getCurrentIconType(this)) {
            return;
        }

        boolean success = IconManager.switchIcon(this, targetType);
        if (success) {
            updateSelectionUI(targetType);
            String name = getString(IconManager.getOption(targetType).nameRes);
            Toast.makeText(this, getString(R.string.icon_switched_hint, name), Toast.LENGTH_SHORT).show();
        }
    }

    private void updateSelectionUI(IconManager.IconType currentType) {
        for (OptionViewHolder holder : mOptionViews) {
            boolean selected = (holder.option.type == currentType);
            holder.radio.setChecked(selected);
            holder.badge.setVisibility(selected ? View.VISIBLE : View.GONE);
        }
    }
}