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
package com.orb.slam2s.app;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.hardware.camera2.CameraManager;
import android.os.Bundle;
import android.util.Log;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.core.content.pm.ShortcutInfoCompat;
import androidx.core.content.pm.ShortcutManagerCompat;
import androidx.core.graphics.drawable.IconCompat;
import androidx.core.view.WindowCompat;

import com.orb.slam2s.R;
import com.orb.slam2s.ui.IconSelectActivity;
import com.orb.slam2s.ui.MainActivity;
import com.orb.slam2s.ui.MapManageActivity;

// SplashActivity：启动权限检查与主界面分发
// 该 Activity 承担权限检查与分发逻辑（非纯启动图），不采用 Android 12+ SplashScreen API
@SuppressLint("CustomSplashScreen")
public class SplashActivity extends Activity {
    private static final String TAG = "SplashActivity";
    private static final int REQUEST_PERMISSION = 233;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        WindowCompat.setDecorFitsSystemWindows(getWindow(), false);
        setupDynamicShortcuts();
        if (checkPermission()) {
            launchMainActivity();
        }
    }

    private void setupDynamicShortcuts() {
        try {
            Intent mapIntent = new Intent(this, MapManageActivity.class);
            mapIntent.setAction(Intent.ACTION_VIEW);
            ShortcutInfoCompat mapShortcut = new ShortcutInfoCompat.Builder(this, "map_management")
                    .setShortLabel(getString(R.string.shortcut_map_manage))
                    .setLongLabel(getString(R.string.shortcut_map_manage))
                    .setIcon(IconCompat.createWithResource(this, R.drawable.ic_shortcut_map))
                    .setIntent(mapIntent)
                    .build();

            Intent iconIntent = new Intent(this, IconSelectActivity.class);
            iconIntent.setAction(Intent.ACTION_VIEW);
            ShortcutInfoCompat iconShortcut = new ShortcutInfoCompat.Builder(this, "change_icon")
                    .setShortLabel(getString(R.string.shortcut_change_icon))
                    .setLongLabel(getString(R.string.shortcut_change_icon))
                    .setIcon(IconCompat.createWithResource(this, R.drawable.ic_shortcut_icon))
                    .setIntent(iconIntent)
                    .build();

            ShortcutManagerCompat.pushDynamicShortcut(this, mapShortcut);
            ShortcutManagerCompat.pushDynamicShortcut(this, iconShortcut);
        } catch (Exception e) {
            Log.w(TAG, "注册动态快捷方式异常: " + e.getMessage());
        }
    }

    private int getCameraCount() {
        try {
            CameraManager cm = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
            if (cm != null) {
                String[] list = cm.getCameraIdList();
                return list != null ? list.length : 0;
            }
        } catch (Exception e) {
            Log.e(TAG, "检测相机列表异常: " + e.getMessage());
        }
        return 0;
    }

    private boolean checkPermission() {
        if (getCameraCount() == 0) {
            Log.w(TAG, "设备无物理相机，跳过相机权限申请直接进入主界面");
            return true;
        }

        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA)
                != PackageManager.PERMISSION_GRANTED) {
            if (ActivityCompat.shouldShowRequestPermissionRationale(this, Manifest.permission.CAMERA)) {
                showHint(getString(R.string.permission_camera_storage_required));
                finish();
            } else {
                ActivityCompat.requestPermissions(this,
                        new String[]{ Manifest.permission.CAMERA },
                        REQUEST_PERMISSION);
            }
            return false;
        }
        return true;
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQUEST_PERMISSION) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                launchMainActivity();
            } else {
                showHint(getString(R.string.permission_camera_storage_required));
                finish();
            }
        } else {
            finish();
        }
    }

    private void showHint(String hint) {
        Toast.makeText(this, hint, Toast.LENGTH_LONG).show();
    }

    private void launchMainActivity() {
        Intent intent = new Intent(SplashActivity.this, MainActivity.class);
        startActivity(intent);
        finish();
    }
}