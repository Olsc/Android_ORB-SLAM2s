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
import android.app.AlertDialog;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.hardware.camera2.CameraManager;
import android.net.Uri;
import android.os.Bundle;
import android.os.StrictMode;
import android.provider.Settings;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.core.content.pm.ShortcutInfoCompat;
import androidx.core.content.pm.ShortcutManagerCompat;
import androidx.core.graphics.drawable.IconCompat;
import androidx.core.view.WindowCompat;

import com.orb.slam2s.BuildConfig;
import com.orb.slam2s.R;
import com.orb.slam2s.ui.IconSelectActivity;
import com.orb.slam2s.ui.MainActivity;
import com.orb.slam2s.ui.MapManageActivity;
import com.orb.slam2s.util.IconManager;

// SplashActivity：启动权限检查与主界面分发
// 该 Activity 承担权限检查与分发逻辑（非纯启动图），不采用 Android 12+ SplashScreen API
@SuppressLint("CustomSplashScreen")
public class SplashActivity extends Activity {
    private static final String TAG = "SplashActivity";
    private static final int REQUEST_PERMISSION = 233;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        enableStrictModeForDebug();
        WindowCompat.setDecorFitsSystemWindows(getWindow(), false);
        // 启动时自检桌面图标组件状态，避免出现“桌面上找不到图标”的情况
        IconManager.ensureLauncherIcon(this);
        setupDynamicShortcuts();
        if (checkPermission()) {
            launchMainActivity();
        }
    }

    // 仅 Debug 构建启用 StrictMode
    private void enableStrictModeForDebug() {
        if (!BuildConfig.DEBUG) return;
        StrictMode.setThreadPolicy(new StrictMode.ThreadPolicy.Builder()
                .detectAll()
                .penaltyLog()
                .build());
        StrictMode.setVmPolicy(new StrictMode.VmPolicy.Builder()
                .detectAll()
                .penaltyLog()
                .build());
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
                == PackageManager.PERMISSION_GRANTED) {
            return true;
        }

        if (ActivityCompat.shouldShowRequestPermissionRationale(this, Manifest.permission.CAMERA)) {
            // 用户此前拒绝过：先说明用途，再重新请求，避免直接退出导致无法再次授权
            showPermissionRationaleDialog();
        } else {
            ActivityCompat.requestPermissions(this,
                    new String[]{ Manifest.permission.CAMERA },
                    REQUEST_PERMISSION);
        }
        return false;
    }

    private void showPermissionRationaleDialog() {
        new AlertDialog.Builder(this)
                .setTitle(R.string.permission_camera_title)
                .setMessage(R.string.permission_camera_rationale)
                .setCancelable(false)
                .setPositiveButton(R.string.button_ok, (d, w) -> ActivityCompat.requestPermissions(
                        SplashActivity.this,
                        new String[]{ Manifest.permission.CAMERA },
                        REQUEST_PERMISSION))
                .setNegativeButton(R.string.action_exit, (d, w) -> finish())
                .show();
    }

    // 权限被“拒绝且不再询问”时，引导到系统设置开启，而不是直接阻断用户
    private void showPermissionDeniedDialog() {
        new AlertDialog.Builder(this)
                .setTitle(R.string.permission_camera_title)
                .setMessage(R.string.permission_camera_denied)
                .setCancelable(false)
                .setPositiveButton(R.string.action_open_settings, (d, w) -> {
                    try {
                        Intent intent = new Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS,
                                Uri.fromParts("package", getPackageName(), null));
                        startActivity(intent);
                    } catch (Exception e) {
                        Log.e(TAG, "打开应用设置失败: " + e.getMessage());
                    }
                    finish();
                })
                .setNegativeButton(R.string.action_exit, (d, w) -> finish())
                .show();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode != REQUEST_PERMISSION) {
            finish();
            return;
        }
        if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            launchMainActivity();
        } else if (ActivityCompat.shouldShowRequestPermissionRationale(this, Manifest.permission.CAMERA)) {
            showPermissionRationaleDialog();
        } else {
            showPermissionDeniedDialog();
        }
    }

    private void launchMainActivity() {
        Intent intent = new Intent(SplashActivity.this, MainActivity.class);
        startActivity(intent);
        finish();
    }
}