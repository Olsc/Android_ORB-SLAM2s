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
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.core.view.WindowCompat;

import com.orb.slam2s.R;
import com.orb.slam2s.ui.MainActivity;

// SplashActivity：启动权限检查与主界面分发
// 该 Activity 承担权限检查与分发逻辑（非纯启动图），不采用 Android 12+ SplashScreen API
@SuppressLint("CustomSplashScreen")
public class SplashActivity extends Activity {
    private static final int REQUEST_PERMISSION = 233;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        WindowCompat.setDecorFitsSystemWindows(getWindow(), false);
        if (checkPermission()) {
            launchMainActivity();
        }
    }

    private boolean checkPermission() {
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
