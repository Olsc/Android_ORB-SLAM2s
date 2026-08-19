package com.orb.slam2s.app;

import android.Manifest;
import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.orb.slam2s.R;
import com.orb.slam2s.ui.MainActivity;

// SplashActivity：启动权限检查与主界面分发
public class SplashActivity extends Activity {
    private static final int REQUEST_PERMISSION = 233;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
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
