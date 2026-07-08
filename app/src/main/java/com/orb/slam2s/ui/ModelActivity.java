package com.orb.slam2s.ui;

import android.Manifest;
import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;

import android.widget.Toast;

import com.orb.slam2s.R;

import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

/**
 * ModelActivity 类，用于处理权限请求、初始化模型并跳转到下一个活动界面。
 * 该活动会在启动时请求权限，并解压所需的模型文件。
 */
public class ModelActivity extends Activity {
    private static final int REQUEST_PERMISSION = 233; // 请求权限的请求码

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        // 不再使用加载界面，直接进行初始化

        // 检查权限，若权限通过则初始化
        if(checkPermission())
            init();
    }

    /**
     * 初始化方法，启动异步任务解压模型文件
     */
    private void init(){
        extractModelFiles();
    }

    /**
     * 检查是否获得了 CAMERA 权限
     * @return 是否有权限
     */
    private boolean checkPermission(){
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
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
            } else return true;
        }
        return true;
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String permissions[], int[] grantResults) {
        switch (requestCode) {
            case REQUEST_PERMISSION:
                // 如果权限被授予
                if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    init();  // 权限通过后初始化
                } else {
                    showHint(getString(R.string.permission_camera_storage_required));  // 权限未通过，提示信息
                    finish();  // 结束活动
                }
                break;
            default:
                finish();  // 如果请求码不匹配，结束活动
                break;
        }
    }

    /**
     * 显示提示信息
     * @param hint 提示的消息
     */
    private void showHint(String hint){
        Toast.makeText(this,hint , Toast.LENGTH_LONG).show(); // 弹出提示
    }

    private void extractModelFiles() {
        // 模型文件已通过嵌入式资源加载，无需从 assets 提取
        // 直接跳转到主界面
        Intent intent = new Intent(ModelActivity.this, ArCamUIActivity.class);
        startActivity(intent);
        finish();
    }
}
