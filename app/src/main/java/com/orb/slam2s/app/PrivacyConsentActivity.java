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

import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.res.ColorStateList;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowCompat;
import androidx.core.view.WindowInsetsCompat;
import androidx.core.view.WindowInsetsControllerCompat;

import com.orb.slam2s.R;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

import io.noties.markwon.Markwon;
import io.noties.markwon.ext.tables.TablePlugin;

public class PrivacyConsentActivity extends AppCompatActivity {

    // 隐私协议开关：true 启动时展示隐私协议，false 跳过直接进入应用
    public static final boolean ENABLE_PRIVACY = true;

    private static final String TAG = "PrivacyConsentActivity";
    private static final String PREF_NAME = "privacy_prefs";
    private static final String KEY_AGREED = "privacy_agreed";

    private Button btnAgree;
    private TextView tvScrollHint;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // 如果隐私协议开关关闭，直接跳过隐私页面进入应用
        if (!ENABLE_PRIVACY) {
            startNextActivity();
            return;
        }

        // 如果已经同意过，直接跳转
        if (hasAgreed()) {
            startNextActivity();
            return;
        }

        setContentView(R.layout.activity_privacy_consent);

        // 沉浸式边到边 + 常亮
        WindowCompat.setDecorFitsSystemWindows(getWindow(), false);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        WindowInsetsControllerCompat controller = WindowCompat.getInsetsController(getWindow(), getWindow().getDecorView());
        if (controller != null) {
            controller.setAppearanceLightNavigationBars(false);
            controller.setAppearanceLightStatusBars(false);
        }

        setupWindowInsets();

        ScrollView scrollView = findViewById(R.id.scroll_view_privacy);
        btnAgree = findViewById(R.id.btn_agree);
        tvScrollHint = findViewById(R.id.tv_scroll_hint);

        // 加载对应语言的 markdown 内容
        String markdownContent = loadPrivacyMarkdown();
        if (markdownContent == null) {
            Toast.makeText(this, getString(R.string.privacy_load_error), Toast.LENGTH_LONG).show();
            finish();
            return;
        }

        // 使用 Markwon 渲染 markdown（启用表格插件支持 GFM 表格）
        TextView tvContent = findViewById(R.id.tv_privacy_content);
        Markwon markwon = Markwon.builder(this)
                .usePlugin(TablePlugin.create(this))
                .build();
        markwon.setMarkdown(tvContent, markdownContent);

        // 设置滚动监听：滑到底部才启用同意按钮
        scrollView.setOnScrollChangeListener((v, scrollX, scrollY, oldScrollX, oldScrollY) -> {
            if (!v.canScrollVertically(1)) { // 无法继续向下滚动 = 已到底部
                btnAgree.setEnabled(true);
                btnAgree.setBackgroundTintList(ColorStateList.valueOf(0xFF4CAF50));
                btnAgree.setTextColor(0xFFFFFFFF);
                tvScrollHint.setText(getString(R.string.privacy_scroll_done));
                tvScrollHint.setTextColor(0xFF4CAF50);
            }
        });

        // 同意按钮点击事件
        btnAgree.setOnClickListener(v -> {
            saveAgreement();
            startNextActivity();
        });
    }

    private void setupWindowInsets() {
        View rootLayout = findViewById(R.id.root_privacy_layout);
        if (rootLayout == null) return;

        ViewCompat.setOnApplyWindowInsetsListener(rootLayout, (v, windowInsets) -> {
            Insets insets = windowInsets.getInsets(
                    WindowInsetsCompat.Type.systemBars() | WindowInsetsCompat.Type.displayCutout()
            );

            // 根视图增加两侧与顶部 Safe Insets
            v.setPadding(insets.left, insets.top, insets.right, 0);

            // 底部控制栏增加 Bottom Safe Inset 避让系统导航条/手势棒
            View bottomBar = findViewById(R.id.bottom_bar);
            if (bottomBar != null) {
                int basePadding = (int) (10 * getResources().getDisplayMetrics().density);
                bottomBar.setPadding(
                        bottomBar.getPaddingLeft(),
                        basePadding,
                        bottomBar.getPaddingRight(),
                        basePadding + insets.bottom
                );
            }

            return windowInsets;
        });
    }

    // 根据系统语言加载对应的隐私协议 markdown 文件
    private String loadPrivacyMarkdown() {
        String fileName;
        String localeLang = Locale.getDefault().getLanguage();

        if ("zh".equals(localeLang)) {
            fileName = "privacy/PRIVACY_ZH.md";
        } else {
            fileName = "privacy/PRIVACY_EN.md";
        }

        Log.d(TAG, "正在加载隐私协议文件：" + fileName + " (语言: " + localeLang + ")");

        try (InputStream is = getAssets().open(fileName);
             BufferedReader reader = new BufferedReader(new InputStreamReader(is, StandardCharsets.UTF_8))) {
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) {
                sb.append(line).append("\n");
            }
            return sb.toString();
        } catch (IOException e) {
            Log.e(TAG, "隐私文件加载失败：" + fileName, e);
            return null;
        }
    }

    // 检查用户是否已同意隐私协议
    private boolean hasAgreed() {
        SharedPreferences prefs = getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE);
        return prefs.getBoolean(KEY_AGREED, false);
    }

    // 保存用户同意状态
    private void saveAgreement() {
        SharedPreferences prefs = getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE);
        prefs.edit().putBoolean(KEY_AGREED, true).apply();
        Log.d(TAG, "已保存隐私协议");
    }

    // 跳转到启动引导与权限检查 Activity
    private void startNextActivity() {
        Intent intent = new Intent(this, SplashActivity.class);
        startActivity(intent);
        finish();
    }
}