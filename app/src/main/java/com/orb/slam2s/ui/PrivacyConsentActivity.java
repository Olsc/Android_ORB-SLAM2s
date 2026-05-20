package com.orb.slam2s.ui;

import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import com.orb.slam2s.R;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Locale;

import io.noties.markwon.Markwon;

public class PrivacyConsentActivity extends AppCompatActivity {

    private static final String TAG = "PrivacyConsent";
    private static final String PREF_NAME = "privacy_prefs";
    private static final String KEY_AGREED = "privacy_agreed";

    private ScrollView scrollView;
    private Button btnAgree;
    private TextView tvScrollHint;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // 如果已经同意过，直接跳转
        if (hasAgreed()) {
            startNextActivity();
            return;
        }

        setContentView(R.layout.activity_privacy_consent);

        // 全屏 + 常亮
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
                WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        scrollView = findViewById(R.id.scroll_view_privacy);
        btnAgree = findViewById(R.id.btn_agree);
        tvScrollHint = findViewById(R.id.tv_scroll_hint);

        // 加载对应语言的 markdown 内容
        String markdownContent = loadPrivacyMarkdown();
        if (markdownContent == null) {
            Toast.makeText(this, getString(R.string.privacy_load_error), Toast.LENGTH_LONG).show();
            finish();
            return;
        }

        // 使用 Markwon 渲染 markdown
        TextView tvContent = findViewById(R.id.tv_privacy_content);
        Markwon markwon = Markwon.create(this);
        markwon.setMarkdown(tvContent, markdownContent);

        // 设置滚动监听：滑到底部才启用同意按钮
        scrollView.setOnScrollChangeListener((v, scrollX, scrollY, oldScrollX, oldScrollY) -> {
            if (!v.canScrollVertically(1)) { // 无法继续向下滚动 = 已到底部
                btnAgree.setEnabled(true);
                btnAgree.setBackgroundTintList(android.content.res.ColorStateList.valueOf(0xFF4CAF50));
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

    /**
     * 根据系统语言加载对应的隐私协议 markdown 文件
     */
    private String loadPrivacyMarkdown() {
        String fileName;
        String localeLang = Locale.getDefault().getLanguage();

        if ("zh".equals(localeLang)) {
            fileName = "privacy/PRIVACY_ZH.md";
        } else {
            fileName = "privacy/PRIVACY_EN.md";
        }

        Log.d(TAG, "正在加载隐私协议文件：" + fileName + " (语言: " + localeLang + ")");

        try {
            InputStream is = getAssets().open(fileName);
            BufferedReader reader = new BufferedReader(new InputStreamReader(is, "UTF-8"));
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) {
                sb.append(line).append("\n");
            }
            reader.close();
            return sb.toString();
        } catch (IOException e) {
            Log.e(TAG, "隐私文件加载失败：" + fileName, e);
            return null;
        }
    }

    /**
     * 检查用户是否已同意隐私协议
     */
    private boolean hasAgreed() {
        SharedPreferences prefs = getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE);
        return prefs.getBoolean(KEY_AGREED, false);
    }

    /**
     * 保存用户同意状态
     */
    private void saveAgreement() {
        SharedPreferences prefs = getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE);
        prefs.edit().putBoolean(KEY_AGREED, true).apply();
        Log.d(TAG, "已保存隐私协议");
    }

    /**
     * 跳转到主应用的 ModelActivity
     */
    private void startNextActivity() {
        Intent intent = new Intent(this, ModelActivity.class);
        startActivity(intent);
        finish();
    }
}
