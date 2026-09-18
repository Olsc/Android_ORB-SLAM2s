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
import android.view.View;
import android.widget.ImageButton;
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

// 桌面图标选择 Activity：提供经典蓝与烈焰红两款图标的切换交互
public class IconSelectActivity extends AppCompatActivity {

    private View mCardDefault;
    private View mCardCustom;
    private RadioButton mRbDefault;
    private RadioButton mRbCustom;
    private TextView mTvBadgeDefault;
    private TextView mTvBadgeCustom;

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
        mCardDefault = findViewById(R.id.card_icon_default);
        mCardCustom = findViewById(R.id.card_icon_custom);
        mRbDefault = findViewById(R.id.rb_default);
        mRbCustom = findViewById(R.id.rb_custom);
        mTvBadgeDefault = findViewById(R.id.tv_badge_default);
        mTvBadgeCustom = findViewById(R.id.tv_badge_custom);

        btnBack.setOnClickListener(v -> finish());

        mCardDefault.setOnClickListener(v -> applyIcon(IconManager.IconType.DEFAULT));
        mCardCustom.setOnClickListener(v -> applyIcon(IconManager.IconType.CUSTOM));
    }

    private void applyIcon(IconManager.IconType targetType) {
        if (targetType == IconManager.getCurrentIconType(this)) {
            return;
        }

        boolean success = IconManager.switchIcon(this, targetType);
        if (success) {
            updateSelectionUI(targetType);
            String name = (targetType == IconManager.IconType.DEFAULT)
                    ? getString(R.string.icon_default_name)
                    : getString(R.string.icon_custom_name);
            Toast.makeText(this, getString(R.string.icon_switched_hint, name), Toast.LENGTH_SHORT).show();
        }
    }

    private void updateSelectionUI(IconManager.IconType currentType) {
        boolean isDefault = (currentType == IconManager.IconType.DEFAULT);

        mRbDefault.setChecked(isDefault);
        mRbCustom.setChecked(!isDefault);

        mTvBadgeDefault.setVisibility(isDefault ? View.VISIBLE : View.GONE);
        mTvBadgeCustom.setVisibility(!isDefault ? View.VISIBLE : View.GONE);
    }
}