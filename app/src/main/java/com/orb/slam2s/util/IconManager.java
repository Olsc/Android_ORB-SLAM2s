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
package com.orb.slam2s.util;

import android.content.ComponentName;
import android.content.Context;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.util.Log;

// 桌面图标管理工具类：基于 activity-alias 动态切换桌面应用图标
public class IconManager {
    private static final String TAG = "IconManager";
    private static final String PREFS_NAME = "icon_settings";
    private static final String KEY_CURRENT_ICON = "current_icon";

    public static final String ALIAS_DEFAULT = "com.orb.slam2s.app.SplashActivityDefault";
    public static final String ALIAS_CUSTOM = "com.orb.slam2s.app.SplashActivityCustom";

    public enum IconType {
        DEFAULT,
        CUSTOM
    }

    // 获取当前处于启用状态的桌面图标类型
    public static IconType getCurrentIconType(Context context) {
        try {
            PackageManager pm = context.getPackageManager();
            ComponentName customComponent = new ComponentName(context, ALIAS_CUSTOM);
            int state = pm.getComponentEnabledSetting(customComponent);
            if (state == PackageManager.COMPONENT_ENABLED_STATE_ENABLED) {
                return IconType.CUSTOM;
            }
        } catch (Exception e) {
            Log.w(TAG, "获取图标启用状态异常: " + e.getMessage());
        }

        SharedPreferences sp = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        String saved = sp.getString(KEY_CURRENT_ICON, IconType.DEFAULT.name());
        return IconType.CUSTOM.name().equals(saved) ? IconType.CUSTOM : IconType.DEFAULT;
    }

    // 切换桌面图标样式并持久化组件启用状态
    public static boolean switchIcon(Context context, IconType targetType) {
        try {
            PackageManager pm = context.getPackageManager();
            ComponentName defaultComp = new ComponentName(context, ALIAS_DEFAULT);
            ComponentName customComp = new ComponentName(context, ALIAS_CUSTOM);

            if (targetType == IconType.CUSTOM) {
                pm.setComponentEnabledSetting(defaultComp,
                        PackageManager.COMPONENT_ENABLED_STATE_DISABLED,
                        PackageManager.DONT_KILL_APP);
                pm.setComponentEnabledSetting(customComp,
                        PackageManager.COMPONENT_ENABLED_STATE_ENABLED,
                        PackageManager.DONT_KILL_APP);
            } else {
                pm.setComponentEnabledSetting(customComp,
                        PackageManager.COMPONENT_ENABLED_STATE_DISABLED,
                        PackageManager.DONT_KILL_APP);
                pm.setComponentEnabledSetting(defaultComp,
                        PackageManager.COMPONENT_ENABLED_STATE_ENABLED,
                        PackageManager.DONT_KILL_APP);
            }

            SharedPreferences sp = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
            sp.edit().putString(KEY_CURRENT_ICON, targetType.name()).apply();
            Log.d(TAG, "已切换应用桌面图标为: " + targetType);
            return true;
        } catch (Exception e) {
            Log.e(TAG, "切换桌面图标失败: " + e.getMessage(), e);
            return false;
        }
    }
}