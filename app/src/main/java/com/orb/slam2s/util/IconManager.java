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

import androidx.annotation.DrawableRes;
import androidx.annotation.StringRes;

import com.orb.slam2s.R;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

// 桌面图标管理工具类：基于 activity-alias 动态切换桌面应用图标。
// 图标以 IconOption 列表集中登记，新增一款图标只需在 OPTIONS 中追加一项，
// 并在 AndroidManifest 中声明对应的 activity-alias 即可，无需改动界面逻辑。
public class IconManager {
    private static final String TAG = "IconManager";
    private static final String PREFS_NAME = "icon_settings";
    private static final String KEY_CURRENT_ICON = "current_icon";

    // 图标类型标识，同时用于 SharedPreferences 持久化
    public enum IconType {
        DEFAULT,
        RED,
        MINT
    }

    // 单款桌面图标的配置信息
    public static final class IconOption {
        public final IconType type;
        public final String alias;
        @StringRes
        public final int nameRes;
        @DrawableRes
        public final int previewRes;

        IconOption(IconType type, String alias, @StringRes int nameRes, @DrawableRes int previewRes) {
            this.type = type;
            this.alias = alias;
            this.nameRes = nameRes;
            this.previewRes = previewRes;
        }
    }

    private static final String ALIAS_PACKAGE = "com.orb.slam2s.app.";
    private static final List<IconOption> OPTIONS;

    static {
        List<IconOption> options = new ArrayList<>();
        options.add(new IconOption(
                IconType.DEFAULT,
                ALIAS_PACKAGE + "SplashActivityDefault",
                R.string.icon_default_name,
                R.mipmap.ic_launcher));
        options.add(new IconOption(
                IconType.RED,
                ALIAS_PACKAGE + "SplashActivityRed",
                R.string.icon_red_name,
                R.mipmap.ic_launcher_red));
        options.add(new IconOption(
                IconType.MINT,
                ALIAS_PACKAGE + "SplashActivityMint",
                R.string.icon_mint_name,
                R.mipmap.ic_launcher_mint));
        OPTIONS = Collections.unmodifiableList(options);
    }

    private IconManager() {
    }

    // 获取全部可选的桌面图标配置
    public static List<IconOption> getOptions() {
        return OPTIONS;
    }

    // 按类型获取图标配置
    public static IconOption getOption(IconType type) {
        for (IconOption option : OPTIONS) {
            if (option.type == type) {
                return option;
            }
        }
        return OPTIONS.get(0);
    }

    // 获取当前处于启用状态的桌面图标类型
    public static IconType getCurrentIconType(Context context) {
        try {
            PackageManager pm = context.getPackageManager();
            for (IconOption option : OPTIONS) {
                ComponentName component = new ComponentName(context, option.alias);
                if (pm.getComponentEnabledSetting(component)
                        == PackageManager.COMPONENT_ENABLED_STATE_ENABLED) {
                    return option.type;
                }
            }
        } catch (Exception e) {
            Log.w(TAG, "获取图标启用状态异常: " + e.getMessage());
        }

        return getPreferredIconType(context);
    }

    // 读取用户偏好（兼容旧版本的 CUSTOM 命名），作为组件状态缺失时的兜底
    private static IconType getPreferredIconType(Context context) {
        SharedPreferences sp = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        String saved = sp.getString(KEY_CURRENT_ICON, IconType.DEFAULT.name());
        if ("CUSTOM".equals(saved)) {
            // 旧版本“烈焰红”的持久化值，重命名为 RED 后需要迁移
            return IconType.RED;
        }
        for (IconOption option : OPTIONS) {
            if (option.type.name().equals(saved)) {
                return option.type;
            }
        }
        return IconType.DEFAULT;
    }

    // 校验桌面图标组件状态：确保始终至少有一个 activity-alias 处于启用状态。
    // 用于修复因 alias 重命名、系统状态残留等导致“桌面上找不到应用图标”的问题。
    public static void ensureLauncherIcon(Context context) {
        try {
            PackageManager pm = context.getPackageManager();
            boolean anyEnabled = false;
            for (IconOption option : OPTIONS) {
                ComponentName component = new ComponentName(context, option.alias);
                if (pm.getComponentEnabledSetting(component)
                        == PackageManager.COMPONENT_ENABLED_STATE_ENABLED) {
                    anyEnabled = true;
                    break;
                }
            }
            if (!anyEnabled) {
                IconType preferred = getPreferredIconType(context);
                switchIcon(context, preferred);
                Log.w(TAG, "未检测到可用的桌面图标组件，已自动恢复为: " + preferred);
            }
        } catch (Exception e) {
            Log.e(TAG, "校验桌面图标组件状态失败: " + e.getMessage(), e);
        }
    }

    // 切换桌面图标样式并持久化组件启用状态
    public static boolean switchIcon(Context context, IconType targetType) {
        try {
            PackageManager pm = context.getPackageManager();
            for (IconOption option : OPTIONS) {
                ComponentName component = new ComponentName(context, option.alias);
                int state = (option.type == targetType)
                        ? PackageManager.COMPONENT_ENABLED_STATE_ENABLED
                        : PackageManager.COMPONENT_ENABLED_STATE_DISABLED;
                pm.setComponentEnabledSetting(component, state, PackageManager.DONT_KILL_APP);
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
