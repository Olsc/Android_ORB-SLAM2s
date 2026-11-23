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

import android.content.ActivityNotFoundException;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.net.Uri;
import android.os.Build;
import android.provider.Settings;
import android.util.Log;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

// 厂商"自启动/后台启动"管理页的跳转：各品牌页面不同且没有统一接口，
// 按品牌挑选候选页面逐个尝试，全部失败再回退到系统应用详情页。
public final class AutoStartHelper {
    private static final String TAG = "AutoStartHelper";

    private AutoStartHelper() {
    }

    // 打开能放行 servicePackage 自启动的最合适页面；返回实际打开的页面描述，全部失败返回 null。
    public static String open(Context context, String servicePackage) {
        for (ComponentName candidate : candidatesForCurrentDevice()) {
            if (tryStart(context, new Intent().setComponent(candidate))) {
                return candidate.flattenToShortString();
            }
        }
        // 无自启动管理页的品牌（原生 Android、Pixel、Motorola 等）与所有候选都失败的设备
        final Intent details = new Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS,
                Uri.fromParts("package", servicePackage, null));
        if (tryStart(context, details)) return "app-details";
        Log.w(TAG, "无法打开任何设置页: " + servicePackage);
        return null;
    }

    // 当前设备品牌对应的候选页面，按可用性从高到低排列；未知品牌返回空列表
    static List<ComponentName> candidatesForCurrentDevice() {
        final String brand = (Build.MANUFACTURER + " " + Build.BRAND).toLowerCase(Locale.ROOT);
        final List<ComponentName> list = new ArrayList<>();
        if (matches(brand, "xiaomi", "redmi", "poco", "blackshark")) {
            add(list, "com.miui.securitycenter", "com.miui.permcenter.autostart.AutoStartManagementActivity");
        } else if (matches(brand, "huawei", "honor")) {
            add(list, "com.huawei.systemmanager", "com.huawei.systemmanager.startupmgr.ui.StartupNormalAppListActivity");
            add(list, "com.huawei.systemmanager", "com.huawei.systemmanager.appcontrol.activity.StartupAppControlActivity");
            add(list, "com.huawei.systemmanager", "com.huawei.systemmanager.optimize.process.ProtectActivity");
        } else if (matches(brand, "oppo", "realme", "oneplus")) {
            add(list, "com.coloros.safecenter", "com.coloros.safecenter.permission.startup.StartupAppListActivity");
            add(list, "com.coloros.safecenter", "com.coloros.safecenter.startupapp.StartupAppListActivity");
            add(list, "com.oppo.safe", "com.oppo.safe.permission.startup.StartupAppListActivity");
            add(list, "com.oneplus.security", "com.oneplus.security.chainlaunch.view.ChainLaunchAppListActivity");
            add(list, "com.coloros.oppoguardelf", "com.coloros.powermanager.fuelgaue.PowerUsageModelActivity");
        } else if (matches(brand, "vivo", "iqoo")) {
            add(list, "com.vivo.permissionmanager", "com.vivo.permissionmanager.activity.BgStartUpManagerActivity");
            add(list, "com.iqoo.secure", "com.iqoo.secure.ui.phoneoptimize.BgStartUpManager");
            add(list, "com.iqoo.secure", "com.iqoo.secure.ui.phoneoptimize.AddWhiteListActivity");
            add(list, "com.vivo.permissionmanager", "com.vivo.permissionmanager.activity.PurviewTabActivity");
        } else if (matches(brand, "samsung")) {
            add(list, "com.samsung.android.lool", "com.samsung.android.sm.battery.ui.BatteryActivity");
            add(list, "com.samsung.android.lool", "com.samsung.android.sm.ui.battery.BatteryActivity");
            add(list, "com.samsung.android.sm_cn", "com.samsung.android.sm.ui.ram.AutoRunActivity");
        } else if (matches(brand, "meizu")) {
            add(list, "com.meizu.safe", "com.meizu.safe.permission.SmartBGActivity");
        } else if (matches(brand, "letv", "leeco")) {
            add(list, "com.letv.android.letvsafe", "com.letv.android.letvsafe.AutobootManageActivity");
        } else if (matches(brand, "asus")) {
            add(list, "com.asus.mobilemanager", "com.asus.mobilemanager.autostart.AutoStartActivity");
            add(list, "com.asus.mobilemanager", "com.asus.mobilemanager.powersaver.PowerSaverSettings");
        } else if (matches(brand, "tecno", "infinix", "itel", "transsion")) {
            add(list, "com.transsion.phonemaster", "com.cyin.himgr.autostart.AutoStartActivity");
        } else if (matches(brand, "nokia", "hmd")) {
            add(list, "com.evenwell.powersaving.g3", "com.evenwell.powersaving.g3.exception.PowerSaverExceptionActivity");
        } else if (matches(brand, "lenovo", "zuk")) {
            add(list, "com.lenovo.security", "com.lenovo.security.purebackground.PureBackgroundActivity");
        }
        return Collections.unmodifiableList(list);
    }

    private static boolean matches(String brand, String... keys) {
        for (String key : keys) {
            if (brand.contains(key)) return true;
        }
        return false;
    }

    private static void add(List<ComponentName> list, String pkg, String cls) {
        list.add(new ComponentName(pkg, cls));
    }

    // 页面不存在、未导出或被系统限制都会抛异常，统一视为该候选不可用
    private static boolean tryStart(Context context, Intent intent) {
        try {
            if (!(context instanceof android.app.Activity)) {
                intent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
            }
            context.startActivity(intent);
            return true;
        } catch (ActivityNotFoundException | SecurityException e) {
            return false;
        } catch (RuntimeException e) {
            Log.w(TAG, "启动设置页失败: " + e.getMessage());
            return false;
        }
    }
}