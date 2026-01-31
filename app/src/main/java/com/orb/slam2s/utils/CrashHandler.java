package com.orb.slam2s.utils;

import android.app.Application;
import android.os.Process;
import android.util.Log;

public class CrashHandler implements Thread.UncaughtExceptionHandler {
    private static final String TAG = "CrashHandler";
    private final Thread.UncaughtExceptionHandler defaultHandler;

    private CrashHandler() {
        this.defaultHandler = Thread.getDefaultUncaughtExceptionHandler();
    }

    public static void install(Application app) {
        // 1. 接管后台线程异常
        Thread.setDefaultUncaughtExceptionHandler(new CrashHandler());
    }

    @Override
    public void uncaughtException(Thread t, Throwable e) {
        Log.e(TAG, "捕获未处理异常 (Thread: " + t.getName() + ")", e);

        // 如果有默认处理程序，交给它处理（通常会崩溃并显示“应用已停止”）
        if (defaultHandler != null) {
            defaultHandler.uncaughtException(t, e);
        } else {
            // 必须杀掉当前进程，否则可能处于僵死状态
            Process.killProcess(Process.myPid());
            System.exit(10);
        }
    }
}
