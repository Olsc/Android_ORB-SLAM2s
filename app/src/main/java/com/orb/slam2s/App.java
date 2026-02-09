package com.orb.slam2s;

import android.app.Application;

import com.orb.slam2s.utils.CrashHandler;

public class App extends Application {
    @Override
    public void onCreate() {
        super.onCreate();
        CrashHandler.install(this);
    }
}