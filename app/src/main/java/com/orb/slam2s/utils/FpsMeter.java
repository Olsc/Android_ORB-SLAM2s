package com.orb.slam2s.utils;

import java.text.DecimalFormat;

public class FpsMeter {
    private static final int    STEP              = 20;
    private static final DecimalFormat FPS_FORMAT = new DecimalFormat("0.00");

    private int                 mFramesCounter;
    private double              mFrequency;    // 1e9（nanoTime 以纳秒为单位）
    private long                mprevFrameTime;
    private String              mStrfps;
    boolean                     mIsInitialized = false;

    public void init() {
        mFramesCounter = 0;
        mFrequency = 1.0e9;  // System.nanoTime() 以纳秒为单位
        mprevFrameTime = System.nanoTime();
        mStrfps = "";
    }

    // J-5 配套：本 measure() 调用是否恰好刷新了字符串（每 STEP=20 帧一次），
    // 供调用方决定是否值得投递一次 UI 更新
    private boolean mUpdatedThisCall = false;

    public void measure() {
        mUpdatedThisCall = false;
        if (!mIsInitialized) {
            init();
            mIsInitialized = true;
        } else {
            mFramesCounter++;
            if (mFramesCounter % STEP == 0) {
                long time = System.nanoTime();
                double fps = STEP * mFrequency / (time - mprevFrameTime);
                mprevFrameTime = time;
                mStrfps = FPS_FORMAT.format(fps) + " FPS";
                mUpdatedThisCall = true;
            }
        }
    }

    public boolean isUpdated() {
        return mUpdatedThisCall;
    }

    public String getText() {
        return mStrfps;
    }
}