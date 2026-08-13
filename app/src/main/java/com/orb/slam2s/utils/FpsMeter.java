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

    public void measure() {
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
            }
        }
    }

    public String getText() {
        return mStrfps;
    }
}