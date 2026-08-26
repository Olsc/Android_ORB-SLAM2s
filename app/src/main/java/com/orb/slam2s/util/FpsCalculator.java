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

import java.text.DecimalFormat;

// 帧率与性能计算器
public class FpsCalculator {
    private static final int STEP = 20;
    private static final DecimalFormat FPS_FORMAT = new DecimalFormat("0.00");

    private int mFramesCounter;
    private double mFrequency; // 1e9（nanoTime 以纳秒为单位）
    private long mPrevFrameTime;
    private String mFpsString;
    private boolean mIsInitialized = false;
    private boolean mUpdatedThisCall = false;

    public void init() {
        mFramesCounter = 0;
        mFrequency = 1.0e9;
        mPrevFrameTime = System.nanoTime();
        mFpsString = "";
    }

    public void measure() {
        mUpdatedThisCall = false;
        if (!mIsInitialized) {
            init();
            mIsInitialized = true;
        } else {
            mFramesCounter++;
            if (mFramesCounter % STEP == 0) {
                long time = System.nanoTime();
                double fps = STEP * mFrequency / (time - mPrevFrameTime);
                mPrevFrameTime = time;
                mFpsString = FPS_FORMAT.format(fps) + " FPS";
                mUpdatedThisCall = true;
            }
        }
    }

    public boolean isUpdated() {
        return mUpdatedThisCall;
    }

    public String getText() {
        return mFpsString;
    }
}