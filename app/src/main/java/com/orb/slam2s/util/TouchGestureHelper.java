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

import android.content.Context;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;

import java.util.ArrayList;
import java.util.List;

// 触摸手势与双指缩放辅助类
public class TouchGestureHelper {
    private GestureDetector mGestureDetector;
    private ScaleGestureDetector mScaleGestureDetector;
    private final List<ScalingCallback> mScalingCallbacks = new ArrayList<>();

    public interface ScalingCallback {
        void updateScale(float scaleFactor);
    }

    public TouchGestureHelper(Context context) {
        init(context);
    }

    private void init(Context context) {
        mGestureDetector = new GestureDetector(context, new GestureDetector.SimpleOnGestureListener() {
            @Override
            public boolean onSingleTapConfirmed(MotionEvent e) {
                return super.onSingleTapConfirmed(e);
            }

            @Override
            public boolean onScroll(MotionEvent e1, MotionEvent e2, float distanceX, float distanceY) {
                return super.onScroll(e1, e2, distanceX, distanceY);
            }
        });

        mScaleGestureDetector = new ScaleGestureDetector(context, new ScaleGestureDetector.OnScaleGestureListener() {
            @Override
            public boolean onScale(ScaleGestureDetector detector) {
                float scaleFactor = detector.getScaleFactor();
                for (ScalingCallback callback : mScalingCallbacks) {
                    callback.updateScale(scaleFactor);
                }
                return true;
            }

            @Override
            public boolean onScaleBegin(ScaleGestureDetector detector) {
                return true;
            }

            @Override
            public void onScaleEnd(ScaleGestureDetector detector) {
            }
        });
    }

    public boolean handleTouchEvent(MotionEvent event) {
        boolean ret = mScaleGestureDetector.onTouchEvent(event);
        if (!mScaleGestureDetector.isInProgress()) {
            ret = mGestureDetector.onTouchEvent(event);
        }
        return ret;
    }

    public TouchGestureHelper addScalingCallback(ScalingCallback callback) {
        if (callback != null) {
            mScalingCallbacks.add(callback);
        }
        return this;
    }
}