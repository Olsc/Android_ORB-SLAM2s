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
