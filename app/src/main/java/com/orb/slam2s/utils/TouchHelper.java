package com.orb.slam2s.utils;

import android.content.Context;
import android.content.res.Resources;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;

import java.util.ArrayList;


/**
 * Created by Ads on 2016/11/7.
 * 由Olsc于2025/8/25开始进行修改
 */
public class TouchHelper {
    private GestureDetector gestureDetector;
    private ScaleGestureDetector scaleGestureDetector;

    private static final float sDensity =  Resources.getSystem().getDisplayMetrics().density;
    private static final float sDamping = 0.2f;

    private final Context context;
    private ArrayList<ScalingCallback> scalingCallbacks;

    public TouchHelper(Context context) {
        this.context=context;
        init();
    }

    private void init(){
        scalingCallbacks=new ArrayList<>();
        gestureDetector=new GestureDetector(context,new GestureDetector.SimpleOnGestureListener(){

            @Override
            public boolean onSingleTapConfirmed(MotionEvent e) {
                return super.onSingleTapConfirmed(e);
            }

            @Override
            public boolean onScroll(MotionEvent e1, MotionEvent e2, float distanceX, float distanceY) {
                return super.onScroll(e1, e2, distanceX, distanceY);
            }
        });

        scaleGestureDetector=new ScaleGestureDetector(context, new ScaleGestureDetector.OnScaleGestureListener() {
            @Override
            public boolean onScale(ScaleGestureDetector detector) {
                float scaleFactor=detector.getScaleFactor();
                for(ScalingCallback scalingCallback:scalingCallbacks)
                    scalingCallback.updateScale(scaleFactor);
                return true;
            }

            @Override
            public boolean onScaleBegin(ScaleGestureDetector detector) {
                //返回 true 以进入 onScale()
                return true;
            }

            @Override
            public void onScaleEnd(ScaleGestureDetector detector) {

            }
        });
    }

    public boolean handleTouchEvent(MotionEvent event) {
        //int action = event.getActionMasked(); 已注释
        //也可以通过event.getPointerCount()来判断是双指缩放还是单指触控
        boolean ret=scaleGestureDetector.onTouchEvent(event);
        if (!scaleGestureDetector.isInProgress()){
            ret=gestureDetector.onTouchEvent(event);
        }
        return  ret;
    }

    public interface ScalingCallback{
        void updateScale(float scaleFactor);
    }

    public TouchHelper addScalingCallback(ScalingCallback scalingCallback) {
        scalingCallbacks.add(scalingCallback);
        return this;
    }
}
