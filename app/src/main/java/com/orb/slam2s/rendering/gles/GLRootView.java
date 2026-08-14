package com.orb.slam2s.rendering.gles;

import android.content.Context;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;

public class GLRootView extends GLSurfaceView {
    private int surfaceWidth;
    private int surfaceHeight;
    private double surfaceRatio;

    public GLRootView(Context context) {
        super(context);
    }

    public GLRootView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    public void setAspectRatio(int width, int height) {
        if (width < 0 || height < 0) {
            throw new IllegalArgumentException("Size cannot be negative.");
        }
        surfaceWidth = width;
        surfaceHeight = height;
        surfaceRatio=(double)surfaceWidth/surfaceHeight;
        requestLayout();
    }

    @Override
    protected void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {
        int width = MeasureSpec.getSize(widthMeasureSpec);
        int height = MeasureSpec.getSize(heightMeasureSpec);
        int wMode = MeasureSpec.getMode(widthMeasureSpec);
        int hMode = MeasureSpec.getMode(heightMeasureSpec);
        if (wMode == MeasureSpec.EXACTLY && hMode == MeasureSpec.EXACTLY) {
            setMeasuredDimension(width, height);
            return;
        }
        if (0 == surfaceWidth || 0 == surfaceHeight) {
            setMeasuredDimension(width, height);
        } else {
            if (width < height * surfaceRatio) {
                setMeasuredDimension(width, (int) (width / surfaceRatio));
            } else {
                setMeasuredDimension((int) (height * surfaceRatio), height);
            }
        }
    }
}