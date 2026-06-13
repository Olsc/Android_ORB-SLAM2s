package com.orb.slam2s.rendering.gles;

import android.content.Context;
import android.util.AttributeSet;
import android.view.SurfaceView;

/**
 * 具有比例自适应功能的 SurfaceView。
 */
public class FilamentAspectSurfaceView extends SurfaceView {
    private int surfaceWidth;
    private int surfaceHeight;
    private double surfaceRatio;

    public FilamentAspectSurfaceView(Context context) {
        super(context);
    }

    public FilamentAspectSurfaceView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    public void setAspectRatio(int width, int height) {
        if (width < 0 || height < 0) {
            throw new IllegalArgumentException("Size cannot be negative.");
        }
        surfaceWidth = width;
        surfaceHeight = height;
        surfaceRatio = (double) surfaceWidth / surfaceHeight;
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
