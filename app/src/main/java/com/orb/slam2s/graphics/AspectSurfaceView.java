package com.orb.slam2s.graphics;

import android.content.Context;
import android.util.AttributeSet;
import android.view.SurfaceView;

// 宽高比自适应 SurfaceView（专供 Filament / 自定义 Surface 渲染）
public class AspectSurfaceView extends SurfaceView {
    private int surfaceWidth;
    private int surfaceHeight;
    private double surfaceRatio;

    public AspectSurfaceView(Context context) {
        super(context);
    }

    public AspectSurfaceView(Context context, AttributeSet attrs) {
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
