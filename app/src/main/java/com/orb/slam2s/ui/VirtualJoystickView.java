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
package com.orb.slam2s.ui;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.RadialGradient;
import android.graphics.Shader;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

// 虚拟摇杆控件：控制 AR 物体旋转角度与力度
public class VirtualJoystickView extends View {

    private final Paint mBgPaint;
    private final Paint mHandlePaint;
    private final Paint mRingPaint;
    private final Paint mCrossPaint;
    private final Paint mHandleRingPaint;

    private float mCenterX;
    private float mCenterY;
    private float mRadius;
    private float mHandleRadius;

    private float mHandleX;
    private float mHandleY;
    private boolean mIsDragging = false;

    private float mRotationAngle = 0.0f;
    private float mIntensity = 0.0f;

    private OnJoystickListener mListener;

    private static final float HANDLE_RATIO = 0.28f;
    private static final float DEAD_ZONE_RATIO = 0.08f;
    private static final int HANDLE_COLOR = 0xCCFFFFFF;
    private static final int RING_COLOR = 0x60FFFFFF;

    public interface OnJoystickListener {
        void onJoystickUpdate(float angleDeg, float intensity);
    }

    public VirtualJoystickView(Context context) {
        this(context, null);
    }

    public VirtualJoystickView(Context context, AttributeSet attrs) {
        super(context, attrs);

        mBgPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        mBgPaint.setStyle(Paint.Style.FILL);

        mHandlePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        mHandlePaint.setStyle(Paint.Style.FILL);

        mRingPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        mRingPaint.setStyle(Paint.Style.STROKE);
        mRingPaint.setStrokeWidth(2.5f);
        mRingPaint.setColor(RING_COLOR);

        mCrossPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        mCrossPaint.setStyle(Paint.Style.STROKE);
        mCrossPaint.setStrokeWidth(1.5f);
        mCrossPaint.setColor(0x40FFFFFF);

        mHandleRingPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        mHandleRingPaint.setStyle(Paint.Style.STROKE);
        mHandleRingPaint.setStrokeWidth(2.0f);
        mHandleRingPaint.setColor(0x80FFFFFF);

        setVisibility(GONE);
    }

    public void setOnJoystickListener(OnJoystickListener listener) {
        this.mListener = listener;
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);
        mCenterX = w / 2.0f;
        mCenterY = h / 2.0f;
        mRadius = Math.min(w, h) / 2.0f * 0.85f;
        mHandleRadius = mRadius * HANDLE_RATIO;

        mHandleX = mCenterX;
        mHandleY = mCenterY;

        mBgPaint.setShader(new RadialGradient(
                mCenterX, mCenterY, mRadius,
                new int[]{0x30FFFFFF, 0x10FFFFFF, 0x00FFFFFF},
                new float[]{0.0f, 0.6f, 1.0f},
                Shader.TileMode.CLAMP
        ));

        mHandlePaint.setShader(new RadialGradient(
                mCenterX, mCenterY, mHandleRadius,
                new int[]{HANDLE_COLOR, 0x60FFFFFF},
                new float[]{0.0f, 1.0f},
                Shader.TileMode.CLAMP
        ));
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        if (mRadius <= 0) return;

        // 绘制底座背景与圆环
        canvas.drawCircle(mCenterX, mCenterY, mRadius, mBgPaint);
        canvas.drawCircle(mCenterX, mCenterY, mRadius, mRingPaint);
        canvas.drawCircle(mCenterX, mCenterY, mRadius * 0.65f, mRingPaint);

        // 绘制十字参考线
        float crossLen = mRadius * 0.55f;
        canvas.drawLine(mCenterX - crossLen, mCenterY, mCenterX + crossLen, mCenterY, mCrossPaint);
        canvas.drawLine(mCenterX, mCenterY - crossLen, mCenterX, mCenterY + crossLen, mCrossPaint);

        // 绘制手柄
        canvas.drawCircle(mHandleX, mHandleY, mHandleRadius, mHandlePaint);
        canvas.drawCircle(mHandleX, mHandleY, mHandleRadius + 2, mHandleRingPaint);
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        float touchX = event.getX();
        float touchY = event.getY();

        switch (event.getAction()) {
            case MotionEvent.ACTION_DOWN:
                if (isInsideBase(touchX, touchY)) {
                    mIsDragging = true;
                    updateHandlePosition(touchX, touchY);
                    return true;
                }
                return false;

            case MotionEvent.ACTION_MOVE:
                if (mIsDragging) {
                    updateHandlePosition(touchX, touchY);
                    return true;
                }
                break;

            case MotionEvent.ACTION_UP:
            case MotionEvent.ACTION_CANCEL:
                if (mIsDragging) {
                    mIsDragging = false;
                    resetHandle();
                    performClick();
                    return true;
                }
                break;
        }
        return super.onTouchEvent(event);
    }

    // 无障碍要求：重写 performClick 并在触摸事件中调用，使无障碍服务能触发点击
    @Override
    public boolean performClick() {
        return super.performClick();
    }

    private boolean isInsideBase(float x, float y) {
        float dx = x - mCenterX;
        float dy = y - mCenterY;
        return (dx * dx + dy * dy) <= (mRadius * mRadius);
    }

    private void updateHandlePosition(float touchX, float touchY) {
        float dx = touchX - mCenterX;
        float dy = touchY - mCenterY;
        float distance = (float) Math.sqrt(dx * dx + dy * dy);

        if (distance > mRadius) {
            float scale = mRadius / distance;
            dx *= scale;
            dy *= scale;
            distance = mRadius;
        }

        mHandleX = mCenterX + dx;
        mHandleY = mCenterY + dy;

        float deadZone = mRadius * DEAD_ZONE_RATIO;
        if (distance < deadZone) {
            mRotationAngle = 0.0f;
            mIntensity = 0.0f;
        } else {
            float rawAngle = (float) Math.toDegrees(Math.atan2(dy, dx));
            mRotationAngle = (rawAngle + 360.0f) % 360.0f;

            mIntensity = (distance - deadZone) / (mRadius - deadZone);
            mIntensity = Math.min(1.0f, Math.max(0.0f, mIntensity));
        }

        if (mListener != null) {
            mListener.onJoystickUpdate(mRotationAngle, mIntensity);
        }

        invalidate();
    }

    private void resetHandle() {
        mHandleX = mCenterX;
        mHandleY = mCenterY;
        mRotationAngle = 0.0f;
        mIntensity = 0.0f;

        if (mListener != null) {
            mListener.onJoystickUpdate(0.0f, 0.0f);
        }

        invalidate();
    }
}
