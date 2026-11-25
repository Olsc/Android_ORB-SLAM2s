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

import android.animation.ValueAnimator;
import android.content.Context;
import android.graphics.Canvas;
import android.graphics.LinearGradient;
import android.graphics.Paint;
import android.graphics.Shader;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;
import android.view.animation.DecelerateInterpolator;
import android.view.animation.OvershootInterpolator;

import androidx.annotation.NonNull;

// 虚拟摇杆控件：控制 AR 物体旋转角度与力度
// 由布局 XML 反射实例化，构造函数不会在源码中直接调用
@SuppressWarnings("unused")
public class VirtualJoystickView extends View {

    private static final float HANDLE_RATIO = 0.28f;
    private static final float DEAD_ZONE_RATIO = 0.08f;
    private static final long RESET_MS = 320L;
    private static final long DRAG_IN_MS = 140L;
    private static final long DRAG_OUT_MS = 260L;

    private float mDensity = 1f;
    private Disc mBase;
    private Disc mHandle;

    private float mCenterX;
    private float mCenterY;
    private float mRadius;
    private float mHandleRadius;

    private float mHandleX;
    private float mHandleY;
    private boolean mIsDragging = false;

    private float mRotationAngle = 0.0f;
    private float mIntensity = 0.0f;
    // 拖动程度 0-1，驱动底座与手柄的外发光强度
    private float mDrag = 0.0f;
    private ValueAnimator mResetAnim;
    private ValueAnimator mDragAnim;

    private OnJoystickListener mListener;

    public interface OnJoystickListener {
        void onJoystickUpdate(float angleDeg, float intensity);
    }

    public VirtualJoystickView(Context context) {
        this(context, null);
    }

    public VirtualJoystickView(Context context, AttributeSet attrs) {
        super(context, attrs);
        mDensity = context.getResources().getDisplayMetrics().density;
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

        mBase = new Disc(mRadius, mDensity);
        mHandle = new Disc(mHandleRadius, mDensity);
    }

    @Override
    protected void onDraw(@NonNull Canvas canvas) {
        super.onDraw(canvas);
        if (mRadius <= 0 || mBase == null) return;

        // 底座：近乎透明，拖动时外发光亮起；手柄：更亮的玻璃珠，始终带一圈柔光
        mBase.draw(canvas, mCenterX, mCenterY, 0.22f, 1.0f, 0.8f, 0.6f * mDrag, 1.0f);
        mHandle.draw(canvas, mHandleX, mHandleY, 0.10f, 1.7f, 1.2f, 0.35f + 0.65f * mDrag, 1.0f);
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        float touchX = event.getX();
        float touchY = event.getY();

        switch (event.getAction()) {
            case MotionEvent.ACTION_DOWN:
                if (isInsideBase(touchX, touchY)) {
                    mIsDragging = true;
                    animateDrag(1f, DRAG_IN_MS);
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
                    animateDrag(0f, DRAG_OUT_MS);
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
    @SuppressWarnings("EmptyMethod")
    public boolean performClick() {
        return super.performClick();
    }

    private boolean isInsideBase(float x, float y) {
        float dx = x - mCenterX;
        float dy = y - mCenterY;
        return (dx * dx + dy * dy) <= (mRadius * mRadius);
    }

    private void updateHandlePosition(float touchX, float touchY) {
        if (mResetAnim != null) mResetAnim.cancel();

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

    // 角度与力度立即归零并通知；手柄位置仅在视觉上带回弹地回到中心
    private void resetHandle() {
        mRotationAngle = 0.0f;
        mIntensity = 0.0f;

        if (mListener != null) {
            mListener.onJoystickUpdate(0.0f, 0.0f);
        }

        if (mResetAnim != null) mResetAnim.cancel();
        final float fromX = mHandleX;
        final float fromY = mHandleY;
        mResetAnim = ValueAnimator.ofFloat(0f, 1f);
        mResetAnim.setDuration(RESET_MS);
        mResetAnim.setInterpolator(new OvershootInterpolator(2.2f));
        mResetAnim.addUpdateListener(a -> {
            float t = (Float) a.getAnimatedValue();
            mHandleX = fromX + (mCenterX - fromX) * t;
            mHandleY = fromY + (mCenterY - fromY) * t;
            invalidate();
        });
        mResetAnim.start();
    }

    private void animateDrag(final float to, long ms) {
        if (mDragAnim != null) mDragAnim.cancel();
        final float from = mDrag;
        mDragAnim = ValueAnimator.ofFloat(0f, 1f);
        mDragAnim.setDuration(ms);
        mDragAnim.setInterpolator(new DecelerateInterpolator());
        mDragAnim.addUpdateListener(a -> {
            mDrag = from + (to - from) * (Float) a.getAnimatedValue();
            invalidate();
        });
        mDragAnim.start();
    }

    @Override
    protected void onDetachedFromWindow() {
        if (mResetAnim != null) mResetAnim.cancel();
        if (mDragAnim != null) mDragAnim.cancel();
        super.onDetachedFromWindow();
    }

    // 薄荷糖圆盘：近乎透明的体、顶部柔和高光、渐变细边、向内与向外的渐隐发光
    private static final class Disc {
        private final float mR;
        private final float mW;
        private final Paint mScrim = new Paint(Paint.ANTI_ALIAS_FLAG);
        private final Paint mFill = new Paint(Paint.ANTI_ALIAS_FLAG);
        private final Paint mSpec = new Paint(Paint.ANTI_ALIAS_FLAG);
        private final Paint mSoft = new Paint(Paint.ANTI_ALIAS_FLAG);
        private final Paint mHair = new Paint(Paint.ANTI_ALIAS_FLAG);
        private final Paint mInner = new Paint(Paint.ANTI_ALIAS_FLAG);
        private final Paint mHalo = new Paint(Paint.ANTI_ALIAS_FLAG);

        Disc(float radius, float density) {
            mR = radius;
            mW = density;
            mScrim.setColor(0xFF000000);
            // 渐变均在以圆心为原点的局部坐标内，绘制时平移画布即可，手柄移动无需重建
            mFill.setShader(new LinearGradient(0f, -radius, 0f, radius,
                    0x1CFFFFFF, 0x06FFFFFF, Shader.TileMode.CLAMP));
            mSpec.setShader(new LinearGradient(0f, -radius, 0f, -radius + radius * 1.1f,
                    0x2CFFFFFF, 0x00FFFFFF, Shader.TileMode.CLAMP));
            LinearGradient rim = new LinearGradient(0f, -radius, 0f, radius,
                    new int[]{0xFFFFFFFF, 0x1AFFFFFF, 0x1AFFFFFF, 0x80FFFFFF},
                    new float[]{0f, 0.42f, 0.66f, 1f}, Shader.TileMode.CLAMP);
            mSoft.setStyle(Paint.Style.STROKE);
            mSoft.setStrokeWidth(2f * density);
            mSoft.setShader(rim);
            mHair.setStyle(Paint.Style.STROKE);
            mHair.setStrokeWidth(density);
            mHair.setShader(rim);
            mInner.setStyle(Paint.Style.STROKE);
            mInner.setStrokeWidth(density);
            mInner.setColor(0xFFFFFFFF);
            mHalo.setStyle(Paint.Style.STROKE);
            mHalo.setStrokeWidth(density);
            mHalo.setColor(0xFFFFFFFF);
        }

        void draw(Canvas canvas, float cx, float cy, float scrim, float fill,
                  float inner, float halo, float rim) {
            canvas.save();
            canvas.translate(cx, cy);

            if (halo > 0f) {
                final int layers = 28;
                for (int i = 0; i < layers; i++) {
                    float fall = 1f - (i + 1f) / (layers + 1f);
                    mHalo.setAlpha(a(255f * 0.075f * halo * fall * fall));
                    canvas.drawCircle(0f, 0f, mR + (i + 0.5f) * mW * 0.5f, mHalo);
                }
            }

            mScrim.setAlpha(a(255f * scrim));
            canvas.drawCircle(0f, 0f, mR, mScrim);
            mFill.setAlpha(a(255f * fill));
            canvas.drawCircle(0f, 0f, mR, mFill);
            mSpec.setAlpha(a(255f * 0.9f));
            canvas.drawCircle(0f, 0f, mR, mSpec);

            mSoft.setAlpha(a(255f * 0.09f * rim));
            canvas.drawCircle(0f, 0f, mR - mSoft.getStrokeWidth() / 2f, mSoft);
            mHair.setAlpha(a(255f * 0.72f * rim));
            canvas.drawCircle(0f, 0f, mR - mHair.getStrokeWidth() / 2f, mHair);

            final int layers = 26;
            for (int i = 0; i < layers; i++) {
                float inset = (i + 0.5f) * mW * 0.5f;
                if (inset >= mR) break;
                float fall = 1f - (i + 1f) / (layers + 1f);
                mInner.setAlpha(a(255f * 0.085f * inner * fall * fall));
                canvas.drawCircle(0f, 0f, mR - inset, mInner);
            }
            canvas.restore();
        }

        private static int a(float v) {
            return v < 0f ? 0 : (v > 255f ? 255 : Math.round(v));
        }
    }
}