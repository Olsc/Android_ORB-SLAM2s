package com.orb.slam2s.ui;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RadialGradient;
import android.graphics.Shader;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

/**
 * 虚拟摇杆控件 — 用于控制AR物体的Y轴旋转。
 *
 * 交互方式：
 * - 触摸并拖动手柄绕中心旋转
 * - 手柄偏离中心的角度 = AR物体的旋转角度
 * - 手柄偏离中心的距离 = 旋转速度系数（0~1）
 * - 松手后手柄弹回中心（停止旋转）
 *
 * 只在AR物体存在时由Activity控制显隐。
 */
public class JoystickView extends View {

    private static final String TAG = "JoystickView";

    // 绘制元素
    private final Paint mBgPaint;
    private final Paint mHandlePaint;
    private final Paint mRingPaint;
    private final Paint mCrossPaint;
    private final Paint mHandleRingPaint;  // 复用避免onDraw中分配

    // 几何尺寸
    private float mCenterX;
    private float mCenterY;
    private float mRadius;          // 摇杆底座半径
    private float mHandleRadius;    // 手柄半径

    // 手柄状态
    private float mHandleX;
    private float mHandleY;
    private boolean mIsDragging = false;

    // 输出值
    private float mRotationAngle = 0.0f;   // 0~360度，绕Y轴旋转
    private float mIntensity = 0.0f;        // 0~1，旋转力度

    // 回调
    private OnJoystickListener mListener;

    // 常量
    private static final float HANDLE_RATIO = 0.28f;    // 手柄半径 / 底座半径
    private static final float DEAD_ZONE_RATIO = 0.08f; // 死区半径 / 底座半径
    private static final int DEFAULT_COLOR = 0x80FFFFFF; // 半透明白色
    private static final int HANDLE_COLOR = 0xCCFFFFFF;  // 较亮白色
    private static final int RING_COLOR = 0x60FFFFFF;    // 淡白色环

    public interface OnJoystickListener {
        /**
         * 摇杆值变化回调
         * @param angleDeg 手柄偏离中心的角度（0~360度，0=右，90=上）
         * @param intensity 偏离力度（0~1，0=中心死区）
         */
        void onJoystickUpdate(float angleDeg, float intensity);
    }

    public JoystickView(Context context) {
        this(context, null);
    }

    public JoystickView(Context context, AttributeSet attrs) {
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

        // 默认不可见（AR物体出现时由Activity设为VISIBLE）
        setVisibility(GONE);
    }

    public void setOnJoystickListener(OnJoystickListener listener) {
        mListener = listener;
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);
        mCenterX = w / 2.0f;
        mCenterY = h / 2.0f;
        mRadius = Math.min(w, h) / 2.0f * 0.85f;
        mHandleRadius = mRadius * HANDLE_RATIO;

        // 初始化手柄位置在中心
        mHandleX = mCenterX;
        mHandleY = mCenterY;

        // 创建背景渐变
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

        // 绘制底座背景
        canvas.drawCircle(mCenterX, mCenterY, mRadius, mBgPaint);

        // 绘制底座圆环
        canvas.drawCircle(mCenterX, mCenterY, mRadius, mRingPaint);
        canvas.drawCircle(mCenterX, mCenterY, mRadius * 0.65f, mRingPaint);

        // 绘制十字参考线
        float crossLen = mRadius * 0.55f;
        canvas.drawLine(mCenterX - crossLen, mCenterY, mCenterX + crossLen, mCenterY, mCrossPaint);
        canvas.drawLine(mCenterX, mCenterY - crossLen, mCenterX, mCenterY + crossLen, mCrossPaint);

        // 绘制手柄
        canvas.drawCircle(mHandleX, mHandleY, mHandleRadius, mHandlePaint);

        // 手柄外环（复用预分配的Paint）
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
                    return true;
                }
                break;
        }
        return super.onTouchEvent(event);
    }

    /**
     * 判断触摸点是否在摇杆底座范围内
     */
    private boolean isInsideBase(float x, float y) {
        float dx = x - mCenterX;
        float dy = y - mCenterY;
        return (dx * dx + dy * dy) <= (mRadius * mRadius);
    }

    /**
     * 更新手柄位置并计算输出值
     */
    private void updateHandlePosition(float touchX, float touchY) {
        float dx = touchX - mCenterX;
        float dy = touchY - mCenterY;
        float distance = (float) Math.sqrt(dx * dx + dy * dy);

        // 限制在底座范围内
        if (distance > mRadius) {
            float scale = mRadius / distance;
            dx *= scale;
            dy *= scale;
            distance = mRadius;
        }

        mHandleX = mCenterX + dx;
        mHandleY = mCenterY + dy;

        // 检查死区
        float deadZone = mRadius * DEAD_ZONE_RATIO;
        if (distance < deadZone) {
            mRotationAngle = 0.0f;
            mIntensity = 0.0f;
        } else {
            // 计算角度：atan2(dy, dx)，0=右，顺时针增加
            // 用户左右拖拽 = Y轴旋转（水平拖拽控制绕Y轴旋转）
            float rawAngle = (float) Math.toDegrees(Math.atan2(dy, dx));
            // 规范化到 0~360
            mRotationAngle = (rawAngle + 360.0f) % 360.0f;

            // 力度：死区外线性 0~1
            mIntensity = (distance - deadZone) / (mRadius - deadZone);
            mIntensity = Math.min(1.0f, Math.max(0.0f, mIntensity));
        }

        // 回调
        if (mListener != null) {
            mListener.onJoystickUpdate(mRotationAngle, mIntensity);
        }

        invalidate();
    }

    /**
     * 松手后手柄弹回中心
     */
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

    /**
     * 获取当前旋转角度（度，0~360）
     */
    public float getRotationAngle() {
        return mRotationAngle;
    }

    /**
     * 获取当前力度（0~1）
     */
    public float getIntensity() {
        return mIntensity;
    }
}
