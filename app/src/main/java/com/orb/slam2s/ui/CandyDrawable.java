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
import android.content.res.Resources;
import android.content.res.TypedArray;
import android.graphics.Canvas;
import android.graphics.ColorFilter;
import android.graphics.LinearGradient;
import android.graphics.Paint;
import android.graphics.PixelFormat;
import android.graphics.Rect;
import android.graphics.RectF;
import android.graphics.Shader;
import android.graphics.drawable.Drawable;
import android.util.AttributeSet;
import android.view.animation.DecelerateInterpolator;

import androidx.annotation.NonNull;

import com.orb.slam2s.R;

import org.xmlpull.v1.XmlPullParser;
import org.xmlpull.v1.XmlPullParserException;

import java.io.IOException;

// 薄荷糖背景：近乎透明的底、沿光照方向变化的边缘高光与柔和镜面光斑，按下时整体提亮
// 由 drawable XML 的 class 属性反射实例化，纯 Canvas 绘制，不依赖模糊与离屏缓冲
@SuppressWarnings("unused")
public class CandyDrawable extends Drawable {
    private static final long GLOW_MS = 140L;
    private static final long TINT_MS = 260L;

    private final Paint mBasePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint mScrimPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint mFillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint mTintPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint mSpecPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint mSoftRimPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint mRimPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint mHaloPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint mInnerPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final RectF mRect = new RectF();

    private float mDensity = 1f;
    private float mRadius = 0f;
    private float mInset = 0f;
    private float mScrim = 0f;
    private float mFill = 1f;
    private float mRim = 1f;
    private int mTint = 0xFF0A9DFF;
    private float mTintBase = 0f;
    private int mBase = 0;
    private float mHalo = 0f;
    private float mInner = 0f;
    private float mSpec = 1f;
    private float mShaderTop = Float.NaN;
    private float mShaderBottom = Float.NaN;
    private int mMaster = 255;

    private float mGlow = 0f;
    private float mActive = 0f;
    private boolean mPressed = false;
    private boolean mActivated = false;
    private ValueAnimator mGlowAnim;
    private ValueAnimator mActiveAnim;

    public CandyDrawable() {
        mFillPaint.setStyle(Paint.Style.FILL);
        mSoftRimPaint.setStyle(Paint.Style.STROKE);
        mRimPaint.setStyle(Paint.Style.STROKE);
        mHaloPaint.setStyle(Paint.Style.STROKE);
        mHaloPaint.setColor(0xFFFFFFFF);
        mInnerPaint.setStyle(Paint.Style.STROKE);
        mInnerPaint.setColor(0xFFFFFFFF);
    }

    @Override
    public void inflate(@NonNull Resources r, @NonNull XmlPullParser parser, @NonNull AttributeSet attrs,
                        Resources.Theme theme) throws XmlPullParserException, IOException {
        super.inflate(r, parser, attrs, theme);
        mDensity = r.getDisplayMetrics().density;
        TypedArray a = theme != null
                ? theme.obtainStyledAttributes(attrs, R.styleable.CandyDrawable, 0, 0)
                : r.obtainAttributes(attrs, R.styleable.CandyDrawable);
        try {
            mRadius = a.getDimension(R.styleable.CandyDrawable_candyRadius, 16f * mDensity);
            mInset = a.getDimension(R.styleable.CandyDrawable_candyInset, 0f);
            mScrim = a.getFloat(R.styleable.CandyDrawable_candyScrim, 0f);
            mFill = a.getFloat(R.styleable.CandyDrawable_candyFill, 1f);
            mRim = a.getFloat(R.styleable.CandyDrawable_candyRim, 1f);
            mTint = a.getColor(R.styleable.CandyDrawable_candyTint, 0xFF0A9DFF);
            mTintBase = a.getFloat(R.styleable.CandyDrawable_candyTintLevel, 0f);
            mBase = a.getColor(R.styleable.CandyDrawable_candyBase, 0);
            mHalo = a.getFloat(R.styleable.CandyDrawable_candyGlow, 0f);
            mInner = a.getFloat(R.styleable.CandyDrawable_candyInnerGlow, 0f);
            mSpec = a.getFloat(R.styleable.CandyDrawable_candySpec, 1f);
        } finally {
            a.recycle();
        }
        mBasePaint.setColor(mBase);
        mScrimPaint.setColor(0xFF000000);
        mHaloPaint.setStrokeWidth(1.0f * mDensity);
        mInnerPaint.setStrokeWidth(1.0f * mDensity);
        mTintPaint.setColor(mTint);
        mSoftRimPaint.setStrokeWidth(2f * mDensity);
        mRimPaint.setStrokeWidth(1f * mDensity);
    }

    @Override
    protected void onBoundsChange(Rect b) {
        super.onBoundsChange(b);
        mRect.set(b.left + mInset, b.top + mInset, b.right - mInset, b.bottom - mInset);
        float w = mRect.width();
        float h = mRect.height();
        if (w <= 0f || h <= 0f) return;
        // 三组渐变都只沿垂直方向，宽度变化（如流动指示块）无需重建
        if (mRect.top == mShaderTop && mRect.bottom == mShaderBottom) return;
        mShaderTop = mRect.top;
        mShaderBottom = mRect.bottom;

        // 填充：顶部略亮、底部几乎透明，左右完全对称
        mFillPaint.setShader(new LinearGradient(0f, mRect.top, 0f, mRect.bottom,
                0x1CFFFFFF, 0x06FFFFFF, Shader.TileMode.CLAMP));
        // 高光：仅覆盖上半部、水平方向均匀的柔和反光
        mSpecPaint.setShader(new LinearGradient(0f, mRect.top, 0f, mRect.top + h * 0.55f,
                0x2CFFFFFF, 0x00FFFFFF, Shader.TileMode.CLAMP));
        // 边缘高光：顶部最亮、底部次之、中段几乎消失
        LinearGradient rim = new LinearGradient(0f, mRect.top, 0f, mRect.bottom,
                new int[]{0xFFFFFFFF, 0x1AFFFFFF, 0x1AFFFFFF, 0x80FFFFFF},
                new float[]{0f, 0.42f, 0.66f, 1f}, Shader.TileMode.CLAMP);
        mSoftRimPaint.setShader(rim);
        mRimPaint.setShader(rim);
    }

    @Override
    public void draw(@NonNull Canvas canvas) {
        if (mRect.isEmpty()) return;
        float r = Math.min(mRadius, Math.min(mRect.width(), mRect.height()) / 2f);
        float g = mGlow;
        // 着色只由显式配置的层级决定，激活态本身不再自动染色，选中效果由发光层承担
        float tint = mTintBase;

        drawHalo(canvas, r);

        if ((mBase >>> 24) != 0) {
            mBasePaint.setColor(mBase);
            mBasePaint.setAlpha(al((mBase >>> 24) & 0xFF));
            canvas.drawRoundRect(mRect, r, r, mBasePaint);
        }
        if (mScrim > 0f) {
            mScrimPaint.setAlpha(al(255f * mScrim));
            canvas.drawRoundRect(mRect, r, r, mScrimPaint);
        }

        mFillPaint.setAlpha(al(255f * mFill * (1f + 1.6f * g)));
        canvas.drawRoundRect(mRect, r, r, mFillPaint);

        if (tint > 0f) {
            mTintPaint.setAlpha(al(255f * 0.34f * tint));
            canvas.drawRoundRect(mRect, r, r, mTintPaint);
        }

        mSpecPaint.setAlpha(al(255f * mSpec * (0.9f + 0.9f * g)));
        canvas.drawRoundRect(mRect, r, r, mSpecPaint);

        // 描边内缩半个线宽，确保完整落在形状内
        float soft = mSoftRimPaint.getStrokeWidth();
        mSoftRimPaint.setAlpha(al(255f * 0.09f * mRim * (1f + 0.8f * g)));
        canvas.drawRoundRect(mRect.left + soft / 2f, mRect.top + soft / 2f,
                mRect.right - soft / 2f, mRect.bottom - soft / 2f,
                Math.max(0f, r - soft / 2f), Math.max(0f, r - soft / 2f), mSoftRimPaint);

        float hair = mRimPaint.getStrokeWidth();
        mRimPaint.setAlpha(al(255f * 0.72f * mRim));
        canvas.drawRoundRect(mRect.left + hair / 2f, mRect.top + hair / 2f,
                mRect.right - hair / 2f, mRect.bottom - hair / 2f,
                Math.max(0f, r - hair / 2f), Math.max(0f, r - hair / 2f), mRimPaint);

        drawInnerGlow(canvas, r);
    }

    // 内发光：从边缘向内逐层衰减的白光，中心保持通透，边缘像是在透光而不是一圈描边
    private void drawInnerGlow(Canvas canvas, float r) {
        if (mInner <= 0f) return;
        final int layers = 26;
        float w = mInnerPaint.getStrokeWidth();
        for (int i = 0; i < layers; i++) {
            float inset = (i + 0.5f) * w * 0.5f;
            if (inset * 2f >= Math.min(mRect.width(), mRect.height())) break;
            float fall = 1f - (i + 1f) / (layers + 1f);
            mInnerPaint.setAlpha(al(255f * 0.085f * mInner * fall * fall));
            canvas.drawRoundRect(mRect.left + inset, mRect.top + inset,
                    mRect.right - inset, mRect.bottom - inset,
                    Math.max(0f, r - inset), Math.max(0f, r - inset), mInnerPaint);
        }
    }

    // 外发光：在形状外侧画若干圈逐层衰减的白色描边，靠近的相邻控件一侧会被一并照亮
    // 需要祖先容器关闭 clipChildren 才能溢出到自身范围之外
    private void drawHalo(Canvas canvas, float r) {
        if (mHalo <= 0f) return;
        final int layers = 28;
        float w = mHaloPaint.getStrokeWidth();
        for (int i = 0; i < layers; i++) {
            float spread = (i + 0.5f) * w * 0.5f;
            float fall = 1f - (i + 1f) / (layers + 1f);
            mHaloPaint.setAlpha(al(255f * 0.075f * mHalo * fall * fall));
            canvas.drawRoundRect(mRect.left - spread, mRect.top - spread,
                    mRect.right + spread, mRect.bottom + spread, r + spread, r + spread, mHaloPaint);
        }
    }

    // 叠加整体透明度后转为 0-255，供选择器的淡入淡出使用
    private int al(float v) {
        float x = v * mMaster / 255f;
        return x < 0f ? 0 : (x > 255f ? 255 : Math.round(x));
    }

    @Override
    public boolean isStateful() {
        return true;
    }

    @Override
    protected boolean onStateChange(int[] state) {
        boolean pressed = false;
        boolean activated = false;
        for (int s : state) {
            if (s == android.R.attr.state_pressed) pressed = true;
            if (s == android.R.attr.state_activated) activated = true;
        }
        boolean changed = pressed != mPressed || activated != mActivated;
        mPressed = pressed;
        mActivated = activated;
        if (changed) {
            mGlowAnim = retarget(mGlowAnim, mGlow, pressed ? 1f : 0f, GLOW_MS, true);
            mActiveAnim = retarget(mActiveAnim, mActive, activated ? 1f : 0f, TINT_MS, false);
        }
        return changed;
    }

    // 将光感或着色平滑过渡到目标值，未显示时直接落位
    private ValueAnimator retarget(ValueAnimator old, float from, float to, long ms, final boolean glow) {
        if (old != null) old.cancel();
        if (!isVisible() || getBounds().isEmpty()) {
            setLevel(glow, to);
            return null;
        }
        ValueAnimator va = ValueAnimator.ofFloat(from, to);
        va.setDuration(ms);
        va.setInterpolator(new DecelerateInterpolator());
        va.addUpdateListener(a -> setLevel(glow, (Float) a.getAnimatedValue()));
        va.start();
        return va;
    }

    private void setLevel(boolean glow, float v) {
        if (glow) mGlow = v; else mActive = v;
        invalidateSelf();
    }

    @Override
    public boolean getPadding(@NonNull Rect padding) {
        // 对话框类大圆角背景需要额外的上下内边距，避免内容贴近圆角
        int v = mInset > 0f ? Math.round(10f * mDensity) : 0;
        padding.set(0, v, 0, v);
        return v > 0;
    }

    @Override
    public void setAlpha(int alpha) {
        if (alpha != mMaster) {
            mMaster = alpha;
            invalidateSelf();
        }
    }

    @Override
    public int getAlpha() {
        return mMaster;
    }

    @Override
    public void setColorFilter(ColorFilter colorFilter) {}

    @Override
    public int getOpacity() {
        return PixelFormat.TRANSLUCENT;
    }
}