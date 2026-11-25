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
import android.graphics.drawable.Drawable;
import android.util.AttributeSet;
import android.view.View;
import android.view.animation.DecelerateInterpolator;
import android.view.animation.OvershootInterpolator;
import android.widget.LinearLayout;

import androidx.annotation.NonNull;

import com.orb.slam2s.R;

// 底栏：选中指示块由自身在 onDraw 中绘制，位置存为字段而非布局属性，菜单卡片重排不会打断流动动画
// 切换时前缘先行、后缘稍晚，指示块拉长后收拢；由布局 XML 反射实例化
@SuppressWarnings("unused")
public class CandyDockLayout extends LinearLayout {
    private static final long FLOW_MS = 380L;
    private static final long FADE_MS = 160L;

    private final Drawable mIndicator;
    private View mSelected;
    private View mShown;
    private float mLeft;
    private float mRight;
    private float mAlpha = 0f;
    private float mScale = 1f;
    private ValueAnimator mFlow;
    private ValueAnimator mFade;

    public CandyDockLayout(Context context) {
        this(context, null);
    }

    public CandyDockLayout(Context context, AttributeSet attrs) {
        super(context, attrs);
        setWillNotDraw(false);
        mIndicator = context.getDrawable(R.drawable.bg_dock_indicator);
    }

    // 选中某个分类；传入 null 表示菜单收起，指示块淡出
    public void select(View tab) {
        View previous = mSelected;
        mSelected = tab;
        if (tab == null) {
            stopFlow();
            animateAlpha(0f, FADE_MS, null);
            return;
        }
        mShown = tab;
        if (tab.getWidth() == 0) return;
        if (previous == null || mAlpha < 0.05f) {
            stopFlow();
            mLeft = tab.getLeft();
            mRight = tab.getRight();
            popIn();
        } else {
            flowTo(tab);
        }
    }

    @Override
    protected void onLayout(boolean changed, int l, int t, int r, int b) {
        super.onLayout(changed, l, t, r, b);
        // 布局变化（旋转、文字宽度变化）后，指示块直接对齐到当前选中项
        if (mSelected != null && (mFlow == null || !mFlow.isRunning())) {
            mLeft = mSelected.getLeft();
            mRight = mSelected.getRight();
        }
    }

    @Override
    protected void onDraw(@NonNull Canvas canvas) {
        super.onDraw(canvas);
        View tab = mShown;
        if (mIndicator == null || tab == null || mAlpha < 0.01f) return;
        int top = tab.getTop();
        int bottom = tab.getBottom();
        mIndicator.setAlpha(Math.round(255f * Math.min(1f, mAlpha)));
        mIndicator.setBounds(Math.round(mLeft), top, Math.round(mRight), bottom);
        canvas.save();
        if (mScale != 1f) {
            canvas.scale(mScale, mScale, (mLeft + mRight) / 2f, (top + bottom) / 2f);
        }
        mIndicator.draw(canvas);
        canvas.restore();
    }

    // 首次出现：原地缩放淡入
    private void popIn() {
        mScale = 0.86f;
        animateAlpha(1f, FADE_MS + 60L, new OvershootInterpolator(1.6f));
    }

    private void animateAlpha(final float to, long ms, OvershootInterpolator scaleCurve) {
        if (mFade != null) {
            mFade.removeAllUpdateListeners();
            mFade.cancel();
        }
        final float from = mAlpha;
        final boolean grow = scaleCurve != null;
        final OvershootInterpolator curve = scaleCurve;
        mFade = ValueAnimator.ofFloat(0f, 1f);
        mFade.setDuration(ms);
        mFade.setInterpolator(new DecelerateInterpolator());
        mFade.addUpdateListener(a -> {
            float t = (Float) a.getAnimatedValue();
            mAlpha = from + (to - from) * t;
            if (grow) mScale = 0.86f + 0.14f * curve.getInterpolation(t);
            invalidate();
        });
        mFade.start();
    }

    private void flowTo(View tab) {
        stopFlow();
        if (mFade != null) {
            mFade.removeAllUpdateListeners();
            mFade.cancel();
        }
        mAlpha = 1f;
        mScale = 1f;

        final float fromL = mLeft;
        final float fromR = mRight;
        final float toL = tab.getLeft();
        final float toR = tab.getRight();
        final boolean rightward = toL >= fromL;
        // 前缘用带过冲的曲线先到位，后缘用缓出曲线稍晚跟上，由此产生拉伸与回收
        final OvershootInterpolator lead = new OvershootInterpolator(1.5f);
        final DecelerateInterpolator trail = new DecelerateInterpolator(1.8f);

        mFlow = ValueAnimator.ofFloat(0f, 1f);
        mFlow.setDuration(FLOW_MS);
        mFlow.addUpdateListener(a -> {
            float t = (Float) a.getAnimatedValue();
            float leadT = lead.getInterpolation(Math.min(1f, t * 1.25f));
            float trailT = trail.getInterpolation(t * t * (3f - 2f * t));
            mLeft = fromL + (toL - fromL) * (rightward ? trailT : leadT);
            mRight = fromR + (toR - fromR) * (rightward ? leadT : trailT);
            invalidate();
        });
        mFlow.start();
    }

    private void stopFlow() {
        if (mFlow != null) {
            mFlow.removeAllUpdateListeners();
            mFlow.cancel();
            mFlow = null;
        }
    }
}