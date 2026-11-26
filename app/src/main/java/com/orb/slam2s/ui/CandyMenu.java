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

import android.animation.Animator;
import android.animation.AnimatorListenerAdapter;
import android.animation.ValueAnimator;
import android.view.View;
import android.view.ViewGroup;
import android.view.animation.OvershootInterpolator;

// 共用菜单卡片：打开与收起沿用同一个外框，切换分类时外框变形为目标尺寸，内容交叉淡入淡出
public final class CandyMenu {
    private static final long MORPH_MS = 340L;

    private final ViewGroup mCard;
    private final ViewGroup[] mGroups;
    private ViewGroup mCurrent;
    private ValueAnimator mMorph;
    private ViewGroup mMorphOld;

    public CandyMenu(ViewGroup card, ViewGroup[] groups) {
        mCard = card;
        mGroups = groups;
    }

    public boolean isShowing(View group) {
        return mCurrent != null && mCurrent == group;
    }

    // 点击已展开的分类则收起，未展开时打开，已有其他分类展开时变形切换
    public void toggle(ViewGroup target) {
        if (target == null) return;
        if (mCurrent == target) {
            close();
        } else if (mCurrent == null) {
            open(target);
        } else {
            morphTo(target);
        }
    }

    private void open(ViewGroup target) {
        finishMorph();
        for (ViewGroup g : mGroups) {
            g.setAlpha(1f);
            g.setTranslationY(0f);
            g.setVisibility(g == target ? View.VISIBLE : View.GONE);
        }
        wrapCard();
        mCard.setVisibility(View.VISIBLE);
        mCurrent = target;
        CandyTouch.popIn(mCard, target);
    }

    private void close() {
        finishMorph();
        mCurrent = null;
        CandyTouch.popOut(mCard);
    }

    private void morphTo(final ViewGroup target) {
        finishMorph();
        final ViewGroup old = mCurrent;
        mCurrent = target;
        mMorphOld = old;

        final int oldW = mCard.getWidth();
        final int oldH = mCard.getHeight();
        // 手动测量目标分组，得到卡片变形后的尺寸
        target.measure(View.MeasureSpec.makeMeasureSpec(0, View.MeasureSpec.UNSPECIFIED),
                View.MeasureSpec.makeMeasureSpec(0, View.MeasureSpec.UNSPECIFIED));
        final int newW = Math.max(oldW, target.getMeasuredWidth() + mCard.getPaddingLeft() + mCard.getPaddingRight());
        final int newH = target.getMeasuredHeight() + mCard.getPaddingTop() + mCard.getPaddingBottom();

        for (ViewGroup g : mGroups) {
            if (g != old && g != target) g.setVisibility(View.GONE);
        }
        final float rise = 8f * mCard.getResources().getDisplayMetrics().density;
        old.setAlpha(1f);
        target.setAlpha(0f);
        target.setTranslationY(rise);
        target.setVisibility(View.VISIBLE);

        // 变形期间卡片尺寸固定为当前值并裁剪内容，新分组不会露出旧外框
        final ViewGroup.LayoutParams lp = mCard.getLayoutParams();
        lp.width = oldW;
        lp.height = oldH;
        mCard.setClipChildren(true);
        mCard.requestLayout();

        final OvershootInterpolator size = new OvershootInterpolator(1.15f);
        mMorph = ValueAnimator.ofFloat(0f, 1f);
        mMorph.setDuration(MORPH_MS);
        mMorph.addUpdateListener(a -> {
            float t = (Float) a.getAnimatedValue();
            float f = size.getInterpolation(t);
            lp.width = Math.round(oldW + (newW - oldW) * f);
            lp.height = Math.round(oldH + (newH - oldH) * f);
            mCard.requestLayout();
            old.setAlpha(1f - clamp01(t / 0.4f));
            float in = clamp01((t - 0.2f) / 0.6f);
            target.setAlpha(in);
            target.setTranslationY((1f - in) * rise);
        });
        mMorph.addListener(new AnimatorListenerAdapter() {
            @Override
            public void onAnimationEnd(Animator animation) {
                finishMorph();
            }
        });
        mMorph.start();
    }

    private static float clamp01(float v) {
        return v < 0f ? 0f : (v > 1f ? 1f : v);
    }

    // 立即结束进行中的变形：落到最终状态，卡片恢复自适应尺寸
    private void finishMorph() {
        if (mMorph != null) {
            mMorph.removeAllUpdateListeners();
            mMorph.removeAllListeners();
            mMorph.cancel();
            mMorph = null;
        }
        if (mMorphOld != null) {
            if (mMorphOld != mCurrent) mMorphOld.setVisibility(View.GONE);
            mMorphOld.setAlpha(1f);
            mMorphOld = null;
        }
        if (mCurrent != null) {
            mCurrent.setAlpha(1f);
            mCurrent.setTranslationY(0f);
        }
        wrapCard();
    }

    private void wrapCard() {
        ViewGroup.LayoutParams lp = mCard.getLayoutParams();
        lp.width = ViewGroup.LayoutParams.WRAP_CONTENT;
        lp.height = ViewGroup.LayoutParams.WRAP_CONTENT;
        mCard.setClipChildren(false);
        mCard.requestLayout();
    }
}