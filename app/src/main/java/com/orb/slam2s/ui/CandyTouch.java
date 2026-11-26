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

import android.animation.TimeInterpolator;
import android.annotation.SuppressLint;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.view.ViewTreeObserver;
import android.widget.Button;
import android.widget.ImageButton;

// 薄荷糖界面动效：按下收缩、松手轻弹回位，仅用属性动画并在动画期间启用硬件层
public final class CandyTouch {
    private static final float PRESS_SCALE_BUTTON = 0.92f;
    private static final float PRESS_SCALE_ROW = 0.97f;
    private static final float PRESS_SCALE_CARD = 0.97f;
    private static final long PRESS_MS = 70L;
    private static final long RELEASE_MS = 300L;
    private static final long POP_MS = 320L;
    private static final long POP_OUT_MS = 140L;
    private static final long ROW_MS = 220L;
    private static final long ROW_STAGGER_MS = 24L;
    private static final long ENTER_MS = 280L;
    private static final long ENTER_STAGGER_MS = 28L;

    // 阻尼振荡 1 - e^(-kt)cos(wt)：回弹一次后即稳定，用于松手
    private static final TimeInterpolator SPRING = t ->
            (float) (1.0 - Math.exp(-5.0 * t) * Math.cos(2.0 * Math.PI * 1.5 * t));

    // 阻尼更大的弹簧，仅有轻微过冲，用于弹出与入场
    private static final TimeInterpolator SETTLE = t ->
            (float) (1.0 - Math.exp(-7.0 * t) * Math.cos(2.0 * Math.PI * 1.1 * t));

    // 弹出专用：过冲约一成，约 0.3 秒内收敛
    private static final TimeInterpolator POP = t ->
            (float) (1.0 - Math.exp(-5.5 * t) * Math.cos(2.0 * Math.PI * 1.3 * t));

    private static final TimeInterpolator PRESS = t -> 1.0f - (1.0f - t) * (1.0f - t);

    private CandyTouch() {}

    // 递归给子树内的按钮挂载触感，标记为 row 的菜单行收缩幅度更小
    public static void attachAll(View root) {
        if (root instanceof Button || root instanceof ImageButton) {
            attach(root, "row".equals(root.getTag()) ? PRESS_SCALE_ROW : PRESS_SCALE_BUTTON);
        }
        if (root instanceof ViewGroup) {
            ViewGroup group = (ViewGroup) root;
            for (int i = 0; i < group.getChildCount(); i++) {
                attachAll(group.getChildAt(i));
            }
        }
    }

    // 大尺寸可点击卡片使用更小的收缩幅度
    public static void attachCard(View card) {
        attach(card, PRESS_SCALE_CARD);
    }

    @SuppressLint("ClickableViewAccessibility")
    private static void attach(View view, final float pressScale) {
        view.setOnTouchListener((v, event) -> {
            switch (event.getActionMasked()) {
                case MotionEvent.ACTION_DOWN:
                    v.animate().cancel();
                    v.animate().scaleX(pressScale).scaleY(pressScale)
                            .setStartDelay(0L).setDuration(PRESS_MS).setInterpolator(PRESS).start();
                    break;
                case MotionEvent.ACTION_UP:
                case MotionEvent.ACTION_CANCEL:
                    v.animate().cancel();
                    v.animate().scaleX(1f).scaleY(1f)
                            .setStartDelay(0L).setDuration(RELEASE_MS).setInterpolator(SPRING).start();
                    break;
                default:
                    break;
            }
            return false;
        });
    }

    // 菜单卡片从 Dock 正上方放大弹出，rows 内的行随后依次上浮；不启用硬件层，溢出的光晕不会被层边界裁掉
    public static void popIn(final View menu, final ViewGroup rows) {
        final float density = menu.getResources().getDisplayMetrics().density;
        menu.setTag(null);
        menu.animate().cancel();
        menu.setAlpha(0f);
        menu.setTranslationY(18f * density);
        menu.setScaleX(0.7f);
        menu.setScaleY(0.7f);
        if (rows != null) {
            for (int i = 0; i < rows.getChildCount(); i++) {
                View row = rows.getChildAt(i);
                row.animate().cancel();
                row.setAlpha(0f);
                row.setTranslationY(8f * density);
            }
        }
        menu.getViewTreeObserver().addOnPreDrawListener(new ViewTreeObserver.OnPreDrawListener() {
            @Override
            public boolean onPreDraw() {
                menu.getViewTreeObserver().removeOnPreDrawListener(this);
                menu.setPivotX(menu.getWidth() / 2f);
                menu.setPivotY(menu.getHeight());
                menu.animate().alpha(1f).translationY(0f).scaleX(1f).scaleY(1f)
                        .setStartDelay(0L).setDuration(POP_MS).setInterpolator(POP).start();
                if (rows != null) {
                    int shown = 0;
                    for (int i = 0; i < rows.getChildCount(); i++) {
                        View row = rows.getChildAt(i);
                        if (row.getVisibility() != View.VISIBLE) continue;
                        row.animate().alpha(1f).translationY(0f)
                                .setStartDelay(60L + shown * ROW_STAGGER_MS).setDuration(ROW_MS)
                                .setInterpolator(new android.view.animation.DecelerateInterpolator()).start();
                        shown++;
                    }
                }
                return true;
            }
        });
    }

    // 菜单卡片收起：缩小淡出后再隐藏，期间若又被打开则不再隐藏
    public static void popOut(final View menu) {
        final float density = menu.getResources().getDisplayMetrics().density;
        menu.setTag("closing");
        menu.animate().cancel();
        menu.setPivotX(menu.getWidth() / 2f);
        menu.setPivotY(menu.getHeight());
        menu.animate().alpha(0f).translationY(10f * density).scaleX(0.85f).scaleY(0.85f)
                .setStartDelay(0L).setDuration(POP_OUT_MS)
                .setInterpolator(new android.view.animation.AccelerateInterpolator())
                .withEndAction(() -> {
                    if ("closing".equals(menu.getTag())) {
                        menu.setVisibility(View.GONE);
                        menu.setAlpha(1f);
                        menu.setTranslationY(0f);
                        menu.setScaleX(1f);
                        menu.setScaleY(1f);
                        menu.setTag(null);
                    }
                }).start();
    }

    // 列表项入场：自下而上淡入，按位置轻微错开
    public static void enter(View view, int index) {
        float rise = 20f * view.getResources().getDisplayMetrics().density;
        view.animate().cancel();
        view.setAlpha(0f);
        view.setTranslationY(rise);
        view.animate().alpha(1f).translationY(0f)
                .setStartDelay(index * ENTER_STAGGER_MS).setDuration(ENTER_MS)
                .setInterpolator(SETTLE).start();
    }

    // 复用的视图清除动画残留
    public static void reset(View view) {
        view.animate().cancel();
        view.setAlpha(1f);
        view.setTranslationY(0f);
    }
}