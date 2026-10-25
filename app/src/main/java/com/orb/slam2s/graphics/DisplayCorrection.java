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
package com.orb.slam2s.graphics;

import android.opengl.Matrix;

import androidx.annotation.NonNull;

// 显示层几何矫正：把 SLAM 使用的原始相机像素坐标系映射到屏幕显示坐标系。
// 仅用于预览位图、点云与 AR 绘制，不参与 SLAM 计算，也不改变地图坐标系与 MenthaAR 输出。
public final class DisplayCorrection {

    public final int rawWidth;
    public final int rawHeight;

    // 相机帧顺时针旋转到显示方向的角，归一到 0/90/180/270
    public final int rotationDegrees;

    // 显示方向的镜像开关，不进入 SLAM
    public final boolean mirrorH;
    public final boolean mirrorV;

    // 矫正后的画面尺寸，90/270 旋转时长宽互换
    public final int displayWidth;
    public final int displayHeight;

    // 含奇数次反射时三角形绕序翻转，渲染端需交换正反面剔除
    public final boolean flipWinding;

    // 原始像素到显示像素索引的线性映射：dx = ax*x + ay*y + cx，dy = bx*x + by*y + cy
    private final int ax;
    private final int ay;
    private final int cx;
    private final int bx;
    private final int by;
    private final int cy;

    // 4x4 列主序，左乘到投影矩阵（NDC 内的旋转与镜像）
    private final float[] mNdcTransform = new float[16];

    private DisplayCorrection(int rawWidth, int rawHeight, int rotationDegrees,
                              boolean mirrorH, boolean mirrorV) {
        this.rawWidth = rawWidth;
        this.rawHeight = rawHeight;
        this.rotationDegrees = normalizeRotation(rotationDegrees);
        this.mirrorH = mirrorH;
        this.mirrorV = mirrorV;

        final boolean swap = (this.rotationDegrees == 90 || this.rotationDegrees == 270);
        this.displayWidth = swap ? rawHeight : rawWidth;
        this.displayHeight = swap ? rawWidth : rawHeight;
        this.flipWinding = (mirrorH != mirrorV);

        // 原始像素到显示像素的线性映射（未镜像）：
        // 旋转 0:(x,y) 90:(rawH-1-y,x) 180:(rawW-1-x,rawH-1-y) 270:(y,rawW-1-x)
        int lax, lay, lcx, lbx, lby, lcy;
        switch (this.rotationDegrees) {
            case 90:
                lax = 0;  lay = -1; lcx = rawHeight - 1;
                lbx = 1;  lby = 0;  lcy = 0;
                break;
            case 180:
                lax = -1; lay = 0;  lcx = rawWidth - 1;
                lbx = 0;  lby = -1; lcy = rawHeight - 1;
                break;
            case 270:
                lax = 0;  lay = 1;  lcx = 0;
                lbx = -1; lby = 0;  lcy = rawWidth - 1;
                break;
            default:
                lax = 1;  lay = 0;  lcx = 0;
                lbx = 0;  lby = 1;  lcy = 0;
                break;
        }

        // 镜像施加在显示坐标上：dx 取反后加 displayWidth-1，dy 取反后加 displayHeight-1
        if (mirrorH) {
            lax = -lax;
            lay = -lay;
            lcx = displayWidth - 1 - lcx;
        }
        if (mirrorV) {
            lbx = -lbx;
            lby = -lby;
            lcy = displayHeight - 1 - lcy;
        }
        ax = lax; ay = lay; cx = lcx;
        bx = lbx; by = lby; cy = lcy;

        buildNdcTransform();
    }

    public static DisplayCorrection create(int rawWidth, int rawHeight, int rotationDegrees,
                                           boolean mirrorH, boolean mirrorV) {
        return new DisplayCorrection(rawWidth, rawHeight, rotationDegrees, mirrorH, mirrorV);
    }

    // 需要应用旋转或镜像矫正时返回 true
    public boolean needsCorrection() {
        return rotationDegrees != 0 || mirrorH || mirrorV;
    }

    // 返回作用于投影矩阵左侧的 4x4 列主序变换，调用方只读
    public float[] getNdcTransform() {
        return mNdcTransform;
    }

    // 预览位图写入用：第 y 行首像素在显示缓冲中的索引
    public int previewRowBase(int y) {
        return (by * y + cy) * displayWidth + (ay * y + cx);
    }

    // 预览位图写入用：行内每前进一个原始像素，显示缓冲索引的增量
    public int previewRowStep() {
        return bx * displayWidth + ax;
    }

    public static int normalizeRotation(int degrees) {
        int r = degrees % 360;
        if (r < 0) r += 360;
        if (r < 45 || r >= 315) return 0;
        if (r < 135) return 90;
        if (r < 225) return 180;
        return 270;
    }

    // NDC 内的 2x2 变换：先旋转到显示方向，再在显示方向镜像。
    // NDC 的 y 轴朝上，顺时针旋转对应 R(-deg)：90 为 (x,y)->(y,-x)，180 取反，270 为 (-y,x)
    private void buildNdcTransform() {
        float r00, r01, r10, r11;
        switch (rotationDegrees) {
            case 90:
                r00 = 0f;  r01 = 1f;  r10 = -1f; r11 = 0f;
                break;
            case 180:
                r00 = -1f; r01 = 0f;  r10 = 0f;  r11 = -1f;
                break;
            case 270:
                r00 = 0f;  r01 = -1f; r10 = 1f;  r11 = 0f;
                break;
            default:
                r00 = 1f;  r01 = 0f;  r10 = 0f;  r11 = 1f;
                break;
        }
        if (mirrorH) {
            // 左乘 diag(-1,1)，第 0 行取反
            r00 = -r00;
            r01 = -r01;
        }
        if (mirrorV) {
            // 左乘 diag(1,-1)，第 1 行取反
            r10 = -r10;
            r11 = -r11;
        }

        Matrix.setIdentityM(mNdcTransform, 0);
        mNdcTransform[0] = r00;
        mNdcTransform[1] = r10;
        mNdcTransform[4] = r01;
        mNdcTransform[5] = r11;
    }

    @Override
    @NonNull
    public String toString() {
        return "DisplayCorrection{raw=" + rawWidth + "x" + rawHeight
                + ", rot=" + rotationDegrees
                + ", mirrorH=" + mirrorH + ", mirrorV=" + mirrorV
                + ", display=" + displayWidth + "x" + displayHeight
                + ", flipWinding=" + flipWinding + '}';
    }
}