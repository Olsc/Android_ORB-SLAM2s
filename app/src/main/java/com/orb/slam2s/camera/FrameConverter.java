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
package com.orb.slam2s.camera;

import android.graphics.Bitmap;
import android.util.Log;

import com.orb.slam2s.graphics.DisplayCorrection;

import java.nio.ByteBuffer;

// 相机 RGBA 帧的单次遍历转换（native，见 app/src/main/cpp/frame/frame_convert_jni.cpp）：
// 灰度直接写进 SLAM 共享内存槽位，ARGB 按显示矫正直接写进预览位图，不经过任何中间 byte[]/int[]。
public final class FrameConverter {
    private static final String TAG = "FrameConverter";
    private static final boolean sAvailable;

    static {
        boolean ok;
        try {
            System.loadLibrary("glb_renderer");
            ok = true;
        } catch (Throwable t) {
            Log.w(TAG, "native 帧转换不可用，回退到 Java 路径", t);
            ok = false;
        }
        sAvailable = ok;
    }

    private FrameConverter() {}

    public static boolean isAvailable() {
        return sAvailable;
    }

    // rgba：CameraX RGBA 平面的 direct ByteBuffer（从其当前 position 起读）；gray：可为 null；bitmap：可为 null。
    // 返回 false 表示本帧未转换（调用方应走 Java 回退路径）
    public static boolean convert(ByteBuffer rgba, int rowStride, int width, int height,
                                  ByteBuffer gray, Bitmap bitmap, DisplayCorrection c) {
        if (!sAvailable || rgba == null || !rgba.isDirect()) return false;
        if (gray != null && !gray.isDirect()) return false;
        if (bitmap != null && c == null) return false;
        return nativeConvert(rgba, rgba.position(), rowStride, width, height, gray, bitmap,
                c != null ? c.ax() : 1, c != null ? c.ay() : 0, c != null ? c.cx() : 0,
                c != null ? c.bx() : 0, c != null ? c.by() : 1, c != null ? c.cy() : 0,
                c != null ? c.displayWidth : width);
    }

    private static native boolean nativeConvert(ByteBuffer rgba, int rgbaPosition, int rowStride,
                                                int width, int height,
                                                ByteBuffer gray, Bitmap bitmap,
                                                int ax, int ay, int cx, int bx, int by, int cy,
                                                int displayWidth);
}
