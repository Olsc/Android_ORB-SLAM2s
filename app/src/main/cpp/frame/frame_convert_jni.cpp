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

// 相机帧单次遍历转换：直接读 CameraX 的 RGBA 平面（direct ByteBuffer），
//   - 灰度 (77R+150G+29B)>>8 写进 SLAM 共享内存的目标槽位（与原 Java 实现同一整数公式，逐位一致）；
//   - ARGB 按显示矫正后的目标下标直接写进预览 Bitmap 的像素（AndroidBitmap_lockPixels）。
// 取代原来的 buf.get(rgbaBuf) 整帧拷贝 → Java 双路循环写 yBuf/int[] → setPixels → writeFrame 四次中间拷贝
// （每帧 CPU 写出 16.6 MB → 8.3 MB）。
#include <jni.h>
#include <android/bitmap.h>
#include <cstdint>
#include <cstddef>

extern "C" JNIEXPORT jboolean JNICALL
Java_com_orb_slam2s_camera_FrameConverter_nativeConvert(
        JNIEnv* env, jclass,
        jobject rgbaBuffer, jint rgbaPosition, jint rowStride, jint width, jint height,
        jobject grayBuffer, jobject bitmap,
        jint ax, jint ay, jint cx, jint bx, jint by, jint cy, jint displayWidth)
{
    if (!rgbaBuffer || width <= 0 || height <= 0 || rowStride < width * 4) return JNI_FALSE;
    const uint8_t* rgbaBase = static_cast<const uint8_t*>(env->GetDirectBufferAddress(rgbaBuffer));
    if (!rgbaBase) return JNI_FALSE;
    const jlong rgbaCap = env->GetDirectBufferCapacity(rgbaBuffer);
    // 最后一行可能没有完整的 rowStride 填充，只要求能读到最后一行的 width*4 字节
    if (rgbaCap < (jlong)rgbaPosition + (jlong)(height - 1) * rowStride + (jlong)width * 4) return JNI_FALSE;
    const uint8_t* rgba = rgbaBase + rgbaPosition;

    uint8_t* gray = nullptr;
    if (grayBuffer) {
        gray = static_cast<uint8_t*>(env->GetDirectBufferAddress(grayBuffer));
        if (!gray || env->GetDirectBufferCapacity(grayBuffer) < (jlong)width * height) return JNI_FALSE;
    }

    uint32_t* pixels = nullptr;
    AndroidBitmapInfo info;
    bool bitmapLocked = false;
    if (bitmap) {
        if (AndroidBitmap_getInfo(env, bitmap, &info) != ANDROID_BITMAP_RESULT_SUCCESS) return JNI_FALSE;
        if (info.format != ANDROID_BITMAP_FORMAT_RGBA_8888) return JNI_FALSE;
        // 显示缓冲必须能容纳 displayWidth × (rawWidth 或 rawHeight) 个像素：由调用方保证尺寸与 DisplayCorrection 一致
        if ((jint)info.width != displayWidth) return JNI_FALSE;
        void* p = nullptr;
        if (AndroidBitmap_lockPixels(env, bitmap, &p) != ANDROID_BITMAP_RESULT_SUCCESS || !p) return JNI_FALSE;
        pixels = static_cast<uint32_t*>(p);
        bitmapLocked = true;
    }

    const int rowStep = bx * displayWidth + ax;
    const long pixelCount = bitmapLocked ? (long)info.width * (long)info.height : 0;

    for (int y = 0; y < height; ++y) {
        const uint8_t* src = rgba + (size_t)y * rowStride;
        uint8_t* g = gray ? gray + (size_t)y * width : nullptr;
        long dst = bitmapLocked ? (long)(by * y + cy) * displayWidth + (long)(ay * y + cx) : 0;
        // 该行映射到的显示下标范围 [dst, dst+(width-1)*rowStep] 必须落在位图内
        if (bitmapLocked) {
            const long dstEnd = dst + (long)(width - 1) * rowStep;
            if (dst < 0 || dst >= pixelCount || dstEnd < 0 || dstEnd >= pixelCount) {
                AndroidBitmap_unlockPixels(env, bitmap);
                return JNI_FALSE;
            }
        }
        if (g && bitmapLocked) {
            for (int x = 0; x < width; ++x, src += 4, dst += rowStep) {
                const uint32_t r = src[0], gg = src[1], b = src[2];
                g[x] = (uint8_t)((r * 77u + gg * 150u + b * 29u) >> 8);
                // ARGB_8888 位图在内存中按 R,G,B,A 字节存放（小端 uint32 = 0xAABBGGRR），与 setPixels(0xFFRRGGBB) 结果相同
                pixels[dst] = 0xFF000000u | (b << 16) | (gg << 8) | r;
            }
        } else if (g) {
            for (int x = 0; x < width; ++x, src += 4) {
                g[x] = (uint8_t)((src[0] * 77u + src[1] * 150u + src[2] * 29u) >> 8);
            }
        } else if (bitmapLocked) {
            for (int x = 0; x < width; ++x, src += 4, dst += rowStep) {
                pixels[dst] = 0xFF000000u | ((uint32_t)src[2] << 16) | ((uint32_t)src[1] << 8) | src[0];
            }
        }
    }

    if (bitmapLocked) AndroidBitmap_unlockPixels(env, bitmap);
    return JNI_TRUE;
}
