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

#include "glb_renderer.h"
#include <jni.h>

namespace {

// 把 GLB 内嵌的图像二进制交给 Java 侧解码并上传为 GL 纹理，返回纹理 ID（失败为 0）
int loadTextureFromJava(JNIEnv* env, jobject thiz, const uint8_t* imageBytes, size_t imageSize) {
    if (!env || !thiz || !imageBytes || imageSize == 0) return 0;

    jclass cls = env->GetObjectClass(thiz);
    if (!cls) return 0;
    jmethodID mid = env->GetMethodID(cls, "onLoadTextureFromBytes", "([B)I");
    env->DeleteLocalRef(cls);
    if (!mid) return 0;

    jbyteArray jBytes = env->NewByteArray(static_cast<jsize>(imageSize));
    if (!jBytes) return 0;
    env->SetByteArrayRegion(jBytes, 0, static_cast<jsize>(imageSize),
                            reinterpret_cast<const jbyte*>(imageBytes));
    jint texId = env->CallIntMethod(thiz, mid, jBytes);
    env->DeleteLocalRef(jBytes);

    if (env->ExceptionCheck()) {
        env->ExceptionDescribe();
        env->ExceptionClear();
        return 0;
    }
    return static_cast<int>(texId);
}

} // namespace

struct GlbContext {
    std::shared_ptr<GlbModel> model;
    std::unique_ptr<GlbRenderer> renderer;

    GlbContext()
        : model(std::make_shared<GlbModel>()),
          renderer(std::make_unique<GlbRenderer>()) {
        renderer->setModel(model);
    }
};

extern "C" {

JNIEXPORT jlong JNICALL
Java_com_orb_slam2s_graphics_GlbModelRenderer_nativeCreate(JNIEnv* /*env*/, jobject /*thiz*/) {
    auto* ctx = new GlbContext();
    return reinterpret_cast<jlong>(ctx);
}

JNIEXPORT jboolean JNICALL
Java_com_orb_slam2s_graphics_GlbModelRenderer_nativeLoadModel(
        JNIEnv* env, jobject /*thiz*/, jlong handle, jbyteArray dataArray, jint length) {
    auto* ctx = reinterpret_cast<GlbContext*>(handle);
    if (!ctx || !dataArray || length <= 0) return JNI_FALSE;

    jbyte* bytes = env->GetByteArrayElements(dataArray, nullptr);
    bool success = ctx->model->loadFromMemory(reinterpret_cast<const uint8_t*>(bytes),
                                              static_cast<size_t>(length));
    env->ReleaseByteArrayElements(dataArray, bytes, JNI_ABORT);
    return success ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_graphics_GlbModelRenderer_nativeGetBounds(
        JNIEnv* env, jobject /*thiz*/, jlong handle, jfloatArray outBounds) {
    auto* ctx = reinterpret_cast<GlbContext*>(handle);
    if (!ctx || !outBounds) return;

    const auto& b = ctx->model->getBounds();
    jfloat buf[8] = {
        b.center[0], b.center[1], b.center[2],
        b.halfExtent[0], b.halfExtent[1], b.halfExtent[2],
        b.maxDim, b.autoScaleFactor
    };
    env->SetFloatArrayRegion(outBounds, 0, 8, buf);
}

JNIEXPORT jboolean JNICALL
Java_com_orb_slam2s_graphics_GlbModelRenderer_nativeInitGL(
        JNIEnv* env, jobject thiz, jlong handle) {
    auto* ctx = reinterpret_cast<GlbContext*>(handle);
    if (!ctx) return JNI_FALSE;

    if (!ctx->renderer->initGL()) {
        LOGE("GlbModelRenderer: nativeInitGL 失败");
        return JNI_FALSE;
    }

    // 若模型已经在 GL 上下文就绪前加载完成（小模型常见），在这里补上传；
    // 尚未加载完成的，交由 nativeRender 在加载结束后补做。
    ctx->model->uploadGL([&](const uint8_t* imgBytes, size_t imgSize) -> int {
        return loadTextureFromJava(env, thiz, imgBytes, imgSize);
    });

    return JNI_TRUE;
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_graphics_GlbModelRenderer_nativeResize(
        JNIEnv* /*env*/, jobject /*thiz*/, jlong handle, jint width, jint height) {
    auto* ctx = reinterpret_cast<GlbContext*>(handle);
    if (ctx) {
        ctx->renderer->onResize(width, height);
    }
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_graphics_GlbModelRenderer_nativeRender(
        JNIEnv* env, jobject thiz, jlong handle,
        jfloatArray modelM, jfloatArray viewM, jfloatArray projM,
        jfloat scale, jfloat rotX, jfloat rotY) {
    auto* ctx = reinterpret_cast<GlbContext*>(handle);
    if (!ctx) return;

    // 异步加载可能晚于 onSurfaceCreated 完成：这里补做 GPU 上传。
    // uploadGL 内部对"已上传"和"尚无图元"都会立刻返回，逐帧调用开销可忽略。
    if (ctx->model->isLoaded() && !ctx->model->isGPUUploaded()) {
        ctx->model->uploadGL([&](const uint8_t* imgBytes, size_t imgSize) -> int {
            return loadTextureFromJava(env, thiz, imgBytes, imgSize);
        });
    }

    jfloat* mPtr = env->GetFloatArrayElements(modelM, nullptr);
    jfloat* vPtr = env->GetFloatArrayElements(viewM, nullptr);
    jfloat* pPtr = env->GetFloatArrayElements(projM, nullptr);

    ctx->renderer->render(mPtr, vPtr, pPtr, scale, rotX, rotY);

    env->ReleaseFloatArrayElements(modelM, mPtr, JNI_ABORT);
    env->ReleaseFloatArrayElements(viewM, vPtr, JNI_ABORT);
    env->ReleaseFloatArrayElements(projM, pPtr, JNI_ABORT);
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_graphics_GlbModelRenderer_nativeDestroy(
        JNIEnv* /*env*/, jobject /*thiz*/, jlong handle) {
    auto* ctx = reinterpret_cast<GlbContext*>(handle);
    if (ctx) {
        ctx->renderer->destroyGL();
        ctx->model->destroyGL();
        delete ctx;
    }
}

} // extern "C"