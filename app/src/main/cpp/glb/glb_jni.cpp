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

    jclass cls = env->GetObjectClass(thiz);
    jmethodID mid = env->GetMethodID(cls, "onLoadTextureFromBytes", "([B)I");

    ctx->model->uploadGL([&](const uint8_t* imgBytes, size_t imgSize) -> int {
        if (!mid || !imgBytes || imgSize == 0) return 0;
        jbyteArray jBytes = env->NewByteArray(static_cast<jsize>(imgSize));
        env->SetByteArrayRegion(jBytes, 0, static_cast<jsize>(imgSize),
                                reinterpret_cast<const jbyte*>(imgBytes));
        jint texId = env->CallIntMethod(thiz, mid, jBytes);
        env->DeleteLocalRef(jBytes);
        return static_cast<int>(texId);
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
        JNIEnv* env, jobject /*thiz*/, jlong handle,
        jfloatArray modelM, jfloatArray viewM, jfloatArray projM,
        jfloat scale, jfloat rotX, jfloat rotY) {
    auto* ctx = reinterpret_cast<GlbContext*>(handle);
    if (!ctx) return;

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
