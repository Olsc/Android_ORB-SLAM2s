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

#ifndef GLB_COMMON_H
#define GLB_COMMON_H

#include <vector>
#include <string>
#include <cstdint>
#include <GLES2/gl2.h>
#include <android/log.h>

#define GLB_LOG_TAG "GlbNative"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, GLB_LOG_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, GLB_LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, GLB_LOG_TAG, __VA_ARGS__)

struct GlbVertex {
    float pos[3];
    float normal[3];
    float uv[2];
};

struct GlbMaterial {
    float baseColor[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    int textureId = 0;
    bool hasTexture = false;
    bool doubleSided = false;

    // 图像二进制块信息（用于在 GL 线程触发纹理加载）
    const uint8_t* imageBytes = nullptr;
    size_t imageSize = 0;
    int imageIndex = -1;
};

struct GlbPrimitive {
    std::vector<GlbVertex> vertices;
    std::vector<uint32_t> indices;
    GlbMaterial material;
    GLuint vbo = 0;
    GLuint ibo = 0;
    GLsizei indexCount = 0;
    GLenum indexType = GL_UNSIGNED_INT;
    float nodeMatrix[16]; // 节点局部到全局的变换矩阵
};

struct GlbBounds {
    float min[3] = {1e9f, 1e9f, 1e9f};
    float max[3] = {-1e9f, -1e9f, -1e9f};
    float center[3] = {0.0f, 0.0f, 0.0f};
    float halfExtent[3] = {0.0f, 0.0f, 0.0f};
    float maxDim = 1.0f;
    float autoScaleFactor = 1.0f;
};

#endif // GLB_COMMON_H
