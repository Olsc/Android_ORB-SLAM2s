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

#ifndef GLB_MODEL_H
#define GLB_MODEL_H

#include "glb_common.h"
#include <functional>
#include <vector>
#include <unordered_map>

class GlbModel {
public:
    GlbModel();
    ~GlbModel();

    // 从内存二进制中解析加载 GLB
    bool loadFromMemory(const uint8_t* buffer, size_t size);

    // 在 OpenGL 上下文中上传 GPU 顶点缓冲与纹理
    void uploadGL(std::function<int(const uint8_t*, size_t)> textureLoader);

    // 释放 GPU 缓冲与纹理
    void destroyGL();

    const GlbBounds& getBounds() const { return mBounds; }
    const std::vector<GlbPrimitive>& getPrimitives() const { return mPrimitives; }
    bool isLoaded() const { return mIsLoaded; }
    bool isGPUUploaded() const { return mIsGPUUploaded; }

private:
    void computeNormalsIfMissing(GlbPrimitive& prim);

    bool mIsLoaded = false;
    bool mIsGPUUploaded = false;
    GlbBounds mBounds;
    std::vector<GlbPrimitive> mPrimitives;
    std::vector<uint8_t> mRawBuffer; // 保持内存数据用于纹理提取
    std::unordered_map<int, int> mImageToTextureMap; // imageIndex -> GL textureId 缓存
};

#endif // GLB_MODEL_H
