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

#ifndef GLB_RENDERER_H
#define GLB_RENDERER_H

#include "glb_model.h"
#include <memory>

class GlbRenderer {
public:
    GlbRenderer();
    ~GlbRenderer();

    // 初始化 OpenGL ES 着色器程序及渲染状态
    bool initGL();

    // 视口尺寸调整
    void onResize(int width, int height);

    // 设置模型实例
    void setModel(std::shared_ptr<GlbModel> model);

    // 渲染一帧
    void render(const float* slamModelMatrix,
                const float* viewMatrix,
                const float* projMatrix,
                float scale,
                float userRotationX,
                float userRotationY);

    // 销毁并释放 OpenGL 资源
    void destroyGL();

private:
    bool compileShaders();

    std::shared_ptr<GlbModel> mModel;
    int mWidth = 0;
    int mHeight = 0;
    bool mIsGLReady = false;

    GLuint mProgram = 0;
    GLint mLocPosition = -1;
    GLint mLocNormal = -1;
    GLint mLocTexCoord = -1;

    GLint mLocMVPMatrix = -1;
    GLint mLocModelMatrix = -1;
    GLint mLocNormalMatrix = -1;

    GLint mLocTexture = -1;
    GLint mLocHasTexture = -1;
    GLint mLocBaseColor = -1;

    GLint mLocLightDir1 = -1;
    GLint mLocLightColor1 = -1;
    GLint mLocLightDir2 = -1;
    GLint mLocLightColor2 = -1;
    GLint mLocAmbientColor = -1;
    GLint mLocCameraPos = -1;
};

#endif // GLB_RENDERER_H
