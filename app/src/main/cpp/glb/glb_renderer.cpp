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
#include <cmath>
#include <cstring>

namespace {

// 矩阵计算辅助（符合 OpenGL 列主序 Column-Major 规范）
void matrixIdentity(float* m) {
    memset(m, 0, 16 * sizeof(float));
    m[0] = m[5] = m[10] = m[15] = 1.0f;
}

void matrixMultiply(float* r, const float* a, const float* b) {
    float temp[16];
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            temp[col * 4 + row] =
                a[0 * 4 + row] * b[col * 4 + 0] +
                a[1 * 4 + row] * b[col * 4 + 1] +
                a[2 * 4 + row] * b[col * 4 + 2] +
                a[3 * 4 + row] * b[col * 4 + 3];
        }
    }
    memcpy(r, temp, 16 * sizeof(float));
}

void matrixScale(float* m, float sx, float sy, float sz) {
    float s[16];
    matrixIdentity(s);
    s[0] = sx;
    s[5] = sy;
    s[10] = sz;
    matrixMultiply(m, m, s);
}

void matrixTranslate(float* m, float tx, float ty, float tz) {
    float t[16];
    matrixIdentity(t);
    t[12] = tx;
    t[13] = ty;
    t[14] = tz;
    matrixMultiply(m, m, t);
}

void matrixRotate(float* m, float angleDeg, float x, float y, float z) {
    float rad = angleDeg * 3.14159265358979323846f / 180.0f;
    float c = std::cos(rad);
    float s = std::sin(rad);
    float nc = 1.0f - c;

    float len = std::sqrt(x * x + y * y + z * z);
    if (len > 1e-6f) {
        x /= len; y /= len; z /= len;
    }

    float rot[16];
    matrixIdentity(rot);
    rot[0] = x * x * nc + c;
    rot[1] = y * x * nc + z * s;
    rot[2] = z * x * nc - y * s;

    rot[4] = x * y * nc - z * s;
    rot[5] = y * y * nc + c;
    rot[6] = z * y * nc + x * s;

    rot[8] = x * z * nc + y * s;
    rot[9] = y * z * nc - x * s;
    rot[10] = z * z * nc + c;

    matrixMultiply(m, m, rot);
}

bool matrixInvert4x4(float* inv, const float* m) {
    float t[16];
    t[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] +
           m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
    t[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] -
           m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
    t[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] +
           m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
    t[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] -
            m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];

    t[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] -
           m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
    t[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] +
           m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
    t[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] -
           m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
    t[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] +
            m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];

    t[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] +
           m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
    t[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] -
           m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
    t[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] +
            m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
    t[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] -
            m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];

    t[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] -
           m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
    t[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] +
           m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
    t[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] -
            m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
    t[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] +
            m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];

    float det = m[0] * t[0] + m[1] * t[4] + m[2] * t[8] + m[3] * t[12];
    if (std::abs(det) < 1e-8f) return false;

    float invDet = 1.0f / det;
    for (int i = 0; i < 16; ++i) {
        inv[i] = t[i] * invDet;
    }
    return true;
}

void matrixNormal3x3(float* n3x3, const float* m4x4) {
    float inv4[16];
    if (matrixInvert4x4(inv4, m4x4)) {
        // Normal matrix is transpose of upper-left 3x3 inverse
        n3x3[0] = inv4[0]; n3x3[1] = inv4[4]; n3x3[2] = inv4[8];
        n3x3[3] = inv4[1]; n3x3[4] = inv4[5]; n3x3[5] = inv4[9];
        n3x3[6] = inv4[2]; n3x3[7] = inv4[6]; n3x3[8] = inv4[10];
    } else {
        n3x3[0] = 1.0f; n3x3[1] = 0.0f; n3x3[2] = 0.0f;
        n3x3[3] = 0.0f; n3x3[4] = 1.0f; n3x3[5] = 0.0f;
        n3x3[6] = 0.0f; n3x3[7] = 0.0f; n3x3[8] = 1.0f;
    }
}

GLuint compileShader(GLenum type, const char* source) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint compiled = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (!compiled) {
        GLint infoLen = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLen);
        if (infoLen > 1) {
            std::vector<char> info(infoLen);
            glGetShaderInfoLog(shader, infoLen, nullptr, info.data());
            LOGE("着色器编译失败 (%s): %s",
                 type == GL_VERTEX_SHADER ? "VERTEX" : "FRAGMENT", info.data());
        }
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

} // anonymous namespace

GlbRenderer::GlbRenderer() = default;

GlbRenderer::~GlbRenderer() {
    destroyGL();
}

bool GlbRenderer::compileShaders() {
    const char* vShaderSource =
        "attribute vec3 a_Position;\n"
        "attribute vec3 a_Normal;\n"
        "attribute vec2 a_TexCoord;\n"
        "\n"
        "uniform mat4 u_MVPMatrix;\n"
        "uniform mat4 u_ModelMatrix;\n"
        "uniform mat3 u_NormalMatrix;\n"
        "\n"
        "varying vec3 v_Normal;\n"
        "varying vec2 v_TexCoord;\n"
        "varying vec3 v_WorldPos;\n"
        "\n"
        "void main() {\n"
        "    vec4 worldPos = u_ModelMatrix * vec4(a_Position, 1.0);\n"
        "    v_WorldPos = worldPos.xyz;\n"
        "    v_Normal = normalize(u_NormalMatrix * a_Normal);\n"
        "    v_TexCoord = a_TexCoord;\n"
        "    gl_Position = u_MVPMatrix * vec4(a_Position, 1.0);\n"
        "}\n";

    const char* fShaderSource =
        "precision mediump float;\n"
        "\n"
        "varying vec3 v_Normal;\n"
        "varying vec2 v_TexCoord;\n"
        "varying vec3 v_WorldPos;\n"
        "\n"
        "uniform sampler2D u_Texture;\n"
        "uniform int u_HasTexture;\n"
        "uniform vec4 u_BaseColor;\n"
        "\n"
        "uniform vec3 u_LightDir1;\n"
        "uniform vec3 u_LightColor1;\n"
        "uniform vec3 u_LightDir2;\n"
        "uniform vec3 u_LightColor2;\n"
        "uniform vec3 u_AmbientColor;\n"
        "uniform vec3 u_CameraPos;\n"
        "\n"
        "void main() {\n"
        "    vec4 base = u_BaseColor;\n"
        "    if (u_HasTexture == 1) {\n"
        "        base *= texture2D(u_Texture, v_TexCoord);\n"
        "    }\n"
        "    if (base.a < 0.05) {\n"
        "        discard;\n"
        "    }\n"
        "\n"
        "    vec3 N = normalize(v_Normal);\n"
        "    vec3 L1 = normalize(-u_LightDir1);\n"
        "    vec3 L2 = normalize(-u_LightDir2);\n"
        "\n"
        "    float diff1 = max(dot(N, L1), 0.0);\n"
        "    float diff2 = max(dot(N, L2), 0.0);\n"
        "\n"
        "    vec3 V = normalize(u_CameraPos - v_WorldPos);\n"
        "    vec3 H1 = normalize(L1 + V);\n"
        "    float spec1 = pow(max(dot(N, H1), 0.0), 32.0) * 0.25;\n"
        "\n"
        "    vec3 lighting = u_AmbientColor + (diff1 + spec1) * u_LightColor1 + diff2 * u_LightColor2;\n"
        "    gl_FragColor = vec4(base.rgb * lighting, base.a);\n"
        "}\n";

    GLuint vs = compileShader(GL_VERTEX_SHADER, vShaderSource);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fShaderSource);
    if (!vs || !fs) {
        if (vs) glDeleteShader(vs);
        if (fs) glDeleteShader(fs);
        return false;
    }

    mProgram = glCreateProgram();
    glAttachShader(mProgram, vs);
    glAttachShader(mProgram, fs);
    glLinkProgram(mProgram);

    glDeleteShader(vs);
    glDeleteShader(fs);

    GLint linked = 0;
    glGetProgramiv(mProgram, GL_LINK_STATUS, &linked);
    if (!linked) {
        GLint infoLen = 0;
        glGetProgramiv(mProgram, GL_INFO_LOG_LENGTH, &infoLen);
        if (infoLen > 1) {
            std::vector<char> info(infoLen);
            glGetProgramInfoLog(mProgram, infoLen, nullptr, info.data());
            LOGE("着色器链接失败: %s", info.data());
        }
        glDeleteProgram(mProgram);
        mProgram = 0;
        return false;
    }

    mLocPosition = glGetAttribLocation(mProgram, "a_Position");
    mLocNormal = glGetAttribLocation(mProgram, "a_Normal");
    mLocTexCoord = glGetAttribLocation(mProgram, "a_TexCoord");

    mLocMVPMatrix = glGetUniformLocation(mProgram, "u_MVPMatrix");
    mLocModelMatrix = glGetUniformLocation(mProgram, "u_ModelMatrix");
    mLocNormalMatrix = glGetUniformLocation(mProgram, "u_NormalMatrix");

    mLocTexture = glGetUniformLocation(mProgram, "u_Texture");
    mLocHasTexture = glGetUniformLocation(mProgram, "u_HasTexture");
    mLocBaseColor = glGetUniformLocation(mProgram, "u_BaseColor");

    mLocLightDir1 = glGetUniformLocation(mProgram, "u_LightDir1");
    mLocLightColor1 = glGetUniformLocation(mProgram, "u_LightColor1");
    mLocLightDir2 = glGetUniformLocation(mProgram, "u_LightDir2");
    mLocLightColor2 = glGetUniformLocation(mProgram, "u_LightColor2");
    mLocAmbientColor = glGetUniformLocation(mProgram, "u_AmbientColor");
    mLocCameraPos = glGetUniformLocation(mProgram, "u_CameraPos");

    return true;
}

bool GlbRenderer::initGL() {
    if (mIsGLReady) return true;

    if (!compileShaders()) {
        LOGE("GlbRenderer: 编译着色器程序失败");
        return false;
    }

    mIsGLReady = true;
    LOGI("GlbRenderer: OpenGL ES 着色器初始化完成");
    return true;
}

void GlbRenderer::onResize(int width, int height) {
    mWidth = width;
    mHeight = height;
    glViewport(0, 0, width, height);
}

void GlbRenderer::setModel(std::shared_ptr<GlbModel> model) {
    mModel = model;
}

void GlbRenderer::render(const float* slamModelMatrix,
                         const float* viewMatrix,
                         const float* projMatrix,
                         float scale,
                         float userRotationX,
                         float userRotationY) {
    if (!mIsGLReady || !mModel || !mModel->isLoaded() || !mModel->isGPUUploaded()) {
        return;
    }

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glUseProgram(mProgram);

    // 设置双定向光与环境光照参数
    glUniform3f(mLocLightDir1, -0.5f, -1.0f, -0.5f);
    glUniform3f(mLocLightColor1, 0.85f, 0.85f, 0.85f);

    glUniform3f(mLocLightDir2, 0.5f, 1.0f, 0.5f);
    glUniform3f(mLocLightColor2, 0.35f, 0.35f, 0.35f);

    glUniform3f(mLocAmbientColor, 0.45f, 0.45f, 0.45f);

    // 从视图矩阵逆矩阵提取相机世界位置
    float invView[16];
    if (matrixInvert4x4(invView, viewMatrix)) {
        glUniform3f(mLocCameraPos, invView[12], invView[13], invView[14]);
    } else {
        glUniform3f(mLocCameraPos, 0.0f, 0.0f, 0.0f);
    }

    glUniform1i(mLocTexture, 0);

    const auto& bounds = mModel->getBounds();

    // 变换基础：S * R180 * Ry * Rx
    float userTransform[16];
    matrixIdentity(userTransform);
    matrixScale(userTransform, scale, scale, scale);
    matrixRotate(userTransform, 180.0f, 1.0f, 0.0f, 0.0f); // 翻转以对齐 SLAM 坐标系

    if (std::abs(userRotationY) > 0.01f) {
        matrixRotate(userTransform, userRotationY, 0.0f, 1.0f, 0.0f);
    }
    if (std::abs(userRotationX) > 0.01f) {
        matrixRotate(userTransform, userRotationX, 1.0f, 0.0f, 0.0f);
    }

    // 将用户变换叠加在 SLAM ModelMatrix 之后：M_slam * M_user
    float baseWorldMatrix[16];
    matrixMultiply(baseWorldMatrix, slamModelMatrix, userTransform);

    // 计算 VP 矩阵：P * V
    float vpMatrix[16];
    matrixMultiply(vpMatrix, projMatrix, viewMatrix);

    const auto& primitives = mModel->getPrimitives();
    for (const auto& prim : primitives) {
        if (prim.indexCount == 0 || prim.vbo == 0 || prim.ibo == 0) continue;

        // 图元的完整模型矩阵：baseWorldMatrix * prim.nodeMatrix（与 Filament 保持完全一致）
        float finalModelMatrix[16];
        matrixMultiply(finalModelMatrix, baseWorldMatrix, prim.nodeMatrix);

        // 最终 MVP 矩阵：(P * V) * finalModelMatrix
        float mvpMatrix[16];
        matrixMultiply(mvpMatrix, vpMatrix, finalModelMatrix);

        // 法线矩阵
        float normalMatrix[9];
        matrixNormal3x3(normalMatrix, finalModelMatrix);

        glUniformMatrix4fv(mLocMVPMatrix, 1, GL_FALSE, mvpMatrix);
        glUniformMatrix4fv(mLocModelMatrix, 1, GL_FALSE, finalModelMatrix);
        glUniformMatrix3fv(mLocNormalMatrix, 1, GL_FALSE, normalMatrix);

        // 材质与纹理绑定
        glUniform4fv(mLocBaseColor, 1, prim.material.baseColor);
        if (prim.material.hasTexture && prim.material.textureId > 0) {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(prim.material.textureId));
            glUniform1i(mLocHasTexture, 1);
        } else {
            glUniform1i(mLocHasTexture, 0);
        }

        // 双面渲染处理
        if (prim.material.doubleSided) {
            glDisable(GL_CULL_FACE);
        } else {
            glEnable(GL_CULL_FACE);
            glCullFace(GL_BACK);
        }

        // 绑定 VBO 并设置顶点属性
        glBindBuffer(GL_ARRAY_BUFFER, prim.vbo);

        glEnableVertexAttribArray(mLocPosition);
        glVertexAttribPointer(mLocPosition, 3, GL_FLOAT, GL_FALSE,
                              sizeof(GlbVertex), (void*)offsetof(GlbVertex, pos));

        glEnableVertexAttribArray(mLocNormal);
        glVertexAttribPointer(mLocNormal, 3, GL_FLOAT, GL_FALSE,
                              sizeof(GlbVertex), (void*)offsetof(GlbVertex, normal));

        glEnableVertexAttribArray(mLocTexCoord);
        glVertexAttribPointer(mLocTexCoord, 2, GL_FLOAT, GL_FALSE,
                              sizeof(GlbVertex), (void*)offsetof(GlbVertex, uv));

        // 绑定 IBO 并绘制
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, prim.ibo);
        glDrawElements(GL_TRIANGLES, prim.indexCount, prim.indexType, nullptr);

        glDisableVertexAttribArray(mLocPosition);
        glDisableVertexAttribArray(mLocNormal);
        glDisableVertexAttribArray(mLocTexCoord);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    glDisable(GL_CULL_FACE);
}

void GlbRenderer::destroyGL() {
    if (mProgram) {
        glDeleteProgram(mProgram);
        mProgram = 0;
    }
    mIsGLReady = false;
}
