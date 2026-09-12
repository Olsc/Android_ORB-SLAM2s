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

#include "glb_model.h"
#include <cmath>
#include <algorithm>
#include <cstring>

#define CGLTF_IMPLEMENTATION
#include "cgltf.h"

GlbModel::GlbModel() = default;

GlbModel::~GlbModel() {
    destroyGL();
}

bool GlbModel::loadFromMemory(const uint8_t* buffer, size_t size) {
    if (!buffer || size == 0) {
        LOGE("loadFromMemory: 空缓冲");
        return false;
    }

    destroyGL();
    mPrimitives.clear();
    mRawBuffer.assign(buffer, buffer + size);

    mBounds = GlbBounds();

    cgltf_options options{};
    memset(&options, 0, sizeof(options));
    cgltf_data* data = nullptr;
    cgltf_result result = cgltf_parse(&options, mRawBuffer.data(), mRawBuffer.size(), &data);
    if (result != cgltf_result_success) {
        LOGE("cgltf_parse 失败, 错误码: %d", (int)result);
        return false;
    }

    result = cgltf_load_buffers(&options, data, nullptr);
    if (result != cgltf_result_success) {
        LOGE("cgltf_load_buffers 失败, 错误码: %d", (int)result);
        cgltf_free(data);
        return false;
    }

    LOGI("cgltf 解析成功: 节点数=%zu, 网格数=%zu, 材质数=%zu, 图像数=%zu",
         data->nodes_count, data->meshes_count, data->materials_count, data->images_count);

    bool hasAnyVertex = false;

    // 遍历所有节点
    for (cgltf_size i = 0; i < data->nodes_count; ++i) {
        cgltf_node* node = &data->nodes[i];
        if (!node->mesh) continue;

        float nodeMatrix[16];
        cgltf_node_transform_world(node, nodeMatrix);

        cgltf_mesh* mesh = node->mesh;
        for (cgltf_size p = 0; p < mesh->primitives_count; ++p) {
            cgltf_primitive* prim = &mesh->primitives[p];
            if (prim->type != cgltf_primitive_type_triangles) {
                // 仅支持三角形图元
                continue;
            }

            cgltf_accessor* pos_acc = nullptr;
            cgltf_accessor* norm_acc = nullptr;
            cgltf_accessor* uv_acc = nullptr;

            for (cgltf_size a = 0; a < prim->attributes_count; ++a) {
                if (prim->attributes[a].type == cgltf_attribute_type_position) {
                    pos_acc = prim->attributes[a].data;
                } else if (prim->attributes[a].type == cgltf_attribute_type_normal) {
                    norm_acc = prim->attributes[a].data;
                } else if (prim->attributes[a].type == cgltf_attribute_type_texcoord && prim->attributes[a].index == 0) {
                    uv_acc = prim->attributes[a].data;
                }
            }

            if (!pos_acc) continue;

            size_t vertex_count = pos_acc->count;
            std::vector<float> positions(vertex_count * 3);
            cgltf_accessor_unpack_floats(pos_acc, positions.data(), positions.size());

            std::vector<float> normals;
            if (norm_acc) {
                normals.resize(vertex_count * 3);
                cgltf_accessor_unpack_floats(norm_acc, normals.data(), normals.size());
            }

            std::vector<float> uvs;
            if (uv_acc) {
                uvs.resize(vertex_count * 2);
                cgltf_accessor_unpack_floats(uv_acc, uvs.data(), uvs.size());
            }

            GlbPrimitive glbPrim;
            glbPrim.vertices.resize(vertex_count);
            memcpy(glbPrim.nodeMatrix, nodeMatrix, sizeof(nodeMatrix));

            for (size_t v = 0; v < vertex_count; ++v) {
                glbPrim.vertices[v].pos[0] = positions[v * 3 + 0];
                glbPrim.vertices[v].pos[1] = positions[v * 3 + 1];
                glbPrim.vertices[v].pos[2] = positions[v * 3 + 2];

                if (!normals.empty()) {
                    glbPrim.vertices[v].normal[0] = normals[v * 3 + 0];
                    glbPrim.vertices[v].normal[1] = normals[v * 3 + 1];
                    glbPrim.vertices[v].normal[2] = normals[v * 3 + 2];
                } else {
                    glbPrim.vertices[v].normal[0] = 0.0f;
                    glbPrim.vertices[v].normal[1] = 1.0f;
                    glbPrim.vertices[v].normal[2] = 0.0f;
                }

                if (!uvs.empty()) {
                    glbPrim.vertices[v].uv[0] = uvs[v * 2 + 0];
                    glbPrim.vertices[v].uv[1] = uvs[v * 2 + 1];
                } else {
                    glbPrim.vertices[v].uv[0] = 0.0f;
                    glbPrim.vertices[v].uv[1] = 0.0f;
                }

                // 变换到根世界坐标下更新 AABB 包围盒
                float wx = nodeMatrix[0] * glbPrim.vertices[v].pos[0] +
                           nodeMatrix[4] * glbPrim.vertices[v].pos[1] +
                           nodeMatrix[8] * glbPrim.vertices[v].pos[2] + nodeMatrix[12];
                float wy = nodeMatrix[1] * glbPrim.vertices[v].pos[0] +
                           nodeMatrix[5] * glbPrim.vertices[v].pos[1] +
                           nodeMatrix[9] * glbPrim.vertices[v].pos[2] + nodeMatrix[13];
                float wz = nodeMatrix[2] * glbPrim.vertices[v].pos[0] +
                           nodeMatrix[6] * glbPrim.vertices[v].pos[1] +
                           nodeMatrix[10] * glbPrim.vertices[v].pos[2] + nodeMatrix[14];

                mBounds.min[0] = std::min(mBounds.min[0], wx);
                mBounds.min[1] = std::min(mBounds.min[1], wy);
                mBounds.min[2] = std::min(mBounds.min[2], wz);
                mBounds.max[0] = std::max(mBounds.max[0], wx);
                mBounds.max[1] = std::max(mBounds.max[1], wy);
                mBounds.max[2] = std::max(mBounds.max[2], wz);
                hasAnyVertex = true;
            }

            // 索引解包
            if (prim->indices) {
                glbPrim.indices.resize(prim->indices->count);
                cgltf_accessor_unpack_indices(prim->indices, glbPrim.indices.data(), sizeof(uint32_t), glbPrim.indices.size());
            } else {
                glbPrim.indices.resize(vertex_count);
                for (size_t v = 0; v < vertex_count; ++v) {
                    glbPrim.indices[v] = (uint32_t)v;
                }
            }

            // 若原本无提供法线，则动态计算面法线
            if (normals.empty()) {
                computeNormalsIfMissing(glbPrim);
            }

            // 解析材质与纹理
            if (prim->material) {
                cgltf_material* mat = prim->material;
                glbPrim.material.doubleSided = mat->double_sided;
                if (mat->has_pbr_metallic_roughness) {
                    glbPrim.material.baseColor[0] = mat->pbr_metallic_roughness.base_color_factor[0];
                    glbPrim.material.baseColor[1] = mat->pbr_metallic_roughness.base_color_factor[1];
                    glbPrim.material.baseColor[2] = mat->pbr_metallic_roughness.base_color_factor[2];
                    glbPrim.material.baseColor[3] = mat->pbr_metallic_roughness.base_color_factor[3];

                    if (mat->pbr_metallic_roughness.base_color_texture.texture) {
                        cgltf_texture* tex = mat->pbr_metallic_roughness.base_color_texture.texture;
                        if (tex->image) {
                            glbPrim.material.imageIndex = (int)(tex->image - data->images);
                            if (tex->image->buffer_view) {
                                const uint8_t* bData = (const uint8_t*)tex->image->buffer_view->buffer->data +
                                                       tex->image->buffer_view->offset;
                                glbPrim.material.imageBytes = bData;
                                glbPrim.material.imageSize = tex->image->buffer_view->size;
                                glbPrim.material.hasTexture = true;
                            }
                        }
                    }
                }
            }

            mPrimitives.push_back(std::move(glbPrim));
        }
    }

    if (hasAnyVertex) {
        mBounds.center[0] = (mBounds.min[0] + mBounds.max[0]) * 0.5f;
        mBounds.center[1] = (mBounds.min[1] + mBounds.max[1]) * 0.5f;
        mBounds.center[2] = (mBounds.min[2] + mBounds.max[2]) * 0.5f;

        mBounds.halfExtent[0] = (mBounds.max[0] - mBounds.min[0]) * 0.5f;
        mBounds.halfExtent[1] = (mBounds.max[1] - mBounds.min[1]) * 0.5f;
        mBounds.halfExtent[2] = (mBounds.max[2] - mBounds.min[2]) * 0.5f;

        mBounds.maxDim = std::max(mBounds.halfExtent[0],
                                  std::max(mBounds.halfExtent[1], mBounds.halfExtent[2])) * 2.0f;
        mBounds.autoScaleFactor = (mBounds.maxDim > 0.0f) ? (0.5f / mBounds.maxDim) : 1.0f;

        LOGI("模型包围盒: 中心=[%.3f, %.3f, %.3f], 半长=[%.3f, %.3f, %.3f], 最大维度=%.3f, 自动缩放=%.4f",
             mBounds.center[0], mBounds.center[1], mBounds.center[2],
             mBounds.halfExtent[0], mBounds.halfExtent[1], mBounds.halfExtent[2],
             mBounds.maxDim, mBounds.autoScaleFactor);
    }

    cgltf_free(data);
    mIsLoaded = !mPrimitives.empty();
    return mIsLoaded;
}

void GlbModel::computeNormalsIfMissing(GlbPrimitive& prim) {
    for (auto& v : prim.vertices) {
        v.normal[0] = 0.0f;
        v.normal[1] = 0.0f;
        v.normal[2] = 0.0f;
    }

    size_t numTriangles = prim.indices.size() / 3;
    for (size_t t = 0; t < numTriangles; ++t) {
        uint32_t i0 = prim.indices[t * 3 + 0];
        uint32_t i1 = prim.indices[t * 3 + 1];
        uint32_t i2 = prim.indices[t * 3 + 2];

        if (i0 >= prim.vertices.size() || i1 >= prim.vertices.size() || i2 >= prim.vertices.size()) {
            continue;
        }

        const float* p0 = prim.vertices[i0].pos;
        const float* p1 = prim.vertices[i1].pos;
        const float* p2 = prim.vertices[i2].pos;

        float e1x = p1[0] - p0[0], e1y = p1[1] - p0[1], e1z = p1[2] - p0[2];
        float e2x = p2[0] - p0[0], e2y = p2[1] - p0[1], e2z = p2[2] - p0[2];

        float nx = e1y * e2z - e1z * e2y;
        float ny = e1z * e2x - e1x * e2z;
        float nz = e1x * e2y - e1y * e2x;

        prim.vertices[i0].normal[0] += nx; prim.vertices[i0].normal[1] += ny; prim.vertices[i0].normal[2] += nz;
        prim.vertices[i1].normal[0] += nx; prim.vertices[i1].normal[1] += ny; prim.vertices[i1].normal[2] += nz;
        prim.vertices[i2].normal[0] += nx; prim.vertices[i2].normal[1] += ny; prim.vertices[i2].normal[2] += nz;
    }

    for (auto& v : prim.vertices) {
        float len = std::sqrt(v.normal[0] * v.normal[0] + v.normal[1] * v.normal[1] + v.normal[2] * v.normal[2]);
        if (len > 1e-6f) {
            v.normal[0] /= len;
            v.normal[1] /= len;
            v.normal[2] /= len;
        } else {
            v.normal[1] = 1.0f;
        }
    }
}

void GlbModel::uploadGL(std::function<int(const uint8_t*, size_t)> textureLoader) {
    if (mIsGPUUploaded) return;

    for (auto& prim : mPrimitives) {
        // 创建并填充 VBO
        glGenBuffers(1, &prim.vbo);
        glBindBuffer(GL_ARRAY_BUFFER, prim.vbo);
        glBufferData(GL_ARRAY_BUFFER, prim.vertices.size() * sizeof(GlbVertex),
                     prim.vertices.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        // 创建并填充 IBO
        glGenBuffers(1, &prim.ibo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, prim.ibo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, prim.indices.size() * sizeof(uint32_t),
                     prim.indices.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        prim.indexCount = static_cast<GLsizei>(prim.indices.size());
        prim.indexType = GL_UNSIGNED_INT;

        // 加载纹理
        if (prim.material.hasTexture && prim.material.imageBytes && prim.material.imageSize > 0) {
            int imgIdx = prim.material.imageIndex;
            auto it = mImageToTextureMap.find(imgIdx);
            if (it != mImageToTextureMap.end()) {
                prim.material.textureId = it->second;
            } else if (textureLoader) {
                int texId = textureLoader(prim.material.imageBytes, prim.material.imageSize);
                if (texId > 0) {
                    prim.material.textureId = texId;
                    mImageToTextureMap[imgIdx] = texId;
                    LOGI("成功为图元上传纹理, textureId=%d, imageSize=%zu", texId, prim.material.imageSize);
                } else {
                    prim.material.hasTexture = false;
                    LOGW("纹理上传失败, 回退至纯色材质");
                }
            }
        }
    }

    mIsGPUUploaded = true;
}

void GlbModel::destroyGL() {
    if (!mIsGPUUploaded) return;

    for (auto& prim : mPrimitives) {
        if (prim.vbo) {
            glDeleteBuffers(1, &prim.vbo);
            prim.vbo = 0;
        }
        if (prim.ibo) {
            glDeleteBuffers(1, &prim.ibo);
            prim.ibo = 0;
        }
    }

    for (auto& pair : mImageToTextureMap) {
        GLuint tex = static_cast<GLuint>(pair.second);
        if (tex) {
            glDeleteTextures(1, &tex);
        }
    }
    mImageToTextureMap.clear();

    mIsGPUUploaded = false;
}
