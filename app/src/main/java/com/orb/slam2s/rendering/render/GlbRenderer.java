package com.orb.slam2s.rendering.render;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.opengl.GLES20;
import android.opengl.GLUtils;
import android.opengl.Matrix;
import android.util.Log;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;
import java.util.ArrayList;
import java.util.List;

/**
 * GLB (glTF Binary) 3D模型渲染器。
 * <p>
 * 直接在二进制级别解析GLB格式，无需额外依赖。
 * 支持：顶点位置/法线/纹理坐标、索引缓冲、嵌入式纹理、PBR材质→Blinn-Phong转换。
 * <p>
 * 格式参考：https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html
 */
public class GlbRenderer {
    private static final String TAG = "GlbRenderer";

    // ======== GLB 二进制格式常量 ========
    private static final int GLB_MAGIC = 0x46546C67;          // "glTF"
    private static final int GLB_VERSION = 2;
    private static final int CHUNK_TYPE_JSON = 0x4E4F534A;    // "JSON"
    private static final int CHUNK_TYPE_BIN  = 0x004E4942;    // "BIN\0"

    // glTF 组件类型
    private static final int COMP_FLOAT          = 5126;
    private static final int COMP_UNSIGNED_SHORT = 5123;
    private static final int COMP_UNSIGNED_INT   = 5125;
    private static final int COMP_UNSIGNED_BYTE  = 5121;

    // ======== 着色器 (与 ObjectRenderer 一致，Blinn-Phong 光照) ========
    private static final String VERTEX_SHADER =
            "uniform mat4 u_ModelViewProjection;\n" +
            "uniform mat4 u_ModelView;\n" +
            "uniform mat3 u_NormalMatrix;\n" +
            "uniform vec3 u_LightPosition;\n" +
            "\n" +
            "attribute vec4 a_Position;\n" +
            "attribute vec2 a_TexCoord;\n" +
            "attribute vec3 a_Normal;\n" +
            "\n" +
            "varying vec2 v_TexCoord;\n" +
            "varying vec3 v_Normal;\n" +
            "varying vec3 v_ViewPosition;\n" +
            "varying vec3 v_LightDir;\n" +
            "varying float v_Alpha;\n" +
            "\n" +
            "void main() {\n" +
            "   v_TexCoord = a_TexCoord;\n" +
            "   v_Normal = normalize(u_NormalMatrix * a_Normal);\n" +
            "   vec4 viewPosition = u_ModelView * a_Position;\n" +
            "   v_ViewPosition = viewPosition.xyz;\n" +
            "   v_LightDir = normalize(u_LightPosition - v_ViewPosition);\n" +
            "   v_Alpha = 1.0;\n" +
            "   gl_Position = u_ModelViewProjection * a_Position;\n" +
            "}";

    private static final String FRAGMENT_SHADER =
            "precision mediump float;\n" +
            "\n" +
            "uniform sampler2D u_Texture;\n" +
            "uniform vec3 u_MaterialAmbient;\n" +
            "uniform vec3 u_MaterialDiffuse;\n" +
            "uniform vec3 u_MaterialSpecular;\n" +
            "uniform float u_MaterialShininess;\n" +
            "uniform vec3 u_LightAmbient;\n" +
            "uniform vec3 u_LightDiffuse;\n" +
            "uniform vec3 u_LightSpecular;\n" +
            "\n" +
            "varying vec2 v_TexCoord;\n" +
            "varying vec3 v_Normal;\n" +
            "varying vec3 v_ViewPosition;\n" +
            "varying vec3 v_LightDir;\n" +
            "varying float v_Alpha;\n" +
            "\n" +
            "void main() {\n" +
            "    vec4 textureColor = texture2D(u_Texture, v_TexCoord);\n" +
            "    vec3 normal = normalize(v_Normal);\n" +
            "    vec3 lightDir = normalize(v_LightDir);\n" +
            "    vec3 viewDir = normalize(-v_ViewPosition);\n" +
            "\n" +
            "    // Ambient\n" +
            "    vec3 ambient = u_LightAmbient * u_MaterialAmbient;\n" +
            "\n" +
            "    // Diffuse (Lambertian)\n" +
            "    float diff = max(dot(normal, lightDir), 0.0);\n" +
            "    vec3 diffuse = u_LightDiffuse * u_MaterialDiffuse * diff;\n" +
            "\n" +
            "    // Specular (Blinn-Phong)\n" +
            "    vec3 halfwayDir = normalize(lightDir + viewDir);\n" +
            "    float spec = pow(max(dot(normal, halfwayDir), 0.0), u_MaterialShininess);\n" +
            "    vec3 specular = u_LightSpecular * u_MaterialSpecular * spec;\n" +
            "\n" +
            "    vec3 lighting = ambient + diffuse + specular;\n" +
            "    vec3 finalColor = lighting * textureColor.rgb;\n" +
            "    gl_FragColor = vec4(finalColor, textureColor.a * v_Alpha);\n" +
            "}";

    // ======== OpenGL 状态 ========
    private int program;
    private final int[] textures = new int[1];

    private int positionAttribute;
    private int texCoordAttribute;
    private int normalAttribute;

    private int modelViewProjectionUniform;
    private int modelViewUniform;
    private int normalMatrixUniform;
    private int textureUniform;
    private int lightPositionUniform;
    private int lightAmbientUniform;
    private int lightDiffuseUniform;
    private int lightSpecularUniform;
    private int materialAmbientUniform;
    private int materialDiffuseUniform;
    private int materialSpecularUniform;
    private int materialShininessUniform;

    // ======== 顶点数据缓冲 ========
    private FloatBuffer vertexBuffer;
    private FloatBuffer texCoordBuffer;
    private FloatBuffer normalBuffer;
    private ShortBuffer indexBuffer;
    private int vertexCount;

    // ======== 模型变换 ========
    private final float[] modelMatrix = new float[16];
    private float autoScaleFactor = 1.0f;
    private final float[] modelBounds = new float[6];
    private static final float TARGET_SIZE = 0.5f;

    // ======== 材质 (Blinn-Phong 参数, 从 PBR 转换) ========
    private float[] materialAmbient  = {0.2f, 0.2f, 0.2f};
    private float[] materialDiffuse  = {0.8f, 0.8f, 0.8f};
    private float[] materialSpecular = {0.5f, 0.5f, 0.5f};
    private float   materialShininess = 32.0f;

    // ======== 光照 ========
    private final float[] lightPosition  = {2.0f, 4.0f, 3.0f};
    private final float[] lightAmbient   = {0.3f, 0.3f, 0.3f};
    private final float[] lightDiffuse   = {0.8f, 0.8f, 0.8f};
    private final float[] lightSpecular  = {1.0f, 1.0f, 1.0f};

    // ======== 纹理加载状态 ========
    private Bitmap textureBitmap = null;

    // ================================================================
    //  公共 API
    // ================================================================

    public GlbRenderer() {
        Matrix.setIdentityM(modelMatrix, 0);
    }

    /**
     * 在 GL 线程上初始化：编译着色器、解析 GLB、上传纹理。
     *
     * @param context      Android 上下文
     * @param glbAssetName assets 下的 GLB 文件路径（如 "model.glb"）
     */
    public void createOnGlThread(Context context, String glbAssetName) throws IOException {
        // 1. 编译着色器
        int vertexShader   = loadGLShader(GLES20.GL_VERTEX_SHADER, VERTEX_SHADER);
        int fragmentShader = loadGLShader(GLES20.GL_FRAGMENT_SHADER, FRAGMENT_SHADER);

        program = GLES20.glCreateProgram();
        GLES20.glAttachShader(program, vertexShader);
        GLES20.glAttachShader(program, fragmentShader);
        GLES20.glLinkProgram(program);
        GLES20.glUseProgram(program);

        // 2. 获取属性/ uniform 位置
        positionAttribute         = GLES20.glGetAttribLocation(program, "a_Position");
        texCoordAttribute         = GLES20.glGetAttribLocation(program, "a_TexCoord");
        normalAttribute           = GLES20.glGetAttribLocation(program, "a_Normal");

        modelViewProjectionUniform = GLES20.glGetUniformLocation(program, "u_ModelViewProjection");
        modelViewUniform           = GLES20.glGetUniformLocation(program, "u_ModelView");
        normalMatrixUniform        = GLES20.glGetUniformLocation(program, "u_NormalMatrix");
        textureUniform             = GLES20.glGetUniformLocation(program, "u_Texture");
        lightPositionUniform       = GLES20.glGetUniformLocation(program, "u_LightPosition");
        lightAmbientUniform        = GLES20.glGetUniformLocation(program, "u_LightAmbient");
        lightDiffuseUniform        = GLES20.glGetUniformLocation(program, "u_LightDiffuse");
        lightSpecularUniform       = GLES20.glGetUniformLocation(program, "u_LightSpecular");
        materialAmbientUniform     = GLES20.glGetUniformLocation(program, "u_MaterialAmbient");
        materialDiffuseUniform     = GLES20.glGetUniformLocation(program, "u_MaterialDiffuse");
        materialSpecularUniform    = GLES20.glGetUniformLocation(program, "u_MaterialSpecular");
        materialShininessUniform   = GLES20.glGetUniformLocation(program, "u_MaterialShininess");

        // 3. 解析 GLB 文件并填充缓冲
        parseGlb(context, glbAssetName);

        // 4. 加载纹理（仅使用 GLB 内嵌材质，无外部纹理回退）
        if (textureBitmap != null) {
            uploadTexture(textureBitmap);
            Log.i(TAG, "使用 GLB 内嵌纹理");
        } else {
            uploadTexture(createCheckerTexture());
            Log.i(TAG, "GLB 无内嵌纹理，使用默认棋盘格纹理");
        }
    }

    /**
     * 更新模型矩阵（由父级调用，叠加用户缩放和自动缩放）。
     */
    public void updateModelMatrix(float[] parentMatrix, float scaleFactor) {
        float[] scaleMat = new float[16];
        Matrix.setIdentityM(scaleMat, 0);
        float finalScale = scaleFactor * autoScaleFactor;
        Matrix.scaleM(scaleMat, 0, finalScale, finalScale, finalScale);
        Matrix.multiplyMM(this.modelMatrix, 0, parentMatrix, 0, scaleMat, 0);
    }

    /**
     * 每帧绘制。
     *
     * @param cameraView        视图矩阵 4x4
     * @param cameraPerspective 投影矩阵 4x4
     * @param lightIntensity    光照强度系数
     */
    public void draw(float[] cameraView, float[] cameraPerspective, float lightIntensity) {
        GLES20.glUseProgram(program);

        // ---- 矩阵计算 ----
        float[] modelView          = new float[16];
        float[] modelViewProjection = new float[16];
        float[] normalMatrix        = new float[9];

        Matrix.multiplyMM(modelView, 0, cameraView, 0, modelMatrix, 0);
        Matrix.multiplyMM(modelViewProjection, 0, cameraPerspective, 0, modelView, 0);

        // 法线矩阵 = modelView 左上 3×3（忽略平移）
        normalMatrix[0] = modelView[0]; normalMatrix[1] = modelView[1]; normalMatrix[2] = modelView[2];
        normalMatrix[3] = modelView[4]; normalMatrix[4] = modelView[5]; normalMatrix[5] = modelView[6];
        normalMatrix[6] = modelView[8]; normalMatrix[7] = modelView[9]; normalMatrix[8] = modelView[10];

        GLES20.glUniformMatrix4fv(modelViewProjectionUniform, 1, false, modelViewProjection, 0);
        GLES20.glUniformMatrix4fv(modelViewUniform, 1, false, modelView, 0);
        GLES20.glUniformMatrix3fv(normalMatrixUniform, 1, false, normalMatrix, 0);

        // ---- 光照 ----
        float[] lp = {
                lightPosition[0] * lightIntensity,
                lightPosition[1] * lightIntensity,
                lightPosition[2] * lightIntensity
        };
        GLES20.glUniform3fv(lightPositionUniform, 1, lp, 0);

        float[] la = {
                lightAmbient[0] * lightIntensity,
                lightAmbient[1] * lightIntensity,
                lightAmbient[2] * lightIntensity
        };
        float[] ld = {
                lightDiffuse[0] * lightIntensity,
                lightDiffuse[1] * lightIntensity,
                lightDiffuse[2] * lightIntensity
        };
        GLES20.glUniform3fv(lightAmbientUniform,  1, la, 0);
        GLES20.glUniform3fv(lightDiffuseUniform,  1, ld, 0);
        GLES20.glUniform3fv(lightSpecularUniform, 1, lightSpecular, 0);

        // ---- 材质 ----
        GLES20.glUniform3fv(materialAmbientUniform,  1, materialAmbient, 0);
        GLES20.glUniform3fv(materialDiffuseUniform,  1, materialDiffuse, 0);
        GLES20.glUniform3fv(materialSpecularUniform, 1, materialSpecular, 0);
        GLES20.glUniform1f(materialShininessUniform, materialShininess);

        // ---- 纹理 ----
        GLES20.glActiveTexture(GLES20.GL_TEXTURE0);
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, textures[0]);
        GLES20.glUniform1i(textureUniform, 0);

        // ---- 顶点属性 ----
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0);

        GLES20.glVertexAttribPointer(positionAttribute, 3, GLES20.GL_FLOAT, false, 0, vertexBuffer);
        GLES20.glEnableVertexAttribArray(positionAttribute);

        GLES20.glVertexAttribPointer(texCoordAttribute, 2, GLES20.GL_FLOAT, false, 0, texCoordBuffer);
        GLES20.glEnableVertexAttribArray(texCoordAttribute);

        if (normalBuffer != null) {
            GLES20.glVertexAttribPointer(normalAttribute, 3, GLES20.GL_FLOAT, false, 0, normalBuffer);
            GLES20.glEnableVertexAttribArray(normalAttribute);
        } else {
            GLES20.glDisableVertexAttribArray(normalAttribute);
            GLES20.glVertexAttrib3f(normalAttribute, 0.0f, 1.0f, 0.0f);
        }

        // ---- 渲染状态 ----
        GLES20.glEnable(GLES20.GL_DEPTH_TEST);
        GLES20.glEnable(GLES20.GL_BLEND);
        GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA);

        // ---- 绘制 (readIndices 统一转换为 unsigned short) ----
        GLES20.glDrawElements(GLES20.GL_TRIANGLES, vertexCount, GLES20.GL_UNSIGNED_SHORT, indexBuffer);

        // ---- 清理 ----
        GLES20.glDisableVertexAttribArray(positionAttribute);
        GLES20.glDisableVertexAttribArray(texCoordAttribute);
        if (normalBuffer != null) {
            GLES20.glDisableVertexAttribArray(normalAttribute);
        }
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, 0);
    }

    public float getAutoScaleFactor() {
        return autoScaleFactor;
    }

    // ================================================================
    //  GLB 二进制解析
    // ================================================================

    /**
     * 解析 GLB 文件，提取网格数据并创建 OpenGL 缓冲。
     */
    private void parseGlb(Context context, String assetName) throws IOException {
        // ---- 读取完整文件 ----
        byte[] fileBytes;
        try (InputStream is = context.getAssets().open(assetName)) {
            fileBytes = readAllBytes(is);
        }
        ByteBuffer fileBuf = ByteBuffer.wrap(fileBytes).order(ByteOrder.LITTLE_ENDIAN);

        // ---- 解析 GLB 头部 ----
        int magic    = fileBuf.getInt();
        int version  = fileBuf.getInt();
        int fileLen  = fileBuf.getInt();

        if (magic != GLB_MAGIC) {
            throw new IOException("非法的 GLB 文件: magic=0x" + Integer.toHexString(magic));
        }
        if (version != GLB_VERSION) {
            Log.w(TAG, "GLB 版本 " + version + "，与 v2 可能不兼容");
        }

        // ---- 提取各个 chunk ----
        ByteBuffer jsonBuf = null;
        ByteBuffer binBuf  = null;

        while (fileBuf.position() < fileLen) {
            int chunkLen  = fileBuf.getInt();
            int chunkType = fileBuf.getInt();
            int oldLimit  = fileBuf.limit();

            fileBuf.limit(fileBuf.position() + chunkLen);
            ByteBuffer chunk = fileBuf.slice().order(ByteOrder.LITTLE_ENDIAN);
            fileBuf.limit(oldLimit);
            fileBuf.position(fileBuf.position() + chunkLen);

            if (chunkType == CHUNK_TYPE_JSON) {
                jsonBuf = chunk;
            } else if (chunkType == CHUNK_TYPE_BIN) {
                binBuf = chunk;
            }
        }

        if (jsonBuf == null) {
            throw new IOException("GLB 文件中未找到 JSON chunk");
        }

        // ---- 解析 JSON (org.json.JSONObject 方法抛 JSONException) ----
        byte[] jsonBytes = new byte[jsonBuf.remaining()];
        jsonBuf.get(jsonBytes);
        String jsonStr = new String(jsonBytes, "UTF-8");

        try {
            parseGltfJson(jsonStr, binBuf);
        } catch (Exception e) {
            throw new IOException("解析 GLB JSON 失败", e);
        }
    }

    /**
     * 解析 glTF JSON 并提取网格数据。
     * 提取到单独方法中以隔离 JSONException / Exception 处理。
     */
    private void parseGltfJson(String jsonStr, ByteBuffer binBuf) throws Exception {
        JSONObject gltf = new JSONObject(jsonStr);

        if (binBuf == null) {
            // 可能 buffer 数据是外部文件引用
            Log.w(TAG, "GLB 中没有 BIN chunk，尝试加载外部缓冲数据");
        }

        // ---- 遍历场景节点，收集 mesh 索引 ----
        List<Integer> meshIndices = new ArrayList<>();

        if (gltf.has("scenes") && gltf.has("nodes")) {
            JSONArray scenes = gltf.getJSONArray("scenes");
            int sceneIdx = gltf.optInt("scene", 0);
            if (sceneIdx >= 0 && sceneIdx < scenes.length()) {
                JSONObject scene = scenes.getJSONObject(sceneIdx);
                JSONArray nodes = scene.optJSONArray("nodes");
                JSONArray allNodes = gltf.getJSONArray("nodes");
                if (nodes != null) {
                    for (int i = 0; i < nodes.length(); i++) {
                        collectMeshes(allNodes, nodes.getInt(i), meshIndices);
                    }
                }
            }
        }

        // 备用：场景遍历为空时直接取第一个 mesh
        if (meshIndices.isEmpty() && gltf.has("meshes")) {
            JSONArray meshes = gltf.getJSONArray("meshes");
            for (int i = 0; i < meshes.length(); i++) {
                meshIndices.add(i);
            }
        }

        if (meshIndices.isEmpty()) {
            throw new IOException("GLB 文件中未找到任何 mesh");
        }

        JSONArray accessors   = gltf.optJSONArray("accessors");
        JSONArray bufferViews = gltf.optJSONArray("bufferViews");
        JSONArray meshesArr   = gltf.getJSONArray("meshes");

        if (accessors == null || bufferViews == null) {
            throw new IOException("GLB JSON 缺少 accessors 或 bufferViews");
        }

        // ---- 收集所有 primitive 数据 ----
        List<Float>   allVertices  = new ArrayList<>();
        List<Float>   allTexCoords = new ArrayList<>();
        List<Float>   allNormals   = new ArrayList<>();
        List<Short>   allIndices   = new ArrayList<>();

        boolean hasNormalData    = false;
        boolean hasTexCoordData  = false;

        for (int mi : meshIndices) {
            JSONObject meshObj = meshesArr.getJSONObject(mi);
            JSONArray primitives = meshObj.getJSONArray("primitives");

            for (int p = 0; p < primitives.length(); p++) {
                JSONObject prim = primitives.getJSONObject(p);
                JSONObject attrs = prim.getJSONObject("attributes");

                int vertexBase = allVertices.size() / 3;

                // --- POSITION (必需) ---
                int posAccIdx = attrs.getInt("POSITION");
                float[] pos = readAccessorFloats(binBuf, accessors, bufferViews, posAccIdx);
                for (float v : pos) allVertices.add(v);

                // --- NORMAL (可选) ---
                if (attrs.has("NORMAL")) {
                    int normAccIdx = attrs.getInt("NORMAL");
                    float[] norm = readAccessorFloats(binBuf, accessors, bufferViews, normAccIdx);
                    for (float v : norm) allNormals.add(v);
                    hasNormalData = true;
                }

                // --- TEXCOORD_0 (可选) ---
                if (attrs.has("TEXCOORD_0")) {
                    int tcAccIdx = attrs.getInt("TEXCOORD_0");
                    float[] tc   = readAccessorFloats(binBuf, accessors, bufferViews, tcAccIdx);
                    for (float v : tc) allTexCoords.add(v);
                    hasTexCoordData = true;
                }

                // --- INDICES (可选; 无索引则为非索引几何体) ---
                if (prim.has("indices")) {
                    int idxAccIdx = prim.getInt("indices");
                    readIndices(binBuf, accessors, bufferViews, idxAccIdx, allIndices, vertexBase);
                } else {
                    // 非索引几何体: 生成顺序索引
                    int vertexCount = pos.length / 3;
                    for (int i = 0; i < vertexCount; i++) {
                        allIndices.add((short) (vertexBase + i));
                    }
                }

                // --- 材质 + 内嵌纹理 ---
                if (prim.has("material") && gltf.has("materials")) {
                    applyMaterial(gltf, prim.getInt("material"), binBuf, bufferViews, accessors);
                }
            }
        }

        // ---- 合成缺失的顶点属性 ----
        int vertCount = allVertices.size() / 3;

        // 如果没有任何法线，计算平面法线
        if (!hasNormalData || allNormals.isEmpty()) {
            Log.w(TAG, "GLB 无法线数据，计算平面法线");
            computeFlatNormals(allVertices, allIndices, allNormals);
            hasNormalData = true;
        }

        // 如果没有纹理坐标，生成程序化 UV
        if (!hasTexCoordData || allTexCoords.isEmpty()) {
            Log.w(TAG, "GLB 无纹理坐标，生成程序化 UV");
            for (int i = 0; i < vertCount; i++) {
                float x = allVertices.get(i * 3);
                float y = allVertices.get(i * 3 + 1);
                float z = allVertices.get(i * 3 + 2);
                float len = (float) Math.sqrt(x * x + y * y + z * z);
                if (len > 0) {
                    float nx = x / len, ny = y / len, nz = z / len;
                    allTexCoords.add((float) (0.5 + Math.atan2(nz, nx) / (2 * Math.PI)));
                    allTexCoords.add((float) (0.5 - Math.asin(ny) / Math.PI));
                } else {
                    allTexCoords.add(0.5f);
                    allTexCoords.add(0.5f);
                }
            }
        }

        vertexCount = allIndices.size();

        // ---- 计算边界和自动缩放 ----
        if (vertCount > 0) {
            float minX = allVertices.get(0), maxX = allVertices.get(0);
            float minY = allVertices.get(1), maxY = allVertices.get(1);
            float minZ = allVertices.get(2), maxZ = allVertices.get(2);

            for (int i = 0; i < allVertices.size(); i += 3) {
                float x = allVertices.get(i);
                float y = allVertices.get(i + 1);
                float z = allVertices.get(i + 2);
                minX = Math.min(minX, x); maxX = Math.max(maxX, x);
                minY = Math.min(minY, y); maxY = Math.max(maxY, y);
                minZ = Math.min(minZ, z); maxZ = Math.max(maxZ, z);
            }

            modelBounds[0] = minX; modelBounds[1] = maxX;
            modelBounds[2] = minY; modelBounds[3] = maxY;
            modelBounds[4] = minZ; modelBounds[5] = maxZ;
            calculateAutoScale();

            Log.i(TAG, String.format("GLB 解析完成 — 顶点:%d 面:%d 边界:X[%.2f,%.2f] Y[%.2f,%.2f] Z[%.2f,%.2f] 缩放:%.3f",
                    vertCount, vertexCount / 3,
                    minX, maxX, minY, maxY, minZ, maxZ, autoScaleFactor));
        }

        // ---- 创建 OpenGL 缓冲 ----
        createBuffers(allVertices, allTexCoords, allNormals, allIndices);
    }

    // ================================================================
    //  数据读取工具
    // ================================================================

    /**
     * 从 BIN 缓冲读取 accessor 的 float 数据。
     * 正确处理交错布局 (byteStride)。
     */
    private float[] readAccessorFloats(ByteBuffer binBuf, JSONArray accessors,
                                       JSONArray bufferViews, int accIdx) throws Exception {
        JSONObject acc    = accessors.getJSONObject(accIdx);
        int bvIdx         = acc.getInt("bufferView");
        int accByteOffset = acc.optInt("byteOffset", 0);
        int compType      = acc.getInt("componentType");
        int count         = acc.getInt("count");
        String typeStr    = acc.getString("type");

        int numComps = getNumComponents(typeStr);
        int compSize = getComponentSize(compType);

        JSONObject bv      = bufferViews.getJSONObject(bvIdx);
        int bvByteOffset   = bv.getInt("byteOffset");
        int bvByteStride   = bv.optInt("byteStride", 0);

        int elemSize = numComps * compSize;
        int stride   = (bvByteStride > 0) ? bvByteStride : elemSize;

        float[] result = new float[count * numComps];
        int absOffset = bvByteOffset + accByteOffset;

        for (int i = 0; i < count; i++) {
            binBuf.position(absOffset + i * stride);
            for (int c = 0; c < numComps; c++) {
                switch (compType) {
                    case COMP_FLOAT:
                        result[i * numComps + c] = binBuf.getFloat();
                        break;
                    case COMP_UNSIGNED_SHORT:
                        result[i * numComps + c] = (float) (binBuf.getShort() & 0xFFFF);
                        break;
                    case COMP_UNSIGNED_BYTE:
                        result[i * numComps + c] = (float) (binBuf.get() & 0xFF);
                        break;
                    default:
                        result[i * numComps + c] = 0f;
                        break;
                }
            }
        }
        return result;
    }

    /**
     * 读取索引数据并追加到 allIndices。
     * 处理 UNSIGNED_SHORT 和 UNSIGNED_INT。
     */
    private void readIndices(ByteBuffer binBuf, JSONArray accessors, JSONArray bufferViews,
                             int accIdx, List<Short> out, int vertexBase) throws Exception {
        JSONObject acc    = accessors.getJSONObject(accIdx);
        int bvIdx         = acc.getInt("bufferView");
        int accByteOffset = acc.optInt("byteOffset", 0);
        int compType      = acc.getInt("componentType");
        int count         = acc.getInt("count");

        JSONObject bv    = bufferViews.getJSONObject(bvIdx);
        int bvOffset     = bv.getInt("byteOffset");
        int bvStride     = bv.optInt("byteStride", 0);

        int compSize = getComponentSize(compType);
        int stride   = (bvStride > 0) ? bvStride : compSize;
        int absOff   = bvOffset + accByteOffset;

        if (compType == COMP_UNSIGNED_SHORT) {
            for (int i = 0; i < count; i++) {
                binBuf.position(absOff + i * stride);
                out.add((short) ((binBuf.getShort() & 0xFFFF) + vertexBase));
            }
        } else if (compType == COMP_UNSIGNED_INT) {
            // 转换为 short (大部分 AR 模型的顶点 < 65535)
            for (int i = 0; i < count; i++) {
                binBuf.position(absOff + i * stride);
                long idx = binBuf.getInt() & 0xFFFFFFFFL;
                if (idx + vertexBase > 65535) {
                    Log.w(TAG, "索引 " + (idx + vertexBase) + " 超出 unsigned short 范围，截断");
                }
                out.add((short) ((int) (idx + vertexBase)));
            }
        } else if (compType == COMP_UNSIGNED_BYTE) {
            for (int i = 0; i < count; i++) {
                binBuf.position(absOff + i * stride);
                out.add((short) ((binBuf.get() & 0xFF) + vertexBase));
            }
        }
    }

    /**
     * 将解析出的数据转换为 Direct Buffers。
     */
    private void createBuffers(List<Float> verts, List<Float> tcs,
                               List<Float> norms, List<Short> idxs) {
        float[] va = new float[verts.size()];
        for (int i = 0; i < verts.size(); i++) va[i] = verts.get(i);
        float[] ta = new float[tcs.size()];
        for (int i = 0; i < tcs.size(); i++) ta[i] = tcs.get(i);
        float[] na = new float[norms.size()];
        for (int i = 0; i < norms.size(); i++) na[i] = norms.get(i);
        short[] ia = new short[idxs.size()];
        for (int i = 0; i < idxs.size(); i++) ia[i] = idxs.get(i);

        vertexBuffer = ByteBuffer.allocateDirect(va.length * 4)
                .order(ByteOrder.nativeOrder()).asFloatBuffer();
        vertexBuffer.put(va).position(0);

        texCoordBuffer = ByteBuffer.allocateDirect(ta.length * 4)
                .order(ByteOrder.nativeOrder()).asFloatBuffer();
        texCoordBuffer.put(ta).position(0);

        normalBuffer = ByteBuffer.allocateDirect(na.length * 4)
                .order(ByteOrder.nativeOrder()).asFloatBuffer();
        normalBuffer.put(na).position(0);

        indexBuffer = ByteBuffer.allocateDirect(ia.length * 2)
                .order(ByteOrder.nativeOrder()).asShortBuffer();
        indexBuffer.put(ia).position(0);
    }

    // ================================================================
    //  材质处理
    // ================================================================

    /**
     * 从 glTF 材质数组中提取并转换材质参数 (PBR → Blinn-Phong 近似)。
     * 同时尝试加载内嵌纹理。
     */
    private void applyMaterial(JSONObject gltf, int matIdx,
                               ByteBuffer binBuf, JSONArray bufferViews,
                               JSONArray accessors) {
        try {
            JSONArray materials = gltf.optJSONArray("materials");
            if (materials == null || matIdx < 0 || matIdx >= materials.length()) return;
            JSONObject mat = materials.getJSONObject(matIdx);

            // ---- PBR metallic-roughness → Blinn-Phong ----
            float[] baseColor    = {1f, 1f, 1f};
            float   metallic     = 1f;
            float   roughness    = 1f;
            int     baseColorTexIdx = -1;

            if (mat.has("pbrMetallicRoughness")) {
                JSONObject pbr = mat.getJSONObject("pbrMetallicRoughness");

                if (pbr.has("baseColorFactor")) {
                    JSONArray cf = pbr.getJSONArray("baseColorFactor");
                    baseColor[0] = (float) cf.getDouble(0);
                    baseColor[1] = (float) cf.getDouble(1);
                    baseColor[2] = (float) cf.getDouble(2);
                }

                metallic   = (float) pbr.optDouble("metallicFactor", 1.0);
                roughness  = (float) pbr.optDouble("roughnessFactor", 1.0);

                // 记录 baseColorTexture 索引，稍后加载
                if (pbr.has("baseColorTexture") && textureBitmap == null) {
                    baseColorTexIdx = pbr.getJSONObject("baseColorTexture").getInt("index");
                }
            }

            // 转换: PBR → Blinn-Phong
            float metalFactor = Math.min(metallic, 1f);
            materialDiffuse[0] = baseColor[0] * (1f - metalFactor * 0.5f);
            materialDiffuse[1] = baseColor[1] * (1f - metalFactor * 0.5f);
            materialDiffuse[2] = baseColor[2] * (1f - metalFactor * 0.5f);

            float s = 0.04f * (1f - metalFactor) + baseColor[0] * metalFactor;
            materialSpecular[0] = s;
            materialSpecular[1] = s;
            materialSpecular[2] = s;

            float r = Math.min(Math.max(roughness, 0f), 1f);
            materialShininess = (1f - r) * (1f - r) * 128f + 2f;

            Log.d(TAG, String.format("材质转换: PBR→BP diff=[%.2f,%.2f,%.2f] spec=%.2f shininess=%.1f",
                    materialDiffuse[0], materialDiffuse[1], materialDiffuse[2],
                    s, materialShininess));

            // ---- 加载内嵌纹理 ----
            if (baseColorTexIdx >= 0 && binBuf != null) {
                loadEmbeddedTexture(gltf, baseColorTexIdx, binBuf, bufferViews);
            }

        } catch (Exception e) {
            Log.w(TAG, "应用材质时出错，使用默认材质", e);
        }
    }

    /**
     * 从 GLB 的 BIN 缓冲中加载内嵌纹理 (texture → image → bufferView → bytes)。
     */
    private void loadEmbeddedTexture(JSONObject gltf, int textureIdx,
                                     ByteBuffer binBuf, JSONArray bufferViews) {
        if (textureBitmap != null) return;
        try {
            JSONArray textures = gltf.optJSONArray("textures");
            JSONArray images   = gltf.optJSONArray("images");
            if (textures == null || images == null) return;
            if (textureIdx < 0 || textureIdx >= textures.length()) return;

            int sourceIdx = textures.getJSONObject(textureIdx).getInt("source");
            JSONObject img = images.getJSONObject(sourceIdx);

            if (img.has("bufferView")) {
                int bvIdx = img.getInt("bufferView");
                JSONObject bv = bufferViews.getJSONObject(bvIdx);
                int offset = bv.getInt("byteOffset");
                int length = bv.getInt("byteLength");

                binBuf.position(offset);
                byte[] imgBytes = new byte[length];
                binBuf.get(imgBytes);

                textureBitmap = BitmapFactory.decodeByteArray(imgBytes, 0, imgBytes.length);
                if (textureBitmap != null) {
                    Log.i(TAG, "加载 GLB 内嵌纹理成功 (" + length + " bytes)");
                } else {
                    Log.w(TAG, "解码内嵌纹理失败 (" + length + " bytes)");
                }
            }
        } catch (Exception e) {
            Log.w(TAG, "加载 GLB 内嵌纹理失败", e);
        }
    }

    // ================================================================
    //  纹理上传
    // ================================================================

    private void uploadTexture(Bitmap bitmap) {
        GLES20.glGenTextures(1, textures, 0);
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, textures[0]);
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER,
                GLES20.GL_LINEAR_MIPMAP_LINEAR);
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER,
                GLES20.GL_LINEAR);
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_S,
                GLES20.GL_REPEAT);
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_T,
                GLES20.GL_REPEAT);
        GLUtils.texImage2D(GLES20.GL_TEXTURE_2D, 0, bitmap, 0);
        GLES20.glGenerateMipmap(GLES20.GL_TEXTURE_2D);
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, 0);
        bitmap.recycle();
    }

    private Bitmap createCheckerTexture() {
        int size = 256;
        Bitmap bmp = Bitmap.createBitmap(size, size, Bitmap.Config.ARGB_8888);
        for (int y = 0; y < size; y++) {
            for (int x = 0; x < size; x++) {
                int cs = 32;
                boolean white = ((x / cs) + (y / cs)) % 2 == 0;
                bmp.setPixel(x, y, white ? 0xFFFFFFFF : 0xFF808080);
            }
        }
        return bmp;
    }

    // ================================================================
    //  数学辅助
    // ================================================================

    private void calculateAutoScale() {
        float w = modelBounds[1] - modelBounds[0];
        float h = modelBounds[3] - modelBounds[2];
        float d = modelBounds[5] - modelBounds[4];
        float maxDim = Math.max(w, Math.max(h, d));
        if (maxDim > 0) {
            autoScaleFactor = TARGET_SIZE / maxDim;
        } else {
            autoScaleFactor = 1.0f;
        }
    }

    /**
     * 为没有法线的几何体计算平面法线（面法线平均）。
     */
    private void computeFlatNormals(List<Float> verts, List<Short> idxs, List<Float> outNorms) {
        int vertCount = verts.size() / 3;
        float[] normAccum = new float[vertCount * 3];

        for (int i = 0; i < idxs.size(); i += 3) {
            if (i + 2 >= idxs.size()) break;
            int i0 = idxs.get(i) & 0xFFFF;
            int i1 = idxs.get(i + 1) & 0xFFFF;
            int i2 = idxs.get(i + 2) & 0xFFFF;

            float ax = verts.get(i0 * 3), ay = verts.get(i0 * 3 + 1), az = verts.get(i0 * 3 + 2);
            float bx = verts.get(i1 * 3), by = verts.get(i1 * 3 + 1), bz = verts.get(i1 * 3 + 2);
            float cx = verts.get(i2 * 3), cy = verts.get(i2 * 3 + 1), cz = verts.get(i2 * 3 + 2);

            float e1x = bx - ax, e1y = by - ay, e1z = bz - az;
            float e2x = cx - ax, e2y = cy - ay, e2z = cz - az;

            // 叉积
            float nx = e1y * e2z - e1z * e2y;
            float ny = e1z * e2x - e1x * e2z;
            float nz = e1x * e2y - e1y * e2x;

            float len = (float) Math.sqrt(nx * nx + ny * ny + nz * nz);
            if (len > 0) { nx /= len; ny /= len; nz /= len; }

            normAccum[i0 * 3]     += nx; normAccum[i0 * 3 + 1] += ny; normAccum[i0 * 3 + 2] += nz;
            normAccum[i1 * 3]     += nx; normAccum[i1 * 3 + 1] += ny; normAccum[i1 * 3 + 2] += nz;
            normAccum[i2 * 3]     += nx; normAccum[i2 * 3 + 1] += ny; normAccum[i2 * 3 + 2] += nz;
        }

        for (int i = 0; i < vertCount; i++) {
            float nx = normAccum[i * 3], ny = normAccum[i * 3 + 1], nz = normAccum[i * 3 + 2];
            float len = (float) Math.sqrt(nx * nx + ny * ny + nz * nz);
            if (len > 0) { nx /= len; ny /= len; nz /= len; }
            outNorms.add(nx); outNorms.add(ny); outNorms.add(nz);
        }
    }

    // ================================================================
    //  节点遍历
    // ================================================================

    private void collectMeshes(JSONArray allNodes, int nodeIdx, List<Integer> out) {
        try {
            JSONObject node = allNodes.getJSONObject(nodeIdx);
            if (node.has("mesh")) {
                out.add(node.getInt("mesh"));
            }
            if (node.has("children")) {
                JSONArray children = node.getJSONArray("children");
                for (int i = 0; i < children.length(); i++) {
                    collectMeshes(allNodes, children.getInt(i), out);
                }
            }
        } catch (Exception e) {
            Log.w(TAG, "遍历节点时出错", e);
        }
    }

    // ================================================================
    //  工具方法
    // ================================================================

    private static int getNumComponents(String type) {
        switch (type) {
            case "SCALAR": return 1;
            case "VEC2":   return 2;
            case "VEC3":   return 3;
            case "VEC4":   return 4;
            case "MAT2":   return 4;
            case "MAT3":   return 9;
            case "MAT4":   return 16;
            default:       return 3;
        }
    }

    private static int getComponentSize(int compType) {
        switch (compType) {
            case COMP_FLOAT:          return 4;
            case COMP_UNSIGNED_INT:   return 4;
            case COMP_UNSIGNED_SHORT: return 2;
            case COMP_UNSIGNED_BYTE:  return 1;
            default:                  return 4;
        }
    }

    private static byte[] readAllBytes(InputStream is) throws IOException {
        java.io.ByteArrayOutputStream baos = new java.io.ByteArrayOutputStream();
        byte[] buf = new byte[8192];
        int n;
        while ((n = is.read(buf)) != -1) {
            baos.write(buf, 0, n);
        }
        return baos.toByteArray();
    }

    private static int loadGLShader(int type, String code) {
        int shader = GLES20.glCreateShader(type);
        GLES20.glShaderSource(shader, code);
        GLES20.glCompileShader(shader);
        int[] status = new int[1];
        GLES20.glGetShaderiv(shader, GLES20.GL_COMPILE_STATUS, status, 0);
        if (status[0] == 0) {
            Log.e(TAG, "着色器编译失败: " + GLES20.glGetShaderInfoLog(shader));
            GLES20.glDeleteShader(shader);
            shader = 0;
        }
        if (shader == 0) {
            throw new RuntimeException("创建着色器失败");
        }
        return shader;
    }
}
