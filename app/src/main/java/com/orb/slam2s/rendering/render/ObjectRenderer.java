package com.orb.slam2s.rendering.render;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.opengl.GLES20;
import android.opengl.GLUtils;
import android.opengl.Matrix;
import android.util.Log;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;
import java.util.ArrayList;
import java.util.List;

/**
 * 增强的3D对象渲染器，支持从OBJ文件加载现代格式。
 * 支持法线、材质、MTL文件和改进的光照。
 */
public class ObjectRenderer {
    private static final String TAG = "ObjectRenderer";

    // 增强的着色器代码，支持法线和光照
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
            "   \n" +
            "   // Transform normal to eye space\n" +
            "   v_Normal = normalize(u_NormalMatrix * a_Normal);\n" +
            "   \n" +
            "   // Calculate view space position\n" +
            "   vec4 viewPosition = u_ModelView * a_Position;\n" +
            "   v_ViewPosition = viewPosition.xyz;\n" +
            "   \n" +
            "   // Calculate light direction in view space\n" +
            "   v_LightDir = normalize(u_LightPosition - v_ViewPosition);\n" +
            "   \n" +
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
            "    // Sample texture\n" +
            "    vec4 textureColor = texture2D(u_Texture, v_TexCoord);\n" +
            "    \n" +
            "    // Normalize interpolated normal\n" +
            "    vec3 normal = normalize(v_Normal);\n" +
            "    vec3 lightDir = normalize(v_LightDir);\n" +
            "    vec3 viewDir = normalize(-v_ViewPosition);\n" +
            "    \n" +
            "    // Ambient lighting\n" +
            "    vec3 ambient = u_LightAmbient * u_MaterialAmbient;\n" +
            "    \n" +
            "    // Diffuse lighting\n" +
            "    float diff = max(dot(normal, lightDir), 0.0);\n" +
            "    vec3 diffuse = u_LightDiffuse * u_MaterialDiffuse * diff;\n" +
            "    \n" +
            "    // Specular lighting (Blinn-Phong)\n" +
            "    vec3 halfwayDir = normalize(lightDir + viewDir);\n" +
            "    float spec = pow(max(dot(normal, halfwayDir), 0.0), u_MaterialShininess);\n" +
            "    vec3 specular = u_LightSpecular * u_MaterialSpecular * spec;\n" +
            "    \n" +
            "    // Combine lighting with texture\n" +
            "    vec3 lighting = ambient + diffuse + specular;\n" +
            "    vec3 finalColor = lighting * textureColor.rgb;\n" +
            "    \n" +
            "    gl_FragColor = vec4(finalColor, textureColor.a * v_Alpha);\n" +
            "}";

    private int program;
    private int[] textures = new int[1];

    // 属性变量
    private int positionAttribute;
    private int texCoordAttribute;
    private int normalAttribute;

    // 矩阵uniform变量
    private int modelViewProjectionUniform;
    private int modelViewUniform;
    private int normalMatrixUniform;
    
    // 纹理和光照uniform变量
    private int textureUniform;
    private int lightPositionUniform;
    private int lightAmbientUniform;
    private int lightDiffuseUniform;
    private int lightSpecularUniform;
    
    // 材质uniform变量
    private int materialAmbientUniform;
    private int materialDiffuseUniform;
    private int materialSpecularUniform;
    private int materialShininessUniform;

    private FloatBuffer vertexBuffer;
    private FloatBuffer texCoordBuffer;
    private FloatBuffer normalBuffer;
    private ShortBuffer indexBuffer;

    private int vertexCount;
    private float[] modelMatrix = new float[16];
    private float autoScaleFactor = 1.0f;
    private float[] modelBounds = new float[6]; // 边界：minX, maxX, minY, maxY, minZ, maxZ
    private static final float TARGET_SIZE = 0.5f; // AR单位中的目标大小
    private static final int MAX_VERTICES = 50000; // 性能考虑的最大顶点数
    private static final int MAX_FACES = 100000; // 性能考虑的最大面数
    
    // 材质和光照属性
    private MaterialLoader materialLoader;
    private MaterialLoader.Material currentMaterial;
    private String mtlFileName = null;
    
    // 默认光照设置
    private float[] lightPosition = {2.0f, 4.0f, 3.0f};       // 世界空间光源位置
    private float[] lightAmbient = {0.3f, 0.3f, 0.3f};        // 环境光颜色
    private float[] lightDiffuse = {0.8f, 0.8f, 0.8f};        // 漫射光颜色
    private float[] lightSpecular = {1.0f, 1.0f, 1.0f};       // 镜面光颜色

    public ObjectRenderer() {
        Matrix.setIdentityM(modelMatrix, 0);
        materialLoader = new MaterialLoader();
        currentMaterial = materialLoader.getDefaultMaterial();
    }

    public void createOnGlThread(Context context, String objAssetName, String diffuseTextureAssetName)
            throws IOException {
        // 加载着色器程序
        int vertexShader = loadGLShader(GLES20.GL_VERTEX_SHADER, VERTEX_SHADER);
        int fragmentShader = loadGLShader(GLES20.GL_FRAGMENT_SHADER, FRAGMENT_SHADER);

        program = GLES20.glCreateProgram();
        GLES20.glAttachShader(program, vertexShader);
        GLES20.glAttachShader(program, fragmentShader);
        GLES20.glLinkProgram(program);
        GLES20.glUseProgram(program);

        // 获取属性位置
        positionAttribute = GLES20.glGetAttribLocation(program, "a_Position");
        texCoordAttribute = GLES20.glGetAttribLocation(program, "a_TexCoord");
        normalAttribute = GLES20.glGetAttribLocation(program, "a_Normal");

        // 获取矩阵uniform位置
        modelViewProjectionUniform = GLES20.glGetUniformLocation(program, "u_ModelViewProjection");
        modelViewUniform = GLES20.glGetUniformLocation(program, "u_ModelView");
        normalMatrixUniform = GLES20.glGetUniformLocation(program, "u_NormalMatrix");
        
        // 获取纹理和光照uniform位置
        textureUniform = GLES20.glGetUniformLocation(program, "u_Texture");
        lightPositionUniform = GLES20.glGetUniformLocation(program, "u_LightPosition");
        lightAmbientUniform = GLES20.glGetUniformLocation(program, "u_LightAmbient");
        lightDiffuseUniform = GLES20.glGetUniformLocation(program, "u_LightDiffuse");
        lightSpecularUniform = GLES20.glGetUniformLocation(program, "u_LightSpecular");
        
        // 获取材质uniform位置
        materialAmbientUniform = GLES20.glGetUniformLocation(program, "u_MaterialAmbient");
        materialDiffuseUniform = GLES20.glGetUniformLocation(program, "u_MaterialDiffuse");
        materialSpecularUniform = GLES20.glGetUniformLocation(program, "u_MaterialSpecular");
        materialShininessUniform = GLES20.glGetUniformLocation(program, "u_MaterialShininess");

        // 加载OBJ文件
        loadObjFile(context, objAssetName);

        // 加载纹理
        loadTexture(context, diffuseTextureAssetName);
    }

    private void loadObjFile(Context context, String objAssetName) throws IOException {
        // 增强的OBJ加载器，支持现代格式特性
        List<Float> vertices = new ArrayList<>();
        List<Float> texCoords = new ArrayList<>();
        List<Float> normals = new ArrayList<>();
        List<Short> indices = new ArrayList<>();

        // 用于解析数据的临时存储
        List<Float> tempVertices = new ArrayList<>();
        List<Float> tempTexCoords = new ArrayList<>(); 
        List<Float> tempNormals = new ArrayList<>();

        InputStream inputStream = context.getAssets().open(objAssetName);
        BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream));
        String line;

        Log.d(TAG, "开始加载OBJ文件: " + objAssetName);

        while ((line = reader.readLine()) != null) {
            line = line.trim();
            
            // 跳过空行和注释
            if (line.isEmpty() || line.startsWith("#")) {
                continue;
            }

            String[] parts = line.split("\\s+");
            if (parts.length == 0) continue;

            String command = parts[0].toLowerCase();
            
            switch (command) {
                case "mtllib":
                    // 材质库 - 加载MTL文件
                    if (parts.length >= 2) {
                        mtlFileName = parts[1];
                        Log.d(TAG, "发现MTL文件引用: " + mtlFileName);
                        if (materialLoader.loadMTL(context, mtlFileName)) {
                            Log.i(TAG, "成功加载MTL文件: " + mtlFileName);
                        } else {
                            Log.w(TAG, "加载MTL文件失败: " + mtlFileName);
                        }
                    }
                    break;
                    
                case "usemtl":
                    // 使用材质 - 切换当前材质
                    if (parts.length >= 2) {
                        String materialName = parts[1];
                        MaterialLoader.Material material = materialLoader.getMaterial(materialName);
                        if (material != null) {
                            currentMaterial = material;
                            Log.d(TAG, "切换到材质: " + materialName);
                        } else {
                            Log.w(TAG, "未找到材质: " + materialName + "，使用默认材质");
                        }
                    }
                    break;
                    
                case "o":
                case "g":
                    // 对象/组名称 - 用于调试日志
                    if (parts.length >= 2) {
                        Log.d(TAG, "对象/组: " + parts[1]);
                    }
                    break;

                case "v":
                    // 顶点位置
                    if (parts.length >= 4) {
                        tempVertices.add(Float.parseFloat(parts[1]));
                        tempVertices.add(Float.parseFloat(parts[2]));
                        tempVertices.add(Float.parseFloat(parts[3]));
                    }
                    break;

                case "vt":
                    // 纹理坐标 - 修正V坐标方向
                    if (parts.length >= 3) {
                        tempTexCoords.add(Float.parseFloat(parts[1]));
                        tempTexCoords.add(1.0f - Float.parseFloat(parts[2])); // 翻转V坐标
                    }
                    break;

                case "vn":
                    // 顶点法线
                    if (parts.length >= 4) {
                        tempNormals.add(Float.parseFloat(parts[1]));
                        tempNormals.add(Float.parseFloat(parts[2]));
                        tempNormals.add(Float.parseFloat(parts[3]));
                    }
                    break;

                case "f":
                    // 面 - 支持多种格式：f v, f v/vt, f v/vt/vn, f v//vn
                    if (parts.length >= 4) { // 三角形至少需要3个顶点
                        for (int i = 1; i < parts.length; i++) {
                            String[] faceData = parts[i].split("/");
                            
                            // 顶点索引（必需）
                            int vertexIndex = Integer.parseInt(faceData[0]) - 1;
                            
                            // 纹理坐标索引（可选）
                            int texCoordIndex = -1;
                            if (faceData.length > 1 && !faceData[1].isEmpty()) {
                                texCoordIndex = Integer.parseInt(faceData[1]) - 1;
                            }
                            
                            // 法线索引（可选）
                            int normalIndex = -1;
                            if (faceData.length > 2 && !faceData[2].isEmpty()) {
                                normalIndex = Integer.parseInt(faceData[2]) - 1;
                            }

                            // 添加顶点位置
                            if (vertexIndex >= 0 && vertexIndex * 3 + 2 < tempVertices.size()) {
                                vertices.add(tempVertices.get(vertexIndex * 3));
                                vertices.add(tempVertices.get(vertexIndex * 3 + 1));
                                vertices.add(tempVertices.get(vertexIndex * 3 + 2));
                            } else {
                                Log.e(TAG, "无效的顶点索引: " + vertexIndex);
                                continue;
                            }

                            // 添加纹理坐标（如果不可用则使用程序生成的UV）
                            if (texCoordIndex >= 0 && texCoordIndex * 2 + 1 < tempTexCoords.size()) {
                                texCoords.add(tempTexCoords.get(texCoordIndex * 2));
                                texCoords.add(tempTexCoords.get(texCoordIndex * 2 + 1));
                            } else {
                                // 基于顶点位置生成更好的程序化UV坐标
                                if (vertexIndex >= 0 && vertexIndex * 3 + 2 < tempVertices.size()) {
                                    float x = tempVertices.get(vertexIndex * 3);
                                    float y = tempVertices.get(vertexIndex * 3 + 1);
                                    float z = tempVertices.get(vertexIndex * 3 + 2);
                                    
                                    // 使用球面映射以获得更好的UV分布
                                    float length = (float) Math.sqrt(x * x + y * y + z * z);
                                    if (length > 0) {
                                        // 归一化到单位球
                                        float nx = x / length;
                                        float ny = y / length;
                                        float nz = z / length;
                                        
                                        // 球面坐标转UV
                                        float u = (float) (0.5 + Math.atan2(nz, nx) / (2 * Math.PI));
                                        float v = (float) (0.5 - Math.asin(ny) / Math.PI);
                                        
                                        texCoords.add(u);
                                        texCoords.add(v);
                                    } else {
                                        texCoords.add(0.5f);
                                        texCoords.add(0.5f);
                                    }
                                } else {
                                    texCoords.add(0.5f);  // 默认为中心
                                    texCoords.add(0.5f);
                                }
                            }

                            // 添加法线（如果不可用则使用默认值）
                            if (normalIndex >= 0 && normalIndex * 3 + 2 < tempNormals.size()) {
                                normals.add(tempNormals.get(normalIndex * 3));
                                normals.add(tempNormals.get(normalIndex * 3 + 1));
                                normals.add(tempNormals.get(normalIndex * 3 + 2));
                            } else {
                                // 默认法线向上
                                normals.add(0.0f);
                                normals.add(1.0f);
                                normals.add(0.0f);
                            }

                            indices.add((short) (indices.size()));
                        }
                    }
                    break;
                    
                default:
                    // 忽略未知命令
                    Log.v(TAG, "忽略未知OBJ命令: " + command);
                    break;
            }
        }
        reader.close();

        vertexCount = indices.size();
        
        Log.i(TAG, String.format("OBJ加载完成 - 顶点:%d, 纹理坐标:%d, 法向量:%d, 面:%d", 
                tempVertices.size()/3, tempTexCoords.size()/2, tempNormals.size()/3, indices.size()/3));
        
        // 检查模型是否过于复杂并提示优化建议
        if (tempVertices.size()/3 > MAX_VERTICES) {
            Log.w(TAG, String.format("[警告] 模型顶点数量较多 (%d > %d)，可能影响性能", 
                    tempVertices.size()/3, MAX_VERTICES));
        }
        
        if (indices.size()/3 > MAX_FACES) {
            Log.w(TAG, String.format("[警告] 模型面数量较多 (%d > %d)，可能影响性能", 
                    indices.size()/3, MAX_FACES));
        }
        
        // 打印模型边界信息用于调试
        if (!tempVertices.isEmpty()) {
            float minX = tempVertices.get(0), maxX = tempVertices.get(0);
            float minY = tempVertices.get(1), maxY = tempVertices.get(1);
            float minZ = tempVertices.get(2), maxZ = tempVertices.get(2);
            
            for (int i = 0; i < tempVertices.size(); i += 3) {
                float x = tempVertices.get(i);
                float y = tempVertices.get(i + 1);
                float z = tempVertices.get(i + 2);
                
                minX = Math.min(minX, x); maxX = Math.max(maxX, x);
                minY = Math.min(minY, y); maxY = Math.max(maxY, y);
                minZ = Math.min(minZ, z); maxZ = Math.max(maxZ, z);
            }
            
        Log.i(TAG, String.format("模型边界: X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f]", 
                minX, maxX, minY, maxY, minZ, maxZ));
                
            // 存储边界并计算自动缩放
            modelBounds[0] = minX; modelBounds[1] = maxX;
            modelBounds[2] = minY; modelBounds[3] = maxY; 
            modelBounds[4] = minZ; modelBounds[5] = maxZ;
            
            calculateAutoScale();
        }
        
        Log.i(TAG, "当前材质: " + (currentMaterial != null ? currentMaterial.name : "默认材质"));
        Log.i(TAG, "MTL文件: " + (mtlFileName != null ? mtlFileName : "未找到"));
        
        if (tempTexCoords.isEmpty()) {
            Log.w(TAG, "[警告] OBJ文件没有纹理坐标，将使用程序生成的UV坐标");
        }

        // 转换为缓冲区
        float[] vertexArray = new float[vertices.size()];
        for (int i = 0; i < vertices.size(); i++) {
            vertexArray[i] = vertices.get(i);
        }

        float[] texCoordArray = new float[texCoords.size()];
        for (int i = 0; i < texCoords.size(); i++) {
            texCoordArray[i] = texCoords.get(i);
        }

        float[] normalArray = new float[normals.size()];
        for (int i = 0; i < normals.size(); i++) {
            normalArray[i] = normals.get(i);
        }

        short[] indexArray = new short[indices.size()];
        for (int i = 0; i < indices.size(); i++) {
            indexArray[i] = indices.get(i);
        }

        // 创建OpenGL缓冲区
        vertexBuffer = ByteBuffer.allocateDirect(vertexArray.length * 4)
                .order(ByteOrder.nativeOrder()).asFloatBuffer();
        vertexBuffer.put(vertexArray).position(0);

        texCoordBuffer = ByteBuffer.allocateDirect(texCoordArray.length * 4)
                .order(ByteOrder.nativeOrder()).asFloatBuffer();
        texCoordBuffer.put(texCoordArray).position(0);

        normalBuffer = ByteBuffer.allocateDirect(normalArray.length * 4)
                .order(ByteOrder.nativeOrder()).asFloatBuffer();
        normalBuffer.put(normalArray).position(0);

        indexBuffer = ByteBuffer.allocateDirect(indexArray.length * 2)
                .order(ByteOrder.nativeOrder()).asShortBuffer();
        indexBuffer.put(indexArray).position(0);
        
        Log.d(TAG, "OBJ文件缓冲区创建完成");
        
        // 对大模型强制垃圾回收以释放临时内存
        if (tempVertices.size() > MAX_VERTICES || indices.size() > MAX_FACES) {
            Log.d(TAG, "清理临时内存...");
            System.gc();
        }
    }

    private void loadTexture(Context context, String textureAssetName) throws IOException {
        Bitmap bitmap = null;
        InputStream inputStream = null;
        
        // 首先尝试加载指定的纹理
        try {
            inputStream = context.getAssets().open(textureAssetName);
            bitmap = BitmapFactory.decodeStream(inputStream);
            Log.i(TAG, "成功加载纹理: " + textureAssetName);
        } catch (IOException e) {
            Log.w(TAG, "无法加载指定纹理 " + textureAssetName + "，尝试自动检测...");
            
            // 如果原始文件失败，尝试不同的扩展名
            String baseName = textureAssetName.substring(0, textureAssetName.lastIndexOf('.'));
            String[] extensions = {".png", ".jpg", ".jpeg", ".bmp"};
            
            for (String ext : extensions) {
                try {
                    if (inputStream != null) inputStream.close();
                    inputStream = context.getAssets().open(baseName + ext);
                    bitmap = BitmapFactory.decodeStream(inputStream);
                    Log.i(TAG, "找到替代纹理: " + baseName + ext);
                    break;
                } catch (IOException ignored) {
                    // 继续尝试
                }
            }
            
            // 如果仍未找到纹理，创建一个默认的
            if (bitmap == null) {
                Log.w(TAG, "未找到任何纹理文件，创建默认纹理");
                bitmap = createDefaultTexture();
            }
        } finally {
            if (inputStream != null) {
                try {
                    inputStream.close();
                } catch (IOException ignored) {}
            }
        }

        if (bitmap != null) {
            GLES20.glGenTextures(1, textures, 0);
            GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, textures[0]);

            GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR_MIPMAP_LINEAR);
            GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);
            GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_REPEAT);
            GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_REPEAT);
            
            GLUtils.texImage2D(GLES20.GL_TEXTURE_2D, 0, bitmap, 0);
            GLES20.glGenerateMipmap(GLES20.GL_TEXTURE_2D);
            GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, 0);

            bitmap.recycle();
            Log.i(TAG, "纹理加载完成");
        } else {
            throw new IOException("Failed to load texture: " + textureAssetName);
        }
    }
    
    /**
     * 当没有纹理文件可用时创建默认的程序化纹理
     */
    private Bitmap createDefaultTexture() {
        int size = 256;
        Bitmap bitmap = Bitmap.createBitmap(size, size, Bitmap.Config.ARGB_8888);
        
        // 创建棋盘图案
        for (int y = 0; y < size; y++) {
            for (int x = 0; x < size; x++) {
                int checkSize = 32;
                boolean isWhite = ((x / checkSize) + (y / checkSize)) % 2 == 0;
                int color = isWhite ? 0xFFFFFFFF : 0xFF808080; // 白色或灰色
                bitmap.setPixel(x, y, color);
            }
        }
        
        Log.i(TAG, "创建了默认棋盘纹理 (" + size + "x" + size + ")");
        return bitmap;
    }

    public void updateModelMatrix(float[] modelMatrix, float scaleFactor) {
        float[] scaleMatrix = new float[16];
        Matrix.setIdentityM(scaleMatrix, 0);
        // 同时应用用户缩放因子和自动缩放因子
        float finalScale = scaleFactor * autoScaleFactor;
        Matrix.scaleM(scaleMatrix, 0, finalScale, finalScale, finalScale);
        Matrix.multiplyMM(this.modelMatrix, 0, modelMatrix, 0, scaleMatrix, 0);
    }
    
    /**
     * 计算自动缩放因子以使模型适应目标大小
     */
    private void calculateAutoScale() {
        if (modelBounds == null) return;
        
        float width = modelBounds[1] - modelBounds[0];  // maxX - minX
        float height = modelBounds[3] - modelBounds[2]; // maxY - minY
        float depth = modelBounds[5] - modelBounds[4];  // maxZ - minZ
        
        // 找到最大维度
        float maxDimension = Math.max(width, Math.max(height, depth));
        
        if (maxDimension > 0) {
            autoScaleFactor = TARGET_SIZE / maxDimension;
            Log.i(TAG, String.format("自动缩放计算: 最大尺寸=%.2f, 缩放系数=%.3f", 
                    maxDimension, autoScaleFactor));
        } else {
            autoScaleFactor = 1.0f;
            Log.w(TAG, "无法计算模型尺寸，使用默认缩放");
        }
    }
    
    /**
     * 获取当前的自动缩放因子（用于调试）
     */
    public float getAutoScaleFactor() {
        return autoScaleFactor;
    }


    public void draw(float[] cameraView, float[] cameraPerspective, float lightIntensity) {
        GLES20.glUseProgram(program);

        // 计算用于光照和渲染的矩阵
        float[] modelView = new float[16];
        float[] modelViewProjection = new float[16];
        float[] normalMatrix = new float[9];
        
        Matrix.multiplyMM(modelView, 0, cameraView, 0, modelMatrix, 0);
        Matrix.multiplyMM(modelViewProjection, 0, cameraPerspective, 0, modelView, 0);
        
        // 计算法线矩阵（从modelView的左上角3x3提取，然后求逆和转置）
        // 现在使用从modelView简化的法线矩阵
        normalMatrix[0] = modelView[0]; normalMatrix[1] = modelView[1]; normalMatrix[2] = modelView[2];
        normalMatrix[3] = modelView[4]; normalMatrix[4] = modelView[5]; normalMatrix[5] = modelView[6];
        normalMatrix[6] = modelView[8]; normalMatrix[7] = modelView[9]; normalMatrix[8] = modelView[10];

        // 设置矩阵uniform
        GLES20.glUniformMatrix4fv(modelViewProjectionUniform, 1, false, modelViewProjection, 0);
        GLES20.glUniformMatrix4fv(modelViewUniform, 1, false, modelView, 0);
        GLES20.glUniformMatrix3fv(normalMatrixUniform, 1, false, normalMatrix, 0);

        // 设置光照uniform
        float[] adjustedLightPos = {
            lightPosition[0] * lightIntensity,
            lightPosition[1] * lightIntensity,
            lightPosition[2] * lightIntensity
        };
        GLES20.glUniform3fv(lightPositionUniform, 1, adjustedLightPos, 0);
        
        float[] adjustedAmbient = {
            lightAmbient[0] * lightIntensity,
            lightAmbient[1] * lightIntensity,
            lightAmbient[2] * lightIntensity
        };
        float[] adjustedDiffuse = {
            lightDiffuse[0] * lightIntensity,
            lightDiffuse[1] * lightIntensity,
            lightDiffuse[2] * lightIntensity
        };
        GLES20.glUniform3fv(lightAmbientUniform, 1, adjustedAmbient, 0);
        GLES20.glUniform3fv(lightDiffuseUniform, 1, adjustedDiffuse, 0);
        GLES20.glUniform3fv(lightSpecularUniform, 1, lightSpecular, 0);

        // 从当前材质设置材质uniform
        if (currentMaterial != null) {
            GLES20.glUniform3fv(materialAmbientUniform, 1, currentMaterial.ambientColor, 0);
            GLES20.glUniform3fv(materialDiffuseUniform, 1, currentMaterial.diffuseColor, 0);
            GLES20.glUniform3fv(materialSpecularUniform, 1, currentMaterial.specularColor, 0);
            GLES20.glUniform1f(materialShininessUniform, currentMaterial.shininess);
            
            // Log.v(TAG, String.format("使用材质: %s (环境光: [%.2f,%.2f,%.2f], 漫射: [%.2f,%.2f,%.2f], 光泽度: %.1f)",
            //         currentMaterial.name,
            //         currentMaterial.ambientColor[0], currentMaterial.ambientColor[1], currentMaterial.ambientColor[2],
            //         currentMaterial.diffuseColor[0], currentMaterial.diffuseColor[1], currentMaterial.diffuseColor[2],
            //         currentMaterial.shininess));  // 高频日志已注释
        } else {
            // 使用默认材质值
            float[] defaultAmbient = {0.2f, 0.2f, 0.2f};
            float[] defaultDiffuse = {0.8f, 0.8f, 0.8f};
            float[] defaultSpecular = {0.5f, 0.5f, 0.5f};
            GLES20.glUniform3fv(materialAmbientUniform, 1, defaultAmbient, 0);
            GLES20.glUniform3fv(materialDiffuseUniform, 1, defaultDiffuse, 0);
            GLES20.glUniform3fv(materialSpecularUniform, 1, defaultSpecular, 0);
            GLES20.glUniform1f(materialShininessUniform, 32.0f);
        }

        // 设置纹理
        GLES20.glActiveTexture(GLES20.GL_TEXTURE0);
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, textures[0]);
        GLES20.glUniform1i(textureUniform, 0);

        // 设置顶点属性
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0);

        // 位置属性
        GLES20.glVertexAttribPointer(positionAttribute, 3, GLES20.GL_FLOAT, false, 0, vertexBuffer);
        GLES20.glEnableVertexAttribArray(positionAttribute);

        // 纹理坐标属性
        GLES20.glVertexAttribPointer(texCoordAttribute, 2, GLES20.GL_FLOAT, false, 0, texCoordBuffer);
        GLES20.glEnableVertexAttribArray(texCoordAttribute);

        // 法线属性（如果可用）
        if (normalBuffer != null) {
            GLES20.glVertexAttribPointer(normalAttribute, 3, GLES20.GL_FLOAT, false, 0, normalBuffer);
            GLES20.glEnableVertexAttribArray(normalAttribute);
            // Log.v(TAG, "启用法向量attribute");  // 高频日志已注释
        } else {
            // 为每个顶点提供默认法线（向上）
            GLES20.glDisableVertexAttribArray(normalAttribute);
            GLES20.glVertexAttrib3f(normalAttribute, 0.0f, 1.0f, 0.0f);
            // Log.v(TAG, "使用默认法向量");  // 高频日志已注释
        }

        // 启用渲染状态
        GLES20.glEnable(GLES20.GL_DEPTH_TEST);
        GLES20.glEnable(GLES20.GL_BLEND);
        GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA);

        // 绘制对象
        GLES20.glDrawElements(GLES20.GL_TRIANGLES, vertexCount, GLES20.GL_UNSIGNED_SHORT, indexBuffer);

        // 禁用属性
        GLES20.glDisableVertexAttribArray(positionAttribute);
        GLES20.glDisableVertexAttribArray(texCoordAttribute);
        if (normalBuffer != null) {
            GLES20.glDisableVertexAttribArray(normalAttribute);
        }

        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, 0);
    }

    private static int loadGLShader(int type, String code) {
        int shader = GLES20.glCreateShader(type);
        GLES20.glShaderSource(shader, code);
        GLES20.glCompileShader(shader);

        // 获取编译状态
        final int[] compileStatus = new int[1];
        GLES20.glGetShaderiv(shader, GLES20.GL_COMPILE_STATUS, compileStatus, 0);

        // 如果编译失败，删除着色器
        if (compileStatus[0] == 0) {
            Log.e(TAG, "着色器编译错误: " + GLES20.glGetShaderInfoLog(shader));
            GLES20.glDeleteShader(shader);
            shader = 0;
        }

        if (shader == 0) {
            throw new RuntimeException("创建着色器时出错");
        }

        return shader;
    }
}
