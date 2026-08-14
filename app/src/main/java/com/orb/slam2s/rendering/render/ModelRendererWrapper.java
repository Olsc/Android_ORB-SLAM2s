package com.orb.slam2s.rendering.render;

/*
 * Copyright 2017 Google Inc. All Rights Reserved.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

import android.content.Context;
import android.graphics.PixelFormat;
import android.view.Choreographer;
import android.view.Surface;
import android.util.Log;

import com.orb.slam2s.constant.GlobalConstant;
import com.orb.slam2s.rendering.gles.FilamentAspectSurfaceView;
import com.orb.slam2s.slamar.NativeHelper;
import com.orb.slam2s.utils.TouchHelper;

import com.google.android.filament.Engine;
import com.google.android.filament.Renderer;
import com.google.android.filament.Scene;
import com.google.android.filament.View;
import com.google.android.filament.Camera;
import com.google.android.filament.Box;
import com.google.android.filament.SwapChain;
import com.google.android.filament.Viewport;
import com.google.android.filament.EntityManager;
import com.google.android.filament.LightManager;
import com.google.android.filament.TransformManager;
import com.google.android.filament.IndirectLight;
import com.google.android.filament.MaterialInstance;
import com.google.android.filament.RenderableManager;
import com.google.android.filament.android.DisplayHelper;
import com.google.android.filament.android.UiHelper;
import com.google.android.filament.gltfio.AssetLoader;
import com.google.android.filament.gltfio.FilamentAsset;
import com.google.android.filament.gltfio.MaterialProvider;
import com.google.android.filament.gltfio.ResourceLoader;
import com.google.android.filament.gltfio.UbershaderProvider;
import com.google.android.filament.utils.Utils;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

// 用于在 AR 环境中渲染 3D 模型（GLB 格式）的包装类，基于 Google Filament 渲染引擎
public class ModelRendererWrapper implements NativeHelper.OnMVPUpdatedCallback {
    private static final String TAG = "ModelRendererWrapper";

    static {
        // 初始化 Filament 运行环境
        Utils.init();
    }

    private FilamentAspectSurfaceView arObjectView;
    private Context context;
    private NativeHelper nativeHelper;

    private String modelPath;
    private float initSize = 1.0f;

    private boolean isInitialized = false;
    private boolean shouldDraw = false;

    // 存储来自 NativeHelper 回调的矩阵 (OpenGL 格式)
    private final float[] modelMatrix = new float[16];
    private final float[] viewMatrix = new float[16];
    private final float[] projectionMatrix = new float[16];
    private boolean matricesReady = false;

    // 自动缩放和中心化相关的变量
    private final float[] modelCenter = new float[3];
    private final float[] modelHalfExtent = new float[3];
    private float autoScaleFactor = 1.0f;
    private boolean hasBoundingBox = false;

    // 双指缩放相关
    private float currentScaleFactor = 1.0f;  // 当前累积的缩放因子
    private static final float MIN_SCALE = 0.05f;  // 最小缩放比例
    private static final float MAX_SCALE = 10.0f;  // 最大缩放比例

    // 用户旋转控制（摇杆）
    private float userRotationY = 0.0f;      // Y轴旋转累积角度（度，yaw）
    private float userRotationX = 0.0f;      // X轴旋转累积角度（度，pitch）

    // AR物体显隐状态监听器（用于控制摇杆等UI元素）
    public interface DrawStateListener {
        void onDrawStateChanged(boolean shouldDraw);
    }
    private DrawStateListener drawStateListener;

    // 缓存矩阵以避免在 render 循环中 new 对象引发 GC 卡顿
    private final float[] tempCameraModelMatrix = new float[16];
    private final double[] tempDoubleProj = new double[16];
    private final float[] tempTransformMatrix = new float[16];
    private final float[] tempScaledModelMatrix = new float[16];
    private final float[] tempHiddenMatrix = new float[16];

    // Filament 核心对象
    private Engine engine;
    private Renderer renderer;
    private Scene scene;
    private View view;
    private Camera camera;
    private SwapChain swapChain;
    private UiHelper uiHelper;
    private DisplayHelper displayHelper;

    // gltfio 核心加载对象
    private MaterialProvider materialProvider;
    private AssetLoader assetLoader;
    private ResourceLoader resourceLoader;
    private FilamentAsset asset;
    private IndirectLight indirectLight;

    // 灯光实体列表
    private final List<Integer> lightEntities = new ArrayList<>();

    // 编舞者 (Choreographer) 渲染帧循环相关
    private Choreographer choreographer;
    private boolean isFrameCallbackActive = false;

    private final Choreographer.FrameCallback frameCallback = new Choreographer.FrameCallback() {
        @Override
        public void doFrame(long frameTimeNanos) {
            if (isFrameCallbackActive) {
                choreographer.postFrameCallback(this);
            }
            // 始终执行 render() — 丢失追踪时 render 内部会隐藏模型并渲染透明帧
            render(frameTimeNanos);
        }
    };

    private ModelRendererWrapper() {
        // 用单位矩阵初始化矩阵
        android.opengl.Matrix.setIdentityM(modelMatrix, 0);
        android.opengl.Matrix.setIdentityM(viewMatrix, 0);
        android.opengl.Matrix.setIdentityM(projectionMatrix, 0);
    }

    public static ModelRendererWrapper newInstance() {
        return new ModelRendererWrapper();
    }

    public ModelRendererWrapper setArObjectView(FilamentAspectSurfaceView arObjectView) {
        this.arObjectView = arObjectView;
        return this;
    }

    public ModelRendererWrapper setContext(Context context) {
        this.context = context;
        return this;
    }

    public ModelRendererWrapper setNativeHelper(NativeHelper nativeHelper) {
        this.nativeHelper = nativeHelper;
        return this;
    }

    public ModelRendererWrapper setModelPath(String modelPath) {
        this.modelPath = modelPath;
        return this;
    }

    public ModelRendererWrapper setInitSize(float initSize) {
        this.initSize = initSize;
        return this;
    }

    public ModelRendererWrapper setDrawStateListener(DrawStateListener listener) {
        this.drawStateListener = listener;
        return this;
    }

    public ModelRendererWrapper init(TouchHelper touchHelper) {
        if (arObjectView == null) {
            Log.e(TAG, "ArObjectView为空，无法初始化");
            return this;
        }

        // 配置 SurfaceView 保持背景透明且在最上层
        arObjectView.getHolder().setFormat(PixelFormat.TRANSLUCENT);
        arObjectView.setZOrderOnTop(true);

        // 添加双指缩放回调
        if (touchHelper != null) {
            touchHelper.addScalingCallback(new TouchHelper.ScalingCallback() {
                @Override
                public void updateScale(float scaleFactor) {
                    if (shouldDraw && nativeHelper != null) {
                        currentScaleFactor *= scaleFactor;
                        if (currentScaleFactor < MIN_SCALE) {
                            currentScaleFactor = MIN_SCALE;
                        } else if (currentScaleFactor > MAX_SCALE) {
                            currentScaleFactor = MAX_SCALE;
                        }
                        // 同步缩放值到C++，确保保存时正确
                        nativeHelper.updateArObjectScale(scaleFactor);
                    }
                }
            });
        }

        choreographer = Choreographer.getInstance();
        displayHelper = new DisplayHelper(context);

        // 初始化 Filament
        setupFilament();

        return this;
    }

    private void setupFilament() {
        Log.d(TAG, "正在初始化 Filament 引擎...");
        engine = Engine.create();
        renderer = engine.createRenderer();
        scene = engine.createScene();

        // 创建相机实体与组件，设置曝光度
        int cameraEntity = EntityManager.get().create();
        camera = engine.createCamera(cameraEntity);
        camera.setExposure(16.0f, 1.0f / 125.0f, 100.0f);

        // 配置 View
        view = engine.createView();
        view.setScene(scene);
        view.setCamera(camera);

        // 开启透明背景混合模式
        view.setBlendMode(View.BlendMode.TRANSLUCENT);

        // 设置适合移动端的低配渲染质量
        view.setRenderQuality(new View.RenderQuality());
        View.RenderQuality quality = view.getRenderQuality();
        quality.hdrColorBuffer = View.QualityLevel.LOW;
        view.setRenderQuality(quality);

        // 关闭后期处理（防背景变黑或造成卡顿）
        view.setPostProcessingEnabled(false);
        view.setSampleCount(1); // 关闭多重采样抗锯齿 (MSAA)，提升性能

        // 开启动态分辨率（在移动端设备上对性能提升巨大）
        View.DynamicResolutionOptions options = new View.DynamicResolutionOptions();
        options.enabled = true;
        view.setDynamicResolutionOptions(options);

        // 配置渲染器清除选项（清除背景为全透明）
        Renderer.ClearOptions clearOptions = renderer.getClearOptions();
        clearOptions.clearColor = new double[]{0.0, 0.0, 0.0, 0.0};
        clearOptions.clear = true;
        renderer.setClearOptions(clearOptions);

        // 创建 gltf 加载组件
        materialProvider = new UbershaderProvider(engine);
        assetLoader = new AssetLoader(engine, materialProvider, EntityManager.get());
        resourceLoader = new ResourceLoader(engine, false); // false=不生成mipmap,节省GPU内存

        // 创建三点定向光源，让 PBR 材质展示更佳
        createLights();

        // 绑定 Surface 生命周期与 SwapChain 建立
        uiHelper = new UiHelper(UiHelper.ContextErrorPolicy.DONT_CHECK);
        uiHelper.setOpaque(false);
        uiHelper.setRenderCallback(new UiHelper.RendererCallback() {
            @Override
            public void onNativeWindowChanged(Surface surface) {
                Log.d(TAG, "Surface 创建或变更，重新创建 SwapChain");
                if (swapChain != null) {
                    engine.destroySwapChain(swapChain);
                }
                swapChain = engine.createSwapChain(surface);
                displayHelper.attach(renderer, arObjectView.getDisplay());

                if (!isInitialized) {
                    loadModelAsync();
                }
            }

            @Override
            public void onDetachedFromSurface() {
                Log.d(TAG, "Surface 销毁，注销 SwapChain");
                if (swapChain != null) {
                    engine.destroySwapChain(swapChain);
                    swapChain = null;
                }
                displayHelper.detach();
            }

            @Override
            public void onResized(int width, int height) {
                Log.d(TAG, "Surface 大小变更为: " + width + "x" + height);
                view.setViewport(new Viewport(0, 0, width, height));
            }
        });
        uiHelper.attachTo(arObjectView);
    }

    private void createLights() {

        // 1. 主光源 - 纯白
        int mainLight = EntityManager.get().create();
        new LightManager.Builder(LightManager.Type.DIRECTIONAL)
                .color(1.0f, 1.0f, 1.0f)
                .intensity(100000.0f)
                .direction(-0.5f, -1.0f, -0.5f)
                .castShadows(false)
                .build(engine, mainLight);
        scene.addEntity(mainLight);
        lightEntities.add(mainLight);

        // 2. 补光 - 纯白
        int fillLight = EntityManager.get().create();
        new LightManager.Builder(LightManager.Type.DIRECTIONAL)
                .color(1.0f, 1.0f, 1.0f)
                .intensity(50000.0f)
                .direction(0.5f, 1.0f, 0.5f)
                .castShadows(false)
                .build(engine, fillLight);
        scene.addEntity(fillLight);
        lightEntities.add(fillLight);

        // 3. 环境光 (IndirectLight) - 纯白
        float[] sh = new float[27];
        sh[0] = 1.0f; sh[1] = 1.0f; sh[2] = 1.0f;
        for (int i = 3; i < 27; i++) sh[i] = 0.0f;

        try {
            indirectLight = new IndirectLight.Builder()
                    .irradiance(3, sh)
                    .intensity(30000.0f)
                    .build(engine);
            scene.setIndirectLight(indirectLight);
            Log.d(TAG, "纯白环境光已创建");
        } catch (Exception e) {
            Log.e(TAG, "创建环境光时失败", e);
        }
    }

    private void loadModelAsync() {
        if (context == null || modelPath == null) return;

        Log.i(TAG, "开始异步解析加载 GLB: " + modelPath);
        new Thread(new Runnable() {
            @Override
            public void run() {
                try (InputStream is = context.getAssets().open(modelPath)) {
                    byte[] bytes = new byte[is.available()];
                    int bytesRead = is.read(bytes);
                    Log.d(TAG, "已读取 GLB 字节大小: " + bytesRead);

                    final ByteBuffer buffer = ByteBuffer.allocateDirect(bytes.length);
                    buffer.put(bytes);
                    buffer.flip();

                    // 转回 UI 线程添加至场景，保证与主渲染环境线程同步
                    arObjectView.post(new Runnable() {
                        @Override
                        public void run() {
                            try {
                                asset = assetLoader.createAsset(buffer);
                                if (asset == null) {
                                    Log.e(TAG, "AssetLoader 创建模型失败");
                                    return;
                                }

                                // 获取模型包围盒并计算自动缩放因子
                                try {
                                    Box box = asset.getBoundingBox();
                                    if (box != null) {
                                        float[] center = box.getCenter();
                                        float[] halfExtent = box.getHalfExtent();
                                        System.arraycopy(center, 0, modelCenter, 0, 3);
                                        System.arraycopy(halfExtent, 0, modelHalfExtent, 0, 3);
                                        hasBoundingBox = true;

                                        float maxDim = Math.max(halfExtent[0], Math.max(halfExtent[1], halfExtent[2])) * 2.0f;
                                        if (maxDim > 0.0f) {
                                            // 自动缩放模型，使其最大维度为 0.5 米（可根据具体模型调整）
                                            autoScaleFactor = 0.5f / maxDim;
                                        } else {
                                            autoScaleFactor = 1.0f;
                                        }
                                        Log.i(TAG, String.format("模型包围盒成功获取. 中心: [%.4f, %.4f, %.4f], 半长宽高: [%.4f, %.4f, %.4f], 自动缩放系数: %.6f",
                                                modelCenter[0], modelCenter[1], modelCenter[2],
                                                modelHalfExtent[0], modelHalfExtent[1], modelHalfExtent[2],
                                                autoScaleFactor));
                                    } else {
                                        Log.w(TAG, "模型包围盒 Box 为空，将使用默认缩放 1.0 且不进行中心化偏移");
                                    }
                                } catch (Exception e) {
                                    Log.e(TAG, "读取模型包围盒时出现异常，将使用默认缩放 1.0", e);
                                    hasBoundingBox = false;
                                    autoScaleFactor = 1.0f;
                                }

                                // 修改为同步载入贴图与内部材质数据，确保显示的第一帧材质就是完整的
                                Log.i(TAG, "开始同步载入贴图与内部材质数据...");
                                try {
                                    resourceLoader.loadResources(asset);
                                    Log.i(TAG, "贴图与内部材质数据同步载入完成");

                                    // 调试: 输出模型实体总数
                                    Log.i(TAG, "模型加载完成，实体数=" + asset.getEntities().length);
                                } catch (Exception e) {
                                    Log.e(TAG, "同步载入贴图与材质资源时发生异常", e);
                                }

                                // 调整材质参数
                                customizeMaterials();

                                // 将模型中包含的所有实体（网格、灯光等几何体）以及根节点全部添加到场景中
                                int[] entities = asset.getEntities();
                                for (int entity : entities) {
                                    scene.addEntity(entity);
                                }
                                scene.addEntity(asset.getRoot());

                                isInitialized = true;
                                // 初始化完成后启动帧循环（保持常开，丢失追踪时由 TransformManager 隐藏模型）
                                startFrameLoop();
                                Log.i(TAG, "[成功] Filament 模型加载已就绪");
                            } catch (Exception e) {
                                Log.e(TAG, "初始化加载 GLB 节点时触发异常", e);
                            }
                        }
                    });

                } catch (IOException e) {
                    Log.e(TAG, "读取 Assets 中 " + modelPath + " 错误", e);
                }
            }
        }).start();
    }

    // 调整模型所有材质球的属性参数
    private void customizeMaterials() {
        if (asset == null || engine == null) return;

        RenderableManager rm = engine.getRenderableManager();
        int[] entities = asset.getEntities();

        for (int entity : entities) {
            int instance = rm.getInstance(entity);
            if (instance == 0) {
                continue;
            }

            int primitiveCount = rm.getPrimitiveCount(instance);
            String nodeName = asset.getName(entity);
            if (nodeName == null) nodeName = "";
            Log.d(TAG, String.format("处理节点: %s (实体 ID: %d, Primitives: %d)", nodeName, entity, primitiveCount));

            for (int i = 0; i < primitiveCount; i++) {
                MaterialInstance materialInstance = rm.getMaterialInstanceAt(instance, i);
                if (materialInstance == null) continue;

                String matName = materialInstance.getName();
                if (matName == null) matName = "";
                Log.d(TAG, String.format("  Primitive %d - 材质: %s", i, matName));

                try { materialInstance.setParameter("roughnessFactor", 1.0f); } catch (Exception e) {}
                try { materialInstance.setParameter("metallicFactor", 0.0f); } catch (Exception e) {}
                try { materialInstance.setParameter("reflectance", 0.0f); } catch (Exception e) {}
                try { materialInstance.setParameter("normalScale", 0.0f); } catch (Exception e) {}
                try { materialInstance.setParameter("aoStrength", 0.0f); } catch (Exception e) {}
            }
        }
        Log.i(TAG, "材质优化完成");
    }

    private void render(long frameTimeNanos) {
        if (swapChain == null || engine == null) return;

        // 1. 异步贴图资源加载状态机推进
        if (isInitialized && asset != null) {
            resourceLoader.asyncUpdateLoad();
        }

        // 2. 将 SLAM 矩阵传给 Camera 与 Model Transform
        if (matricesReady) {
            // SLAM 视图矩阵为 world-to-camera，而 Filament 相机要求 camera-to-world (即视图矩阵的逆矩阵)
            if (android.opengl.Matrix.invertM(tempCameraModelMatrix, 0, viewMatrix, 0)) {
                camera.setModelMatrix(tempCameraModelMatrix);
            } else {
                camera.setModelMatrix(viewMatrix); // 逆矩阵失败则回退直接赋值
            }

            // 同步 Projection 投影矩阵 (需要转为 double 数组)
            for (int i = 0; i < 16; i++) {
                tempDoubleProj[i] = projectionMatrix[i];
            }
            // 这里的 near/far 裁剪面参数（0.1f 和 1000.0f）必须与 JNI 中 frustumM_RUB 里的参数严格一致
            camera.setCustomProjection(tempDoubleProj, 0.1, 1000.0);
        }

        // 3. 模型变换 / 显隐控制
        //    当 shouldDraw && matricesReady 时：正常定位到 AR 平面
        //    否则：缩放至 0 隐藏（帧循环常开，透明 clearColor 透出相机）
        if (asset != null) {
            int rootEntity = asset.getRoot();
            TransformManager tm = engine.getTransformManager();
            int instance = tm.getInstance(rootEntity);
            if (instance != 0) {
                boolean wantDraw = shouldDraw && matricesReady;
                if (wantDraw) {
                    float finalScale = initSize * currentScaleFactor * autoScaleFactor;

                    android.opengl.Matrix.setIdentityM(tempTransformMatrix, 0);

                    // S: 缩放（自动归一化 + 用户预设 + 手势缩放）
                    android.opengl.Matrix.scaleM(tempTransformMatrix, 0, finalScale, finalScale, finalScale);
                    // R: 绕 X 旋转 180° 修正坐标轴朝向
                    android.opengl.Matrix.rotateM(tempTransformMatrix, 0, 180.0f, 1.0f, 0.0f, 0.0f);
                    // R: 用户摇杆控制的旋转（yaw绕Y, pitch绕X）
                    if (Math.abs(userRotationY) > 0.01f) {
                        android.opengl.Matrix.rotateM(tempTransformMatrix, 0, userRotationY, 0.0f, 1.0f, 0.0f);
                    }
                    if (Math.abs(userRotationX) > 0.01f) {
                        android.opengl.Matrix.rotateM(tempTransformMatrix, 0, userRotationX, 1.0f, 0.0f, 0.0f);
                    }
                    // 无 T(center)：modelMatrix 已由 SLAM 定位到平面

                    android.opengl.Matrix.multiplyMM(tempScaledModelMatrix, 0, modelMatrix, 0, tempTransformMatrix, 0);

                    tm.setTransform(instance, tempScaledModelMatrix);
                } else {
                    // 隐藏：缩放为 0，透明背景透出相机画面
                    android.opengl.Matrix.setIdentityM(tempHiddenMatrix, 0);
                    android.opengl.Matrix.scaleM(tempHiddenMatrix, 0, 0.0f, 0.0f, 0.0f);
                    tm.setTransform(instance, tempHiddenMatrix);
                }
            }
        }

        // 4. 执行渲染绘制
        if (renderer.beginFrame(swapChain, frameTimeNanos)) {
            renderer.render(view);
            renderer.endFrame();
        }
    }

    private void startFrameLoop() {
        if (!isFrameCallbackActive) {
            isFrameCallbackActive = true;
            choreographer.postFrameCallback(frameCallback);
            Log.d(TAG, "开启 Filament 帧渲染循环");
        }
    }

    private void stopFrameLoop() {
        if (isFrameCallbackActive) {
            isFrameCallbackActive = false;
            choreographer.removeFrameCallback(frameCallback);
            Log.d(TAG, "暂停 Filament 帧渲染循环");
        }
    }

    @Override
    public void onUpdateModelMatrix(float[] M) {
        if (M != null && M.length == 16) {
            System.arraycopy(M, 0, modelMatrix, 0, 16);
            checkMatricesReady();
        }
    }

    @Override
    public void onUpdateViewMatrix(float[] M) {
        if (M != null && M.length == 16) {
            System.arraycopy(M, 0, viewMatrix, 0, 16);
            checkMatricesReady();
        }
    }

    @Override
    public void onUpdateProjectionMatrix(float[] M) {
        if (M != null && M.length == 16) {
            System.arraycopy(M, 0, projectionMatrix, 0, 16);
            checkMatricesReady();
        }
    }

    // 摇杆增量旋转，同时更新 Y 轴(yaw)与 X 轴(pitch)
    public void addUserRotation(float yawDelta, float pitchDelta) {
        userRotationY += yawDelta;
        userRotationY = userRotationY % 360.0f;
        if (userRotationY < 0) userRotationY += 360.0f;

        userRotationX += pitchDelta;
        userRotationX = userRotationX % 360.0f;
        if (userRotationX < 0) userRotationX += 360.0f;
    }

    @Override
    public void requestReset() {
        Log.d(TAG, "重置渲染状态");
        shouldDraw = false;
        matricesReady = false;
        currentScaleFactor = 1.0f;
        // 帧循环在 init 后常开，由 render() 中的 TransformManager 控制显隐
    }

    @Override
    public void setDraw(boolean flag) {
        boolean changed = (shouldDraw != flag);
        shouldDraw = flag;
        if (!flag) {
            matricesReady = false;
        } else {
            if (nativeHelper != null) {
                nativeHelper.nativeGetMVP(modelMatrix, viewMatrix, projectionMatrix,
                        GlobalConstant.RESOLUTION_WIDTH, GlobalConstant.RESOLUTION_HEIGHT);
                matricesReady = true;
            }
        }
        if (changed && drawStateListener != null) {
            drawStateListener.onDrawStateChanged(flag);
        }
    }

    private void checkMatricesReady() {
        matricesReady = true;
    }

    // 释放 Filament 占用的 native 与 Java 层资源，必须在 onDestroy() 中调用
    public void destroy() {
        Log.d(TAG, "开始释放 Filament 引擎资源...");
        stopFrameLoop();

        if (uiHelper != null) {
            uiHelper.detach();
        }

        if (engine != null) {
            if (swapChain != null) {
                engine.destroySwapChain(swapChain);
                swapChain = null;
            }

            for (int light : lightEntities) {
                engine.destroyEntity(light);
            }
            lightEntities.clear();

            if (indirectLight != null) {
                engine.destroyIndirectLight(indirectLight);
                indirectLight = null;
            }

            if (asset != null) {
                assetLoader.destroyAsset(asset);
                asset = null;
            }

            if (assetLoader != null) {
                assetLoader.destroy();
                assetLoader = null;
            }

            if (resourceLoader != null) {
                resourceLoader.destroy();
                resourceLoader = null;
            }

            if (materialProvider != null) {
                materialProvider.destroy();
                materialProvider = null;
            }

            if (view != null) {
                engine.destroyView(view);
                view = null;
            }

            if (scene != null) {
                engine.destroyScene(scene);
                scene = null;
            }

            if (camera != null) {
                engine.destroyCameraComponent(camera.getEntity());
                camera = null;
            }

            if (renderer != null) {
                engine.destroyRenderer(renderer);
                renderer = null;
            }

            engine.destroy();
            engine = null;
        }

        isInitialized = false;
        Log.i(TAG, "Filament 引擎资源已全部安全释放");
    }
}