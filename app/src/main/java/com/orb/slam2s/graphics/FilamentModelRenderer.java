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
package com.orb.slam2s.graphics;

import android.content.Context;
import android.graphics.PixelFormat;
import android.opengl.Matrix;
import android.util.Log;
import android.view.Choreographer;
import android.view.Surface;

import com.google.android.filament.Box;
import com.google.android.filament.Camera;
import com.google.android.filament.Engine;
import com.google.android.filament.EntityManager;
import com.google.android.filament.IndirectLight;
import com.google.android.filament.LightManager;
import com.google.android.filament.MaterialInstance;
import com.google.android.filament.RenderableManager;
import com.google.android.filament.Renderer;
import com.google.android.filament.Scene;
import com.google.android.filament.SwapChain;
import com.google.android.filament.TransformManager;
import com.google.android.filament.View;
import com.google.android.filament.Viewport;
import com.google.android.filament.android.DisplayHelper;
import com.google.android.filament.android.UiHelper;
import com.google.android.filament.gltfio.AssetLoader;
import com.google.android.filament.gltfio.FilamentAsset;
import com.google.android.filament.gltfio.MaterialProvider;
import com.google.android.filament.gltfio.ResourceLoader;
import com.google.android.filament.gltfio.UbershaderProvider;
import com.google.android.filament.utils.Utils;
import com.orb.slam2s.graphics.AspectSurfaceView;
import com.orb.slam2s.ipc.SlamIPCClient;
import com.orb.slam2s.util.TouchGestureHelper;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

// 基于 Google Filament 引擎的 AR 3D 模型 (GLB) 渲染包装器
// 采用共享内存直接读取 MVP 与渲染状态标志，免去 Binder 调用与频繁 GC
public class FilamentModelRenderer {
    private static final String TAG = "FilamentModelRenderer";

    static {
        Utils.init();
    }

    private AspectSurfaceView arObjectView;
    private Context context;
    private SlamIPCClient slamIPCClient;

    private String modelPath;
    private float initSize = 1.0f;

    private boolean isInitialized = false;
    private boolean shouldDraw = false;

    private final float[] modelMatrix = new float[16];
    private final float[] viewMatrix = new float[16];
    private final float[] projectionMatrix = new float[16];
    private final float[] tempMvp = new float[48];
    private boolean matricesReady = false;

    private final float[] modelCenter = new float[3];
    private final float[] modelHalfExtent = new float[3];
    private float autoScaleFactor = 1.0f;

    private float currentScaleFactor = 1.0f;
    private static final float MIN_SCALE = 0.05f;
    private static final float MAX_SCALE = 10.0f;

    private float userRotationY = 0.0f;
    private float userRotationX = 0.0f;

    public interface DrawStateListener {
        void onDrawStateChanged(boolean shouldDraw);
    }
    private DrawStateListener drawStateListener;

    private final float[] tempCameraModelMatrix = new float[16];
    private final double[] tempDoubleProj = new double[16];
    private final float[] tempTransformMatrix = new float[16];
    private final float[] tempScaledModelMatrix = new float[16];
    private final float[] tempHiddenMatrix = new float[16];

    private Engine engine;
    private Renderer renderer;
    private Scene scene;
    private View view;
    private Camera camera;
    private SwapChain swapChain;
    private UiHelper uiHelper;
    private DisplayHelper displayHelper;

    private MaterialProvider materialProvider;
    private AssetLoader assetLoader;
    private ResourceLoader resourceLoader;
    private FilamentAsset asset;
    private IndirectLight indirectLight;

    private final List<Integer> lightEntities = new ArrayList<>();

    private Choreographer choreographer;
    private boolean isFrameCallbackActive = false;

    private final Choreographer.FrameCallback frameCallback = new Choreographer.FrameCallback() {
        @Override
        public void doFrame(long frameTimeNanos) {
            if (isFrameCallbackActive) {
                choreographer.postFrameCallback(this);
            }
            render(frameTimeNanos);
        }
    };

    private FilamentModelRenderer() {
        Matrix.setIdentityM(modelMatrix, 0);
        Matrix.setIdentityM(viewMatrix, 0);
        Matrix.setIdentityM(projectionMatrix, 0);
    }

    public static FilamentModelRenderer newInstance() {
        return new FilamentModelRenderer();
    }

    public FilamentModelRenderer setArObjectView(AspectSurfaceView arObjectView) {
        this.arObjectView = arObjectView;
        return this;
    }

    public FilamentModelRenderer setContext(Context context) {
        this.context = context;
        return this;
    }

    public FilamentModelRenderer setSlamIPCClient(SlamIPCClient client) {
        this.slamIPCClient = client;
        return this;
    }

    public FilamentModelRenderer setModelPath(String modelPath) {
        this.modelPath = modelPath;
        return this;
    }

    public FilamentModelRenderer setInitSize(float initSize) {
        this.initSize = initSize;
        return this;
    }

    public FilamentModelRenderer setDrawStateListener(DrawStateListener listener) {
        this.drawStateListener = listener;
        return this;
    }

    public FilamentModelRenderer init(TouchGestureHelper touchHelper) {
        if (arObjectView == null) {
            Log.e(TAG, "ArObjectView为空，无法初始化");
            return this;
        }

        arObjectView.getHolder().setFormat(PixelFormat.TRANSLUCENT);
        arObjectView.setZOrderOnTop(true);

        if (touchHelper != null) {
            touchHelper.addScalingCallback(scaleFactor -> {
                if (shouldDraw) {
                    currentScaleFactor *= scaleFactor;
                    if (currentScaleFactor < MIN_SCALE) {
                        currentScaleFactor = MIN_SCALE;
                    } else if (currentScaleFactor > MAX_SCALE) {
                        currentScaleFactor = MAX_SCALE;
                    }
                    if (slamIPCClient != null) {
                        slamIPCClient.updateArObjectScale(scaleFactor);
                    }
                }
            });
        }

        choreographer = Choreographer.getInstance();
        displayHelper = new DisplayHelper(context);

        setupFilament();

        return this;
    }

    private void setupFilament() {
        engine = Engine.create();
        renderer = engine.createRenderer();
        scene = engine.createScene();

        int cameraEntity = EntityManager.get().create();
        camera = engine.createCamera(cameraEntity);
        camera.setExposure(16.0f, 1.0f / 125.0f, 100.0f);

        view = engine.createView();
        view.setScene(scene);
        view.setCamera(camera);

        view.setBlendMode(View.BlendMode.TRANSLUCENT);

        view.setRenderQuality(new View.RenderQuality());
        View.RenderQuality quality = view.getRenderQuality();
        quality.hdrColorBuffer = View.QualityLevel.LOW;
        view.setRenderQuality(quality);

        view.setPostProcessingEnabled(false);
        view.setSampleCount(1);

        View.DynamicResolutionOptions options = new View.DynamicResolutionOptions();
        options.enabled = true;
        view.setDynamicResolutionOptions(options);

        Renderer.ClearOptions clearOptions = renderer.getClearOptions();
        clearOptions.clearColor = new double[]{0.0, 0.0, 0.0, 0.0};
        clearOptions.clear = true;
        renderer.setClearOptions(clearOptions);

        materialProvider = new UbershaderProvider(engine);
        assetLoader = new AssetLoader(engine, materialProvider, EntityManager.get());
        resourceLoader = new ResourceLoader(engine, false);

        createLights();

        uiHelper = new UiHelper(UiHelper.ContextErrorPolicy.DONT_CHECK);
        uiHelper.setOpaque(false);
        uiHelper.setRenderCallback(new UiHelper.RendererCallback() {
            @Override
            public void onNativeWindowChanged(Surface surface) {
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
                if (swapChain != null) {
                    engine.destroySwapChain(swapChain);
                    swapChain = null;
                }
                displayHelper.detach();
            }

            @Override
            public void onResized(int width, int height) {
                view.setViewport(new Viewport(0, 0, width, height));
            }
        });
        uiHelper.attachTo(arObjectView);
    }

    private void createLights() {
        int mainLight = EntityManager.get().create();
        new LightManager.Builder(LightManager.Type.DIRECTIONAL)
                .color(1.0f, 1.0f, 1.0f)
                .intensity(100000.0f)
                .direction(-0.5f, -1.0f, -0.5f)
                .castShadows(false)
                .build(engine, mainLight);
        scene.addEntity(mainLight);
        lightEntities.add(mainLight);

        int fillLight = EntityManager.get().create();
        new LightManager.Builder(LightManager.Type.DIRECTIONAL)
                .color(1.0f, 1.0f, 1.0f)
                .intensity(50000.0f)
                .direction(0.5f, 1.0f, 0.5f)
                .castShadows(false)
                .build(engine, fillLight);
        scene.addEntity(fillLight);
        lightEntities.add(fillLight);

        float[] sh = new float[27];
        sh[0] = 1.0f; sh[1] = 1.0f; sh[2] = 1.0f;
        for (int i = 3; i < 27; i++) sh[i] = 0.0f;

        try {
            indirectLight = new IndirectLight.Builder()
                    .irradiance(3, sh)
                    .intensity(30000.0f)
                    .build(engine);
            scene.setIndirectLight(indirectLight);
        } catch (Exception e) {
            Log.e(TAG, "创建环境光时失败", e);
        }
    }

    private void loadModelAsync() {
        if (context == null || modelPath == null) return;

        new Thread(() -> {
            try (InputStream is = context.getAssets().open(modelPath)) {
                byte[] bytes = new byte[is.available()];
                is.read(bytes);

                final ByteBuffer buffer = ByteBuffer.allocateDirect(bytes.length);
                buffer.put(bytes);
                buffer.flip();

                arObjectView.post(() -> {
                    try {
                        asset = assetLoader.createAsset(buffer);
                        if (asset == null) {
                            Log.e(TAG, "AssetLoader 创建模型失败");
                            return;
                        }

                        try {
                            Box box = asset.getBoundingBox();
                            if (box != null) {
                                float[] center = box.getCenter();
                                float[] halfExtent = box.getHalfExtent();
                                System.arraycopy(center, 0, modelCenter, 0, 3);
                                System.arraycopy(halfExtent, 0, modelHalfExtent, 0, 3);

                                float maxDim = Math.max(halfExtent[0], Math.max(halfExtent[1], halfExtent[2])) * 2.0f;
                                if (maxDim > 0.0f) {
                                    autoScaleFactor = 0.5f / maxDim;
                                } else {
                                    autoScaleFactor = 1.0f;
                                }
                            } else {
                                autoScaleFactor = 1.0f;
                            }
                        } catch (Exception e) {
                            autoScaleFactor = 1.0f;
                        }

                        try {
                            resourceLoader.loadResources(asset);
                        } catch (Exception e) {
                            Log.e(TAG, "同步载入贴图与材质资源时发生异常", e);
                        }

                        customizeMaterials();

                        int[] entities = asset.getEntities();
                        for (int entity : entities) {
                            scene.addEntity(entity);
                        }
                        scene.addEntity(asset.getRoot());

                        isInitialized = true;
                        startFrameLoop();
                    } catch (Exception e) {
                        Log.e(TAG, "初始化加载 GLB 节点时触发异常", e);
                    }
                });

            } catch (IOException e) {
                Log.e(TAG, "读取 Assets 中 " + modelPath + " 错误", e);
            }
        }).start();
    }

    private void customizeMaterials() {
        if (asset == null || engine == null) return;

        RenderableManager rm = engine.getRenderableManager();
        int[] entities = asset.getEntities();

        for (int entity : entities) {
            int instance = rm.getInstance(entity);
            if (instance == 0) continue;

            int primitiveCount = rm.getPrimitiveCount(instance);
            for (int i = 0; i < primitiveCount; i++) {
                MaterialInstance materialInstance = rm.getMaterialInstanceAt(instance, i);
                if (materialInstance == null) continue;

                try { materialInstance.setParameter("roughnessFactor", 1.0f); } catch (Exception ignored) {}
                try { materialInstance.setParameter("metallicFactor", 0.0f); } catch (Exception ignored) {}
                try { materialInstance.setParameter("reflectance", 0.0f); } catch (Exception ignored) {}
                try { materialInstance.setParameter("normalScale", 0.0f); } catch (Exception ignored) {}
                try { materialInstance.setParameter("aoStrength", 0.0f); } catch (Exception ignored) {}
            }
        }
    }

    private void render(long frameTimeNanos) {
        if (swapChain == null || engine == null) return;

        if (isInitialized && asset != null) {
            resourceLoader.asyncUpdateLoad();
        }

        if (slamIPCClient != null && slamIPCClient.isConnected()) {
            boolean drawFlag = slamIPCClient.readDrawFlag();
            if (drawFlag != shouldDraw) {
                shouldDraw = drawFlag;
                if (drawStateListener != null) {
                    drawStateListener.onDrawStateChanged(shouldDraw);
                }
            }
            if (shouldDraw && slamIPCClient.readMvp(tempMvp)) {
                System.arraycopy(tempMvp, 0, modelMatrix, 0, 16);
                System.arraycopy(tempMvp, 16, viewMatrix, 0, 16);
                System.arraycopy(tempMvp, 32, projectionMatrix, 0, 16);
                matricesReady = true;
            } else {
                matricesReady = false;
            }
        }

        if (matricesReady) {
            if (Matrix.invertM(tempCameraModelMatrix, 0, viewMatrix, 0)) {
                camera.setModelMatrix(tempCameraModelMatrix);
            } else {
                camera.setModelMatrix(viewMatrix);
            }

            for (int i = 0; i < 16; i++) {
                tempDoubleProj[i] = projectionMatrix[i];
            }
            camera.setCustomProjection(tempDoubleProj, 0.1, 1000.0);
        }

        if (asset != null) {
            int rootEntity = asset.getRoot();
            TransformManager tm = engine.getTransformManager();
            int instance = tm.getInstance(rootEntity);
            if (instance != 0) {
                boolean wantDraw = shouldDraw && matricesReady;
                if (wantDraw) {
                    float finalScale = initSize * currentScaleFactor * autoScaleFactor;

                    Matrix.setIdentityM(tempTransformMatrix, 0);
                    Matrix.scaleM(tempTransformMatrix, 0, finalScale, finalScale, finalScale);
                    Matrix.rotateM(tempTransformMatrix, 0, 180.0f, 1.0f, 0.0f, 0.0f);
                    if (Math.abs(userRotationY) > 0.01f) {
                        Matrix.rotateM(tempTransformMatrix, 0, userRotationY, 0.0f, 1.0f, 0.0f);
                    }
                    if (Math.abs(userRotationX) > 0.01f) {
                        Matrix.rotateM(tempTransformMatrix, 0, userRotationX, 1.0f, 0.0f, 0.0f);
                    }

                    Matrix.multiplyMM(tempScaledModelMatrix, 0, modelMatrix, 0, tempTransformMatrix, 0);
                    tm.setTransform(instance, tempScaledModelMatrix);
                } else {
                    Matrix.setIdentityM(tempHiddenMatrix, 0);
                    Matrix.scaleM(tempHiddenMatrix, 0, 0.0f, 0.0f, 0.0f);
                    tm.setTransform(instance, tempHiddenMatrix);
                }
            }
        }

        if (renderer.beginFrame(swapChain, frameTimeNanos)) {
            renderer.render(view);
            renderer.endFrame();
        }
    }

    private void startFrameLoop() {
        if (!isFrameCallbackActive) {
            isFrameCallbackActive = true;
            choreographer.postFrameCallback(frameCallback);
        }
    }

    private void stopFrameLoop() {
        if (isFrameCallbackActive) {
            isFrameCallbackActive = false;
            choreographer.removeFrameCallback(frameCallback);
        }
    }

    public void addUserRotation(float yawDelta, float pitchDelta) {
        userRotationY += yawDelta;
        userRotationY = userRotationY % 360.0f;
        if (userRotationY < 0) userRotationY += 360.0f;

        userRotationX += pitchDelta;
        userRotationX = userRotationX % 360.0f;
        if (userRotationX < 0) userRotationX += 360.0f;
    }

    public void destroy() {
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
    }
}
