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
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.PixelFormat;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.opengl.GLUtils;
import android.opengl.Matrix;
import android.util.Log;

import androidx.annotation.Keep;

import com.orb.slam2s.ipc.SlamIPCClient;
import com.orb.slam2s.util.TouchGestureHelper;

import java.io.ByteArrayOutputStream;
import java.io.InputStream;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

// 基于原生 C++ (cgltf + OpenGL ES) 的 AR 3D 模型 (GLB) 极速渲染器
// 彻底移除 Google Filament 依赖，实现 <200KB 极致原生渲染与零 GC 开销
public class GlbModelRenderer implements GLSurfaceView.Renderer {
    private static final String TAG = "GlbModelRenderer";

    static {
        System.loadLibrary("glb_renderer");
    }

    private AspectGLSurfaceView arObjectView;
    private Context context;
    private SlamIPCClient slamIPCClient;

    private String modelPath;
    private float initSize = 1.0f;

    private long nativeHandle = 0;
    private boolean isModelLoaded = false;
    private boolean shouldDraw = false;

    private final float[] modelMatrix = new float[16];
    private final float[] viewMatrix = new float[16];
    private final float[] projectionMatrix = new float[16];
    private final float[] tempMvp = new float[48];
    private boolean matricesReady = false;

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

    private GlbModelRenderer() {
        Matrix.setIdentityM(modelMatrix, 0);
        Matrix.setIdentityM(viewMatrix, 0);
        Matrix.setIdentityM(projectionMatrix, 0);
        nativeHandle = nativeCreate();
    }

    public static GlbModelRenderer newInstance() {
        return new GlbModelRenderer();
    }

    public GlbModelRenderer setArObjectView(AspectGLSurfaceView arObjectView) {
        this.arObjectView = arObjectView;
        return this;
    }

    public GlbModelRenderer setContext(Context context) {
        this.context = context;
        return this;
    }

    public GlbModelRenderer setSlamIPCClient(SlamIPCClient client) {
        this.slamIPCClient = client;
        return this;
    }

    public GlbModelRenderer setModelPath(String modelPath) {
        this.modelPath = modelPath;
        return this;
    }

    public GlbModelRenderer setInitSize(float initSize) {
        this.initSize = initSize;
        return this;
    }

    public GlbModelRenderer setDrawStateListener(DrawStateListener listener) {
        this.drawStateListener = listener;
        return this;
    }

    public GlbModelRenderer init(TouchGestureHelper touchHelper) {
        if (arObjectView == null) {
            Log.e(TAG, "ArObjectView 为空，无法初始化");
            return this;
        }

        arObjectView.setEGLContextClientVersion(2);
        arObjectView.setEGLConfigChooser(8, 8, 8, 8, 16, 0);
        arObjectView.getHolder().setFormat(PixelFormat.TRANSLUCENT);
        arObjectView.setZOrderOnTop(true);
        arObjectView.setRenderer(this);
        arObjectView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);

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

        loadModelAsync();
        return this;
    }

    private void loadModelAsync() {
        if (context == null || modelPath == null) return;

        new Thread(() -> {
            try (InputStream is = context.getAssets().open(modelPath);
                 ByteArrayOutputStream baos = new ByteArrayOutputStream()) {
                byte[] buffer = new byte[8192];
                int read;
                while ((read = is.read(buffer)) != -1) {
                    baos.write(buffer, 0, read);
                }
                final byte[] bytes = baos.toByteArray();

                boolean success = nativeLoadModel(nativeHandle, bytes, bytes.length);
                if (success) {
                    float[] bounds = new float[8];
                    nativeGetBounds(nativeHandle, bounds);
                    autoScaleFactor = bounds[7];
                    isModelLoaded = true;
                    Log.i(TAG, "GLB 模型异步解析完成: 自动缩放系数=" + autoScaleFactor);
                } else {
                    Log.e(TAG, "原生 cgltf 加载 GLB 失败: " + modelPath);
                }
            } catch (Exception e) {
                Log.e(TAG, "读取模型文件异常: " + modelPath, e);
            }
        }).start();
    }

    @Override
    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        Log.i(TAG, "onSurfaceCreated: 初始化原生 OpenGL ES 环境");
        if (nativeHandle != 0) {
            nativeInitGL(nativeHandle);
        }
    }

    @Override
    public void onSurfaceChanged(GL10 gl, int width, int height) {
        Log.i(TAG, "onSurfaceChanged: 视口宽度=" + width + ", 高度=" + height);
        if (nativeHandle != 0) {
            nativeResize(nativeHandle, width, height);
        }
    }

    @Override
    public void onDrawFrame(GL10 gl) {
        GLES20.glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);

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

        if (shouldDraw && matricesReady && isModelLoaded && nativeHandle != 0) {
            float finalScale = initSize * currentScaleFactor * autoScaleFactor;
            nativeRender(nativeHandle, modelMatrix, viewMatrix, projectionMatrix,
                    finalScale, userRotationX, userRotationY);
        }
    }

    @Keep
    public int onLoadTextureFromBytes(byte[] imageBytes) {
        if (imageBytes == null || imageBytes.length == 0) return 0;
        try {
            Bitmap bitmap = BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.length);
            if (bitmap == null) {
                Log.e(TAG, "BitmapFactory 解码纹理失败");
                return 0;
            }

            int[] textures = new int[1];
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
            Log.i(TAG, "成功在 GL 线程载入纹理 ID: " + textures[0]);
            return textures[0];
        } catch (Exception e) {
            Log.e(TAG, "生成纹理异常", e);
            return 0;
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

    public void onPause() {
        if (arObjectView != null) {
            arObjectView.onPause();
        }
    }

    public void onResume() {
        if (arObjectView != null) {
            arObjectView.onResume();
        }
    }

    public void destroy() {
        if (arObjectView != null && nativeHandle != 0) {
            final long handleToDestroy = nativeHandle;
            nativeHandle = 0;
            arObjectView.queueEvent(() -> nativeDestroy(handleToDestroy));
        }
        isModelLoaded = false;
    }

    // 原生接口声明
    private native long nativeCreate();
    private native boolean nativeLoadModel(long handle, byte[] data, int length);
    private native void nativeGetBounds(long handle, float[] outBounds);
    private native boolean nativeInitGL(long handle);
    private native void nativeResize(long handle, int width, int height);
    private native void nativeRender(long handle, float[] modelM, float[] viewM, float[] projM,
                                     float scale, float rotX, float rotY);
    private native void nativeDestroy(long handle);
}
