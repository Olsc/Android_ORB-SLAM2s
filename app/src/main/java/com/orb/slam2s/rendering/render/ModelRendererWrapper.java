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
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.os.Build;
import android.util.Log;

import com.orb.slam2s.constant.GlobalConstant;
import com.orb.slam2s.rendering.gles.GLRootView;
import com.orb.slam2s.slamar.NativeHelper;
import com.orb.slam2s.utils.TouchHelper;

import java.io.IOException;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

/**
 * 用于在AR环境中渲染3D模型（GLB格式）的包装类。
 * 遵循类似于ArObjectWrapper的构建器模式。
 */
public class ModelRendererWrapper implements GLSurfaceView.Renderer, NativeHelper.OnMVPUpdatedCallback {
    private static final String TAG = "ModelRendererWrapper";

    private GLRootView arObjectView;
    private Context context;
    private NativeHelper nativeHelper;
    private GlbRenderer glbRenderer;

    private String modelPath;
    private float initSize = 1.0f;

    private boolean isInitialized = false;
    private boolean shouldDraw = false;
    
    // 存储来自NativeHelper回调的矩阵
    private float[] modelMatrix = new float[16];
    private float[] viewMatrix = new float[16]; 
    private float[] projectionMatrix = new float[16];
    private boolean matricesReady = false;
    
    // 双指缩放相关
    private float currentScaleFactor = 1.0f;  // 当前累积的缩放因子
    private static final float MIN_SCALE = 0.1f;  // 最小缩放比例
    private static final float MAX_SCALE = 5.0f;  // 最大缩放比例

    private ModelRendererWrapper() {
        glbRenderer = new GlbRenderer();
        // 用单位矩阵初始化矩阵
        android.opengl.Matrix.setIdentityM(modelMatrix, 0);
        android.opengl.Matrix.setIdentityM(viewMatrix, 0);
        android.opengl.Matrix.setIdentityM(projectionMatrix, 0);
    }

    public static ModelRendererWrapper newInstance() {
        return new ModelRendererWrapper();
    }

    public ModelRendererWrapper setArObjectView(GLRootView arObjectView) {
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

    public ModelRendererWrapper init(TouchHelper touchHelper) {
        if (arObjectView == null) {
            Log.e(TAG, "ArObjectView为空，无法初始化");
            return this;
        }

        arObjectView.setEGLContextClientVersion(2);
        arObjectView.setEGLConfigChooser(8, 8, 8, 8, 16, 0);
        arObjectView.getHolder().setFormat(PixelFormat.TRANSLUCENT);
        arObjectView.setZOrderOnTop(true);

        arObjectView.setRenderer(this);
        arObjectView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB) {
            arObjectView.setPreserveEGLContextOnPause(true);
        }

        // 如果提供了touchHelper，添加缩放回调
        if (touchHelper != null) {
            touchHelper.addScalingCallback(new TouchHelper.ScalingCallback() {
                @Override
                public void updateScale(float scaleFactor) {
                    // 仅在有AR物体正在渲染时才允许缩放
                    if (shouldDraw && nativeHelper != null && 
                        nativeHelper.getLastTrackingResult() == GlobalConstant.SLAM_ON) {
                        // 累积缩放因子（scaleFactor是增量值，不是绝对值）
                        currentScaleFactor *= scaleFactor;
                        
                        // 限制缩放范围，防止物体过大或过小
                        if (currentScaleFactor < MIN_SCALE) {
                            currentScaleFactor = MIN_SCALE;
                        } else if (currentScaleFactor > MAX_SCALE) {
                            currentScaleFactor = MAX_SCALE;
                        }
                        
                        Log.d(TAG, String.format("缩放更新: 因子=%.2f, 总缩放=%.2f", 
                            scaleFactor, currentScaleFactor));
                    }
                }
            });
        }

        return this;
    }

    @Override
    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        Log.d(TAG, "表面已创建");
        GLES20.glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        if (context != null && modelPath != null) {
            try {
                Log.i(TAG, String.format("开始初始化GlbRenderer - 模型: %s", modelPath));
                glbRenderer.createOnGlThread(context, modelPath);
                isInitialized = true;
                Log.i(TAG, "[成功] GlbRenderer初始化成功");

                // 显示自动缩放信息
                float autoScale = glbRenderer.getAutoScaleFactor();
                if (autoScale != 1.0f) {
                    Log.i(TAG, String.format("[信息] 自动缩放已应用: %.3fx", autoScale));
                }

            } catch (IOException e) {
                Log.e(TAG, "[错误] 初始化GlbRenderer失败", e);
                Log.e(TAG, String.format("检查文件是否存在: model='%s'", modelPath));
                isInitialized = false;
                
                // 尝试给出更多调试信息
                try {
                    context.getAssets().open(modelPath).close();
                    Log.d(TAG, "[成功] GLB文件存在: " + modelPath);
                } catch (IOException me) {
                    Log.e(TAG, "[错误] GLB文件不存在: " + modelPath);
                }
            } catch (Exception e) {
                Log.e(TAG, "[错误] 初始化GlbRenderer遇到意外错误", e);
                isInitialized = false;
            }
        } else {
            Log.e(TAG, String.format("[错误] 无法初始化GlbRenderer - context: %s, modelPath: %s",
                    context != null ? "正常" : "为空",
                    modelPath != null ? modelPath : "null"));
        }
    }

    @Override
    public void onSurfaceChanged(GL10 gl, int width, int height) {
        Log.d(TAG, "表面已改变: " + width + "x" + height);
        GLES20.glViewport(0, 0, width, height);
    }

    @Override
    public void onDrawFrame(GL10 gl) {
        // 清除屏幕
        GLES20.glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        GLES20.glClear(GLES20.GL_DEPTH_BUFFER_BIT | GLES20.GL_COLOR_BUFFER_BIT);

        // 仅在以下情况下渲染：
        // 1. GlbRenderer已初始化
        // 2. NativeHelper指示应该绘制（SLAM_ON && planeDetected）
        // 3. 我们从native回调获得了有效的矩阵
        if (!isInitialized || !shouldDraw || !matricesReady) {
            return;
        }

        // 应用变换：旋转 + 缩放
        float[] transformMatrix = new float[16];
        float[] rotationMatrix = new float[16];
        float[] scaledModelMatrix = new float[16];
        
        // 创建旋转矩阵：绕X轴旋转180度以将模型翻转为直立
        android.opengl.Matrix.setIdentityM(rotationMatrix, 0);
        android.opengl.Matrix.rotateM(rotationMatrix, 0, 180.0f, 1.0f, 0.0f, 0.0f);
        
        // 创建缩放矩阵（结合初始大小和用户缩放）
        float finalScale = initSize * currentScaleFactor;
        android.opengl.Matrix.setIdentityM(transformMatrix, 0);
        android.opengl.Matrix.scaleM(transformMatrix, 0, finalScale, finalScale, finalScale);
        
        // 组合：首先应用旋转，然后缩放
        android.opengl.Matrix.multiplyMM(transformMatrix, 0, transformMatrix, 0, rotationMatrix, 0);
        
        // 应用到来自NativeHelper的模型矩阵
        android.opengl.Matrix.multiplyMM(scaledModelMatrix, 0, modelMatrix, 0, transformMatrix, 0);

        // 使用来自NativeHelper的矩阵更新并绘制对象
        glbRenderer.updateModelMatrix(scaledModelMatrix, 1.0f);
        glbRenderer.draw(viewMatrix, projectionMatrix, 1.0f);
    }

    @Override
    public void onUpdateModelMatrix(float[] M) {
        // 当模型矩阵更新时，NativeHelper会调用此方法
        if (M != null && M.length == 16) {
            System.arraycopy(M, 0, modelMatrix, 0, 16);
            // Log.d(TAG, "模型矩阵已更新");  // 高频日志已注释
            checkMatricesReady();
        }
    }

    @Override
    public void onUpdateViewMatrix(float[] M) {
        // 当视图矩阵更新时，NativeHelper会调用此方法
        if (M != null && M.length == 16) {
            System.arraycopy(M, 0, viewMatrix, 0, 16);
            // Log.d(TAG, "视图矩阵已更新");  // 高频日志已注释
            checkMatricesReady();
        }
    }

    @Override
    public void onUpdateProjectionMatrix(float[] M) {
        // 当投影矩阵更新时，NativeHelper会调用此方法
        if (M != null && M.length == 16) {
            System.arraycopy(M, 0, projectionMatrix, 0, 16);
            // Log.d(TAG, "投影矩阵已更新");  // 高频日志已注释
            checkMatricesReady();
        }
    }

    @Override
    public void requestReset() {
        Log.d(TAG, "请求重置被调用 - 重置渲染状态");
        matricesReady = false;
        shouldDraw = false;
        currentScaleFactor = 1.0f;  // 重置缩放因子
    }

    @Override
    public void setDraw(boolean flag) {
        //Log.d(TAG, "setDraw: " + flag);
        shouldDraw = flag;
        if (!flag) {
            // 如果应该停止绘制，也重置矩阵就绪状态
            matricesReady = false;
        } else {
            //  当设置为绘制时，从NativeHelper获取最新矩阵
            // 这样可以支持从地图加载恢复的平面（矩阵已在C++端就绪）
            if (nativeHelper != null) {
                nativeHelper.getM(modelMatrix);
                nativeHelper.getV(viewMatrix);
                nativeHelper.getP(GlobalConstant.RESOLUTION_WIDTH, GlobalConstant.RESOLUTION_HEIGHT, projectionMatrix);
                matricesReady = true;
                //Log.d(TAG, "setDraw: 已从Native获取矩阵，matricesReady=true");
            }
        }
    }

    /**
     * 检查NativeHelper是否已设置所有必需的矩阵
     */
    private void checkMatricesReady() {
        // 如果我们至少收到了模型和视图矩阵，则认为矩阵已就绪
        // 投影矩阵在平面检测期间设置一次
        matricesReady = true;
        // Log.d(TAG, "矩阵已准备好进行渲染");  // 高频日志已注释
    }
}
