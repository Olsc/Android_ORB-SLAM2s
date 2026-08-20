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

import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.opengl.Matrix;
import android.util.Log;

import com.orb.slam2s.sensors.OrientationSensor;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

// 3DOF 立方体渲染器：在视角前方指定距离处生成彩色立方体并进行 3DOF 空间跟踪
public class ThreeDofCubeRenderer implements GLSurfaceView.Renderer {
    private static final String TAG = "ThreeDofCubeRenderer";

    private static final float AR_OBJECT_SPIN_Y_DEG = 45.0f;
    private static final float AR_OBJECT_TILT_X_DEG = 30.0f;
    private static final float AR_3DOF_ZNEAR = 1.0f;
    private static final float AR_3DOF_ZFAR = 100.0f;

    private boolean mInitialized = false;
    private float[] mObjectWorldPos = new float[3];

    private final OrientationSensor orientationSensor;
    private int program;

    private final float[] mvpMatrix = new float[16];
    private float mRatio = 1.0f;
    private float mDistance = 5.0f; // 默认 5 米

    private FloatBuffer vertexBuffer;
    private FloatBuffer colorBuffer;
    private ShortBuffer indexBuffer;

    private static final int CUBE_INDEX_COUNT = 36;

    // 控制是否显示立方体
    private boolean mShowCube = false;

    public ThreeDofCubeRenderer(OrientationSensor sensor) {
        this.orientationSensor = sensor;
    }

    // 在视角前方指定距离处生成立方体
    public void spawnCubeAtDistance(float distance) {
        this.mDistance = distance;
        this.mInitialized = false; // 重置，下一帧会重新计算位置
        this.mShowCube = true;
        Log.d(TAG, "请求在前方 " + distance + " 米处生成立方体");
    }

    // 隐藏立方体
    public void hideCube() {
        this.mShowCube = false;
    }

    @Override
    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        GLES20.glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        GLES20.glEnable(GLES20.GL_DEPTH_TEST);
        GLES20.glDepthFunc(GLES20.GL_LEQUAL);
        GLES20.glEnable(GLES20.GL_BLEND);
        GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA);

        String vertexShaderCode =
            "uniform mat4 uMVPMatrix;" +
            "attribute vec4 vPosition;" +
            "attribute vec4 vColor;" +
            "varying vec4 fColor;" +
            "void main() {" +
            "  gl_Position = uMVPMatrix * vPosition;" +
            "  fColor = vColor;" +
            "}";

        String fragmentShaderCode =
            "precision mediump float;" +
            "varying vec4 fColor;" +
            "void main() {" +
            "  gl_FragColor = fColor;" +
            "}";

        program = GLUtils.createProgram(vertexShaderCode, fragmentShaderCode);

        createCube();
    }

    @Override
    public void onSurfaceChanged(GL10 gl, int width, int height) {
        GLES20.glViewport(0, 0, width, height);
        mRatio = (float) width / height;
    }

    @Override
    public void onDrawFrame(GL10 gl) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);

        if (!mShowCube || !orientationSensor.isReady()) {
            return;
        }

        float[] rotationMatrix = orientationSensor.getRotationMatrix();

        boolean isIdentity = true;
        for (int i = 1; i < 4; i++) {
            if (rotationMatrix[i] != 0) isIdentity = false;
        }

        if (!mInitialized && !isIdentity) {
            mObjectWorldPos = calculate3DofInsertionPoint(rotationMatrix, mDistance);
            mInitialized = true;
            Log.d(TAG, String.format("立方体世界坐标已初始化: [%.2f, %.2f, %.2f]",
                    mObjectWorldPos[0], mObjectWorldPos[1], mObjectWorldPos[2]));
        }

        if (mInitialized) {
            compute3DofMVP(mvpMatrix, rotationMatrix, mRatio, mObjectWorldPos);
            drawCube();
        }
    }

    // 纯 Java 计算 3DOF 物体世界坐标插入点（视角前方指定距离）
    private float[] calculate3DofInsertionPoint(float[] rotationMatrix, float distance) {
        float[] invViewMatrix = new float[16];
        Matrix.transposeM(invViewMatrix, 0, rotationMatrix, 0);

        float[] targetPosCameraSpace = {0.0f, 0.0f, -distance, 1.0f};
        float[] worldPos = new float[4];
        Matrix.multiplyMV(worldPos, 0, invViewMatrix, 0, targetPosCameraSpace, 0);
        return new float[]{worldPos[0], worldPos[1], worldPos[2]};
    }

    // 纯 Java 计算 3DOF MVP 矩阵
    private void compute3DofMVP(float[] outMvp, float[] rotationMatrix, float ratio, float[] objectPos) {
        float[] projectionMatrix = new float[16];
        Matrix.frustumM(projectionMatrix, 0, -ratio, ratio, -1, 1, AR_3DOF_ZNEAR, AR_3DOF_ZFAR);

        float[] modelMatrix = new float[16];
        Matrix.setIdentityM(modelMatrix, 0);
        float ox = objectPos != null && objectPos.length > 0 ? objectPos[0] : 0.0f;
        float oy = objectPos != null && objectPos.length > 1 ? objectPos[1] : 0.0f;
        float oz = objectPos != null && objectPos.length > 2 ? objectPos[2] : 0.0f;
        Matrix.translateM(modelMatrix, 0, ox, oy, oz);
        Matrix.rotateM(modelMatrix, 0, AR_OBJECT_SPIN_Y_DEG, 0.0f, 1.0f, 0.0f);
        Matrix.rotateM(modelMatrix, 0, AR_OBJECT_TILT_X_DEG, 1.0f, 0.0f, 0.0f);

        float[] tempMatrix = new float[16];
        Matrix.multiplyMM(tempMatrix, 0, rotationMatrix, 0, modelMatrix, 0);
        Matrix.multiplyMM(outMvp, 0, projectionMatrix, 0, tempMatrix, 0);
    }

    private void createCube() {
        float size = 0.5f;

        float[] vertices = {
            -size, -size, size,
            size, -size, size,
            size, size, size,
            -size, size, size,
            -size, -size, -size,
            size, -size, -size,
            size, size, -size,
            -size, size, -size
        };

        short[] indices = {
            0, 1, 2, 0, 2, 3,
            1, 5, 6, 1, 6, 2,
            5, 4, 7, 5, 7, 6,
            4, 0, 3, 4, 3, 7,
            3, 2, 6, 3, 6, 7,
            4, 5, 1, 4, 1, 0
        };

        float[][] faceColors = {
            {1.0f, 0.0f, 0.0f, 1.0f}, // 红
            {0.0f, 1.0f, 0.0f, 1.0f}, // 绿
            {0.0f, 0.0f, 1.0f, 1.0f}, // 蓝
            {1.0f, 1.0f, 0.0f, 1.0f}, // 黄
            {0.0f, 1.0f, 1.0f, 1.0f}, // 青
            {1.0f, 0.0f, 1.0f, 1.0f}  // 品红
        };

        float[] expandedColors = new float[indices.length * 4];
        int colorIdx = 0;
        for (float[] color : faceColors) {
            for (int i = 0; i < 6; i++) {
                expandedColors[colorIdx++] = color[0];
                expandedColors[colorIdx++] = color[1];
                expandedColors[colorIdx++] = color[2];
                expandedColors[colorIdx++] = color[3];
            }
        }

        ByteBuffer vbb = ByteBuffer.allocateDirect(vertices.length * 4);
        vbb.order(ByteOrder.nativeOrder());
        vertexBuffer = vbb.asFloatBuffer();
        vertexBuffer.put(vertices);
        vertexBuffer.position(0);

        ByteBuffer cbb = ByteBuffer.allocateDirect(expandedColors.length * 4);
        cbb.order(ByteOrder.nativeOrder());
        colorBuffer = cbb.asFloatBuffer();
        colorBuffer.put(expandedColors);
        colorBuffer.position(0);

        ByteBuffer ibb = ByteBuffer.allocateDirect(indices.length * 2);
        ibb.order(ByteOrder.nativeOrder());
        indexBuffer = ibb.asShortBuffer();
        indexBuffer.put(indices);
        indexBuffer.position(0);
    }

    private int positionHandle = -1;
    private int colorHandle = -1;
    private int mvpMatrixHandle = -1;

    private void drawCube() {
        GLES20.glUseProgram(program);

        if (positionHandle < 0) {
            positionHandle = GLES20.glGetAttribLocation(program, "vPosition");
            colorHandle = GLES20.glGetAttribLocation(program, "vColor");
            mvpMatrixHandle = GLES20.glGetUniformLocation(program, "uMVPMatrix");
        }

        GLES20.glEnableVertexAttribArray(positionHandle);
        GLES20.glEnableVertexAttribArray(colorHandle);

        GLES20.glVertexAttribPointer(positionHandle, 3, GLES20.GL_FLOAT, false, 12, vertexBuffer);
        GLES20.glVertexAttribPointer(colorHandle, 4, GLES20.GL_FLOAT, false, 16, colorBuffer);

        GLES20.glUniformMatrix4fv(mvpMatrixHandle, 1, false, mvpMatrix, 0);

        GLES20.glDrawElements(GLES20.GL_TRIANGLES, CUBE_INDEX_COUNT, GLES20.GL_UNSIGNED_SHORT, indexBuffer);

        GLES20.glDisableVertexAttribArray(positionHandle);
        GLES20.glDisableVertexAttribArray(colorHandle);
    }
}
