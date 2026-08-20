package com.orb.slam2s.graphics;

import android.opengl.GLES20;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

// 基于 OpenGL ES 2.0 的 3D SLAM 点云着色与渲染程序 (GL_POINTS)
// 每个点数据属性包含 7 个 float: [x, y, z, r, g, b, point_size]
public class GLPointCloudRenderer {
    private static final String VERTEX_SHADER =
            "uniform mat4 uVPMatrix;\n" +
            "attribute vec3 aPosition;\n" +
            "attribute vec3 aColor;\n" +
            "attribute float aPointSize;\n" +
            "varying vec4 vColor;\n" +
            "void main() {\n" +
            "    gl_Position = uVPMatrix * vec4(aPosition, 1.0);\n" +
            "    gl_PointSize = aPointSize;\n" +
            "    vColor = vec4(aColor, 1.0);\n" +
            "}\n";

    private static final String FRAGMENT_SHADER =
            "precision mediump float;\n" +
            "varying vec4 vColor;\n" +
            "void main() {\n" +
            "    gl_FragColor = vColor;\n" +
            "}\n";

    private int mProgram;
    private int mVPMatrixHandle;
    private int mPositionHandle;
    private int mColorHandle;
    private int mPointSizeHandle;

    private FloatBuffer mVertexBuffer;
    private int mPointCount = 0;

    public void init() {
        mProgram = GLUtils.createProgram(VERTEX_SHADER, FRAGMENT_SHADER);

        mVPMatrixHandle = GLES20.glGetUniformLocation(mProgram, "uVPMatrix");
        mPositionHandle = GLES20.glGetAttribLocation(mProgram, "aPosition");
        mColorHandle = GLES20.glGetAttribLocation(mProgram, "aColor");
        mPointSizeHandle = GLES20.glGetAttribLocation(mProgram, "aPointSize");
    }

    // 更新点云数据；pointData 为复用的大缓冲，count 为实际有效 float 数量
    public void updatePoints(float[] pointData, int count) {
        if (pointData == null || count <= 0) {
            mPointCount = 0;
            return;
        }
        mPointCount = count / 7;

        int requiredCapacity = count * 4;
        if (mVertexBuffer == null || mVertexBuffer.capacity() < requiredCapacity) {
            ByteBuffer bb = ByteBuffer.allocateDirect(requiredCapacity);
            bb.order(ByteOrder.nativeOrder());
            mVertexBuffer = bb.asFloatBuffer();
        }
        mVertexBuffer.clear();
        mVertexBuffer.put(pointData, 0, count);
        mVertexBuffer.position(0);
    }

    public void draw(float[] vpMatrix) {
        if (mProgram == 0 || mPointCount == 0 || mVertexBuffer == null) return;

        GLES20.glUseProgram(mProgram);
        GLES20.glUniformMatrix4fv(mVPMatrixHandle, 1, false, vpMatrix, 0);

        int stride = 7 * 4; // 每个点 7 个 float (28 字节)

        mVertexBuffer.position(0);
        GLES20.glEnableVertexAttribArray(mPositionHandle);
        GLES20.glVertexAttribPointer(mPositionHandle, 3, GLES20.GL_FLOAT, false, stride, mVertexBuffer);

        mVertexBuffer.position(3);
        GLES20.glEnableVertexAttribArray(mColorHandle);
        GLES20.glVertexAttribPointer(mColorHandle, 3, GLES20.GL_FLOAT, false, stride, mVertexBuffer);

        mVertexBuffer.position(6);
        GLES20.glEnableVertexAttribArray(mPointSizeHandle);
        GLES20.glVertexAttribPointer(mPointSizeHandle, 1, GLES20.GL_FLOAT, false, stride, mVertexBuffer);

        GLES20.glDrawArrays(GLES20.GL_POINTS, 0, mPointCount);

        GLES20.glDisableVertexAttribArray(mPositionHandle);
        GLES20.glDisableVertexAttribArray(mColorHandle);
        GLES20.glDisableVertexAttribArray(mPointSizeHandle);
    }

    public void destroy() {
        if (mProgram != 0) {
            GLES20.glDeleteProgram(mProgram);
            mProgram = 0;
        }
    }
}
