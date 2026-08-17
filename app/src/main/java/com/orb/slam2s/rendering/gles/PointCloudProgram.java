package com.orb.slam2s.rendering.gles;

import android.opengl.GLES20;
import android.util.Log;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

// 基于 OpenGL ES 2.0 的 3D 点云着色器程序 (GL_POINTS)
// 每个点数据属性包含 7 个 float: [x, y, z, r, g, b, point_size]
public class PointCloudProgram {
    private static final String TAG = "PointCloudProgram";

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

    private int program;
    private int vpMatrixHandle;
    private int positionHandle;
    private int colorHandle;
    private int pointSizeHandle;

    private FloatBuffer vertexBuffer;
    private int pointCount = 0;

    public void init() {
        int vertexShader = loadShader(GLES20.GL_VERTEX_SHADER, VERTEX_SHADER);
        int fragmentShader = loadShader(GLES20.GL_FRAGMENT_SHADER, FRAGMENT_SHADER);

        program = GLES20.glCreateProgram();
        GLES20.glAttachShader(program, vertexShader);
        GLES20.glAttachShader(program, fragmentShader);
        GLES20.glLinkProgram(program);

        vpMatrixHandle = GLES20.glGetUniformLocation(program, "uVPMatrix");
        positionHandle = GLES20.glGetAttribLocation(program, "aPosition");
        colorHandle = GLES20.glGetAttribLocation(program, "aColor");
        pointSizeHandle = GLES20.glGetAttribLocation(program, "aPointSize");
    }

    public void updatePoints(float[] pointData) {
        updatePoints(pointData, pointData == null ? 0 : pointData.length);
    }

    // 更新点云数据；data 可能为复用的大缓冲，count 为实际有效 float 数量
    public void updatePoints(float[] pointData, int count) {
        if (pointData == null || count <= 0) {
            pointCount = 0;
            return;
        }
        pointCount = count / 7;

        int requiredCapacity = count * 4;
        if (vertexBuffer == null || vertexBuffer.capacity() < requiredCapacity) {
            ByteBuffer bb = ByteBuffer.allocateDirect(requiredCapacity);
            bb.order(ByteOrder.nativeOrder());
            vertexBuffer = bb.asFloatBuffer();
        }
        vertexBuffer.clear();
        vertexBuffer.put(pointData, 0, count);
        vertexBuffer.position(0);
    }

    public void draw(float[] vpMatrix) {
        if (program == 0 || pointCount == 0 || vertexBuffer == null) return;

        GLES20.glUseProgram(program);

        GLES20.glUniformMatrix4fv(vpMatrixHandle, 1, false, vpMatrix, 0);

        int stride = 7 * 4; // 每个点 7 个 float (28 字节)

        vertexBuffer.position(0);
        GLES20.glEnableVertexAttribArray(positionHandle);
        GLES20.glVertexAttribPointer(positionHandle, 3, GLES20.GL_FLOAT, false, stride, vertexBuffer);

        vertexBuffer.position(3);
        GLES20.glEnableVertexAttribArray(colorHandle);
        GLES20.glVertexAttribPointer(colorHandle, 3, GLES20.GL_FLOAT, false, stride, vertexBuffer);

        vertexBuffer.position(6);
        GLES20.glEnableVertexAttribArray(pointSizeHandle);
        GLES20.glVertexAttribPointer(pointSizeHandle, 1, GLES20.GL_FLOAT, false, stride, vertexBuffer);

        GLES20.glDrawArrays(GLES20.GL_POINTS, 0, pointCount);

        GLES20.glDisableVertexAttribArray(positionHandle);
        GLES20.glDisableVertexAttribArray(colorHandle);
        GLES20.glDisableVertexAttribArray(pointSizeHandle);
    }

    public void destroy() {
        if (program != 0) {
            GLES20.glDeleteProgram(program);
            program = 0;
        }
    }

    private static int loadShader(int type, String shaderCode) {
        int shader = GLES20.glCreateShader(type);
        GLES20.glShaderSource(shader, shaderCode);
        GLES20.glCompileShader(shader);
        return shader;
    }
}