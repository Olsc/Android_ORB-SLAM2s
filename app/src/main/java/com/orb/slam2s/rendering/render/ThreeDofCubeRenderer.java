package com.orb.slam2s.rendering.render;

import android.content.Context;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.util.Log;
import android.view.WindowManager;

import com.orb.slam2s.sensors.OrientationSensor;
import com.orb.slam2s.slamar.NativeHelper;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

// 3DOF立方体渲染器：在视角前方指定距离处生成彩色立方体并进行3DOF跟踪
public class ThreeDofCubeRenderer implements GLSurfaceView.Renderer {
    private static final String TAG = "ThreeDofCubeRenderer";

    private boolean mInitialized = false;
    private float[] mObjectWorldPos = new float[3];

    private final OrientationSensor orientationSensor;
    private int program;

    private final float[] mvpMatrix = new float[16];
    private float mRatio = 1.0f;
    private float mDistance = 5.0f; // 默认5米

    private FloatBuffer vertexBuffer;
    private FloatBuffer colorBuffer;
    private ShortBuffer indexBuffer;

    private static final int CUBE_INDEX_COUNT = 36;
    private final Context context;
    private NativeHelper nativeHelper;

    // 控制是否显示立方体
    private boolean mShowCube = false;

    public ThreeDofCubeRenderer(Context context, OrientationSensor sensor, NativeHelper nativeHelper) {
        this.context = context;
        this.orientationSensor = sensor;
        this.nativeHelper = nativeHelper;
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

        int vertexShader = loadShader(GLES20.GL_VERTEX_SHADER, vertexShaderCode);
        int fragmentShader = loadShader(GLES20.GL_FRAGMENT_SHADER, fragmentShaderCode);

        program = GLES20.glCreateProgram();
        GLES20.glAttachShader(program, vertexShader);
        GLES20.glAttachShader(program, fragmentShader);
        GLES20.glLinkProgram(program);

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

        int rotation = getDisplayRotation();

        float[] rotationMatrix = orientationSensor.getRotationMatrix();

        // 检查旋转矩阵是否有效
        boolean isIdentity = true;
        for (int i = 1; i < 4; i++) {
            if (rotationMatrix[i] != 0) isIdentity = false;
        }

        // 延迟初始化：当获取到有效的传感器数据后，在相机前方固定距离生成物体
        if (!mInitialized && !isIdentity) {
            mObjectWorldPos = nativeHelper.calculate3DofInsertionPoint(rotationMatrix, rotation, mDistance);
            mInitialized = true;
            Log.d(TAG, String.format("立方体世界坐标已初始化: [%.2f, %.2f, %.2f]",
                    mObjectWorldPos[0], mObjectWorldPos[1], mObjectWorldPos[2]));
        }

        if (mInitialized) {
            // J-9：出参版 compute3DofMVP 直接填充复用缓冲，消除每帧一次的 JNI float[16] 分配
            nativeHelper.compute3DofMVP(mvpMatrix, rotationMatrix, rotation, mRatio, mObjectWorldPos);
            drawCube();
        }
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

        // 为每个面分配不同的颜色
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

    // J-9：句柄在链接后缓存——原先每帧 3 次驱动级查询是纯开销
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

    private int loadShader(int type, String shaderCode) {
        int shader = GLES20.glCreateShader(type);
        GLES20.glShaderSource(shader, shaderCode);
        GLES20.glCompileShader(shader);
        return shader;
    }

    private int getDisplayRotation() {
        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.R) {
            try {
                return context.getDisplay().getRotation();
            } catch (Exception | NoSuchMethodError e) {
                // 回退方案
            }
        }
        return getLegacyRotation();
    }

    @SuppressWarnings("deprecation")
    private int getLegacyRotation() {
        WindowManager wm = (WindowManager) context.getSystemService(Context.WINDOW_SERVICE);
        if (wm != null) {
            return wm.getDefaultDisplay().getRotation();
        }
        return 0;
    }
}