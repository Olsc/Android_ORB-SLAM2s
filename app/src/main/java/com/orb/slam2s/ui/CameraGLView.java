package com.orb.slam2s.ui;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.PixelFormat;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
import android.util.Log;
import android.view.SurfaceHolder;

import androidx.camera.core.Camera;
import androidx.camera.core.CameraSelector;
import androidx.camera.core.ImageAnalysis;
import androidx.camera.core.ImageProxy;
import androidx.camera.lifecycle.ProcessCameraProvider;
import androidx.core.content.ContextCompat;
import androidx.lifecycle.LifecycleOwner;

import com.orb.slam2s.constant.GlobalConstant;
import com.orb.slam2s.rendering.gles.OrthoFilter;
import com.orb.slam2s.utils.TextureUtils;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

/**
 * 相机 GL 视图 — 100% 纯 Java SDK 图像采集与 SharedMemory 帧推送
 */
@SuppressWarnings("deprecation")
public class CameraGLView extends CameraGLViewBase {

    private static final String TAG = "JavaCameraView";

    private ProcessCameraProvider cameraProvider;
    private Camera cameraX;
    private ImageAnalysis imageAnalysis;
    private ExecutorService analyzerExecutor;

    private byte[][] mYuvSendBuffers; // 灰度帧发送双缓冲（复用，避免每帧 clone）
    private byte[][] mRgbaBuffers;    // 相机 RGBA 帧双缓冲（复用，供灰度转换与彩色预览）
    private int mSendPing;            // 当前发送缓冲索引 (0/1)
    private int[] mCachePixels;       // 预览位图像素数组 (w * h)

    private OrthoFilter ortho;
    private com.orb.slam2s.rendering.gles.PointCloudProgram pointCloudProgram;
    // 点云读取缓冲（从共享内存直接读，零 binder）
    private final float[] pointCloudBuffer =
            new float[com.orb.slam2s.ipc.SharedMemoryBuffer.POINTCLOUD_MAX_BYTES / 4];
    // 3D 点云渲染：共享内存 MVP（M[16]+V[16]+P[16]）→ VP = P*V
    private final float[] tempMvp = new float[48];
    private final float[] vpMatrix = new float[16];
    private final float[] tmpVp = new float[16];
    // SLAM(RDF: 右-下-前) → GL(RUB: 右-上-后)：翻转 Y/Z。点云世界坐标为 RDF 系，
    // 而共享内存里的 view 是 RUB 系（AR 物体用，其 model 也是 RUB）——投影点云前必须补乘 Rx。
    private static final float[] RDF_TO_RUB = {
            1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, -1, 0,
            0, 0, 0, 1
    };

    private final Context context;
    private com.orb.slam2s.ipc.SlamIPCClient slamIPCClient;
    private volatile boolean mPendingDetectPlane; // 待处理的平面检测请求（下一帧触发）

    private ExecutorService mIpcSendExecutor;
    private final java.util.concurrent.atomic.AtomicBoolean mIsIpcProcessing = new java.util.concurrent.atomic.AtomicBoolean(false);

    public void setSlamIPCClient(com.orb.slam2s.ipc.SlamIPCClient client) {
        this.slamIPCClient = client;
    }

    /**
     * 请求在下一帧触发平面检测
     */
    public void requestPlaneDetection() {
        mPendingDetectPlane = true;
    }

    public CameraGLView(Context context, AttributeSet attrs) {
        super(context, attrs);
        this.context = context;
    }

    public void init() {
        setAspectRatio(GlobalConstant.RESOLUTION_WIDTH, GlobalConstant.RESOLUTION_HEIGHT);
        setEGLContextClientVersion(2);
        setEGLConfigChooser(8, 8, 8, 8, 16, 0);
        getHolder().setFormat(PixelFormat.TRANSLUCENT);
        setZOrderOnTop(false);

        setRenderer(new CameraGLRender());

        setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
        setPreserveEGLContextOnPause(true);
        ortho = new OrthoFilter(context);
    }

    protected boolean initializeCamera() {
        try {
            com.google.common.util.concurrent.ListenableFuture<ProcessCameraProvider> future = ProcessCameraProvider.getInstance(getContext());
            future.addListener(() -> {
                try {
                    cameraProvider = future.get();

                    mFrameWidth = GlobalConstant.RESOLUTION_WIDTH;
                    mFrameHeight = GlobalConstant.RESOLUTION_HEIGHT;
                    AllocateCache();

                    analyzerExecutor = Executors.newSingleThreadExecutor();
                    mIpcSendExecutor = Executors.newSingleThreadExecutor();

                    ImageAnalysis.Builder builder = new ImageAnalysis.Builder()
                            .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                            .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
                            .setTargetResolution(new android.util.Size(mFrameWidth, mFrameHeight));

                    imageAnalysis = builder.build();
                    imageAnalysis.setAnalyzer(analyzerExecutor, new ImageAnalysis.Analyzer() {
                        @Override
                        public void analyze(ImageProxy image) {
                            if (analyzerExecutor == null || analyzerExecutor.isShutdown()) {
                                image.close();
                                return;
                            }
                            try {
                                int w = image.getWidth();
                                int h = image.getHeight();
                                ImageProxy.PlaneProxy[] planes = image.getPlanes();
                                if (planes == null || planes.length == 0) {
                                    image.close();
                                    return;
                                }

                                ImageProxy.PlaneProxy rgbaPlane = planes[0];
                                java.nio.ByteBuffer buf = rgbaPlane.getBuffer();
                                if (buf == null) {
                                    image.close();
                                    return;
                                }

                                int rowStride = rgbaPlane.getRowStride();
                                int requiredSize = w * h;
                                int rgbaSize = w * h * 4;

                                if (mRgbaBuffers == null) {
                                    mRgbaBuffers = new byte[2][];
                                }
                                if (mRgbaBuffers[mSendPing] == null
                                        || mRgbaBuffers[mSendPing].length < rgbaSize) {
                                    mRgbaBuffers[mSendPing] = new byte[rgbaSize];
                                }
                                byte[] rgbaBuf = mRgbaBuffers[mSendPing];

                                int bufPos = buf.position();
                                if (rowStride == w * 4) {
                                    buf.get(rgbaBuf, 0, Math.min(buf.remaining(), rgbaSize));
                                } else {
                                    for (int row = 0; row < h; row++) {
                                        buf.position(bufPos + row * rowStride);
                                        buf.get(rgbaBuf, row * w * 4, Math.min(w * 4, buf.remaining()));
                                    }
                                }
                                buf.position(bufPos);

                                if (mYuvSendBuffers == null) {
                                    mYuvSendBuffers = new byte[2][];
                                }
                                if (mYuvSendBuffers[mSendPing] == null
                                        || mYuvSendBuffers[mSendPing].length < requiredSize) {
                                    mYuvSendBuffers[mSendPing] = new byte[requiredSize];
                                }
                                byte[] yBuf = mYuvSendBuffers[mSendPing];

                                if (mCacheBitmap != null && !mCacheBitmap.isRecycled()) {
                                    if (mCachePixels == null || mCachePixels.length < requiredSize) {
                                        mCachePixels = new int[requiredSize];
                                    }
                                    for (int i = 0; i < requiredSize; i++) {
                                        int idx = i * 4;
                                        int r = rgbaBuf[idx] & 0xFF;
                                        int g = rgbaBuf[idx + 1] & 0xFF;
                                        int b = rgbaBuf[idx + 2] & 0xFF;
                                        yBuf[i] = (byte) ((r * 77 + g * 150 + b * 29) >> 8);
                                        mCachePixels[i] = 0xFF000000 | (r << 16) | (g << 8) | b;
                                    }
                                    mCacheBitmap.setPixels(mCachePixels, 0, w, 0, 0, w, h);
                                    queueEvent(() -> {
                                        synchronized (mSyncObject) {
                                            if (mCacheBitmap != null && !mCacheBitmap.isRecycled()) {
                                                TextureUtils.loadTexture(mCacheBitmap, imageTextureId);
                                            }
                                        }
                                    });
                                } else {
                                    for (int i = 0; i < requiredSize; i++) {
                                        int idx = i * 4;
                                        int r = rgbaBuf[idx] & 0xFF;
                                        int g = rgbaBuf[idx + 1] & 0xFF;
                                        int b = rgbaBuf[idx + 2] & 0xFF;
                                        yBuf[i] = (byte) ((r * 77 + g * 150 + b * 29) >> 8);
                                    }
                                }

                                com.orb.slam2s.compat.DeviceCompat_RokidGlass3.checkAndFlipFrame(yBuf, w, h);

                                if (slamIPCClient != null && slamIPCClient.isConnected()) {
                                    if (mIsIpcProcessing.compareAndSet(false, true)) {
                                        final byte[] sendBuffer = yBuf;
                                        mSendPing ^= 1;
                                        final int frameW = w;
                                        final int frameH = h;
                                        if (mIpcSendExecutor != null && !mIpcSendExecutor.isShutdown()) {
                                            mIpcSendExecutor.execute(() -> {
                                                try {
                                                    slamIPCClient.sendFrameData(sendBuffer, frameW, frameH);
                                                    if (mPendingDetectPlane) {
                                                        mPendingDetectPlane = false;
                                                        slamIPCClient.detectPlane();
                                                    }
                                                } catch (Exception e) {
                                                    Log.e(TAG, "异步 IPC 发送帧异常: " + e.getMessage());
                                                } finally {
                                                    mIsIpcProcessing.set(false);
                                                }
                                            });
                                        } else {
                                            mIsIpcProcessing.set(false);
                                        }
                                    }
                                }

                                deliverAndDrawFrame(new XCameraFrame());

                                image.close();
                            } catch (Throwable e) {
                                Log.e(TAG, "相机帧分析与灰度图层更新错误: " + e.getMessage());
                                try {
                                    image.close();
                                } catch (Exception ignored) {}
                            }
                        }
                    });

                    CameraSelector selector = new CameraSelector.Builder()
                            .requireLensFacing(CameraSelector.LENS_FACING_BACK)
                            .build();

                    cameraProvider.unbindAll();
                    cameraX = cameraProvider.bindToLifecycle((LifecycleOwner) getContext(), selector, imageAnalysis);
                } catch (Exception e) {
                    Log.e(TAG, "CameraX 初始化失败: " + e.getMessage());
                }
            }, ContextCompat.getMainExecutor(getContext()));
        } catch (Exception ex) {
            Log.e(TAG, "初始化 CameraX 异常: " + ex.getMessage());
            return false;
        }
        return true;
    }

    @Override
    protected boolean connectCamera(int width, int height) {
        return initializeCamera();
    }

    @Override
    protected void disconnectCamera() {
        if (cameraProvider != null) {
            final ProcessCameraProvider provider = cameraProvider;
            new android.os.Handler(android.os.Looper.getMainLooper()).post(new Runnable() {
                @Override
                public void run() {
                    try {
                        provider.unbindAll();
                    } catch (Exception e) {
                        Log.e(TAG, "unbindAll error: " + e.getMessage());
                    }
                }
            });
        }
        if (analyzerExecutor != null) {
            analyzerExecutor.shutdown();
            analyzerExecutor = null;
        }
        if (mIpcSendExecutor != null) {
            mIpcSendExecutor.shutdown();
            mIpcSendExecutor = null;
        }
    }

    public void autoFocusCenter() {
        try {
            if (cameraX == null) return;
            androidx.camera.core.MeteringPointFactory factory = new androidx.camera.core.SurfaceOrientedMeteringPointFactory(getWidth(), getHeight());
            androidx.camera.core.MeteringPoint point = factory.createPoint(getWidth()/2f, getHeight()/2f);
            androidx.camera.core.FocusMeteringAction action = new androidx.camera.core.FocusMeteringAction.Builder(point).setAutoCancelDuration(3, java.util.concurrent.TimeUnit.SECONDS).build();
            cameraX.getCameraControl().startFocusAndMetering(action);
        } catch (Exception e) {
            Log.e(TAG, "居中心自动对焦错误: " + e.getMessage());
        }
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        super.surfaceDestroyed(holder);
        synchronized(mSyncObject) {
            mSurfaceExist = false;
            checkCurrentState();
            ortho.destroy();
            if (pointCloudProgram != null) {
                pointCloudProgram.destroy();
                pointCloudProgram = null;
            }
        }
    }

    class CameraGLRender implements GLSurfaceView.Renderer {
        @Override
        public void onSurfaceCreated(GL10 gl, EGLConfig config) {
            Bitmap bitmap = Bitmap.createBitmap(GlobalConstant.RESOLUTION_WIDTH, GlobalConstant.RESOLUTION_HEIGHT, Bitmap.Config.ARGB_8888);
            imageTextureId = TextureUtils.loadTexture(bitmap, 0);
            bitmap.recycle();
            ortho.init();

            pointCloudProgram = new com.orb.slam2s.rendering.gles.PointCloudProgram();
            pointCloudProgram.init();
        }

        @Override
        public void onSurfaceChanged(GL10 gl, int width, int height) {
            ortho.onSurfaceChanged(width, height);
            synchronized(mSyncObject) {
                if (!mSurfaceExist) {
                    mSurfaceExist = true;
                    checkCurrentState();
                } else {
                    mSurfaceExist = false;
                    checkCurrentState();
                    mSurfaceExist = true;
                    checkCurrentState();
                }
            }
        }

        @Override
        public void onDrawFrame(GL10 gl) {
            GLES20.glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
            GLES20.glClear(GLES20.GL_DEPTH_BUFFER_BIT | GLES20.GL_COLOR_BUFFER_BIT);
            ortho.onDrawFrame(imageTextureId);

            if (slamIPCClient != null && slamIPCClient.isConnected() && pointCloudProgram != null) {
                int floats = slamIPCClient.readPointCloud(pointCloudBuffer, pointCloudBuffer.length);
                if (floats > 0 && slamIPCClient.readMvp(tempMvp)) {
                    android.opengl.Matrix.multiplyMM(tmpVp, 0, tempMvp, 16, RDF_TO_RUB, 0);
                    android.opengl.Matrix.multiplyMM(vpMatrix, 0, tempMvp, 32, tmpVp, 0);
                    pointCloudProgram.updatePoints(pointCloudBuffer, floats);
                    GLES20.glDisable(GLES20.GL_DEPTH_TEST);
                    pointCloudProgram.draw(vpMatrix);
                    GLES20.glEnable(GLES20.GL_DEPTH_TEST);
                }
            }
        }
    }

    private static class XCameraFrame implements CameraGLViewBase.CvCameraViewFrame {
        @Override
        public long rgba() { return 0; }
        @Override
        public long gray() { return 0; }
    }
}