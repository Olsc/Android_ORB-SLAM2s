package com.orb.slam2s.ui;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.ImageFormat;
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

    private byte[] mYuvDataCache; // 复用缓冲区，避免每帧 GC 分配

    private OrthoFilter ortho;
    private final Context context;
    private final Object mAnalyzeLock = new Object();
    private com.orb.slam2s.ipc.SlamIPCClient slamIPCClient;
    private volatile boolean mPendingDetectPlane; // 待处理的平面检测请求（下一帧触发）

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
        //Log.d(TAG, "初始化 CameraX");
        try {
            com.google.common.util.concurrent.ListenableFuture<ProcessCameraProvider> future = ProcessCameraProvider.getInstance(getContext());
            future.addListener(() -> {
                try {
                    cameraProvider = future.get();

                    mFrameWidth = GlobalConstant.RESOLUTION_WIDTH;
                    mFrameHeight = GlobalConstant.RESOLUTION_HEIGHT;
                    AllocateCache();

                    analyzerExecutor = Executors.newSingleThreadExecutor();

                    ImageAnalysis.Builder builder = new ImageAnalysis.Builder()
                            .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                            .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
                            .setTargetResolution(new android.util.Size(mFrameWidth, mFrameHeight));

                    imageAnalysis = builder.build();
                    imageAnalysis.setAnalyzer(analyzerExecutor, new ImageAnalysis.Analyzer() {
                        @Override
                        public void analyze(ImageProxy image) {
                            synchronized (mAnalyzeLock) {
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

                                    ImageProxy.PlaneProxy mainPlane = planes[0];
                                    java.nio.ByteBuffer buf = mainPlane.getBuffer();
                                    if (buf == null) {
                                        image.close();
                                        return;
                                    }

                                    int rowStride = mainPlane.getRowStride();
                                    int pixelStride = mainPlane.getPixelStride();
                                    int lineBytes = w * pixelStride;
                                    int requiredSize = w * h * pixelStride;
                                    if (mYuvDataCache == null || mYuvDataCache.length < requiredSize) {
                                        mYuvDataCache = new byte[requiredSize];
                                    }

                                    int bufPos = buf.position();
                                    if (rowStride == lineBytes) {
                                        buf.get(mYuvDataCache, 0, Math.min(buf.remaining(), requiredSize));
                                    } else {
                                        for (int row = 0; row < h; row++) {
                                            buf.position(bufPos + row * rowStride);
                                            buf.get(mYuvDataCache, row * lineBytes, Math.min(lineBytes, buf.remaining()));
                                        }
                                    }
                                    buf.position(bufPos);

                                    // 1. 通过 SharedMemory 零拷贝将完整 RGBA 帧推送至 MenthaAR SLAM 进程
                                    if (slamIPCClient != null && slamIPCClient.isConnected()) {
                                        slamIPCClient.sendFrameData(mYuvDataCache, w, h);

                                        // 2. 消费"创建AR物体"请求：在下一帧触发平面检测
                                        if (mPendingDetectPlane) {
                                            mPendingDetectPlane = false;
                                            slamIPCClient.detectPlane();
                                        }
                                    }

                                    // 2. 实时从 SharedMemory 映射的 ByteBuffer 中提取由 C++ SLAM 引擎绘制了绿/蓝色点云与特征点的 RGBA 图像
                                    java.nio.ByteBuffer sharedBuf = (slamIPCClient != null) ? slamIPCClient.getSharedMemoryBuffer() : null;
                                    if (mCacheBitmap != null && !mCacheBitmap.isRecycled()) {
                                        if (sharedBuf != null) {
                                            sharedBuf.position(0);
                                            mCacheBitmap.copyPixelsFromBuffer(sharedBuf);
                                        } else {
                                            mCacheBitmap.copyPixelsFromBuffer(java.nio.ByteBuffer.wrap(mYuvDataCache));
                                        }
                                        queueEvent(() -> {
                                            synchronized (mSyncObject) {
                                                if (mCacheBitmap != null && !mCacheBitmap.isRecycled()) {
                                                    TextureUtils.loadTexture(mCacheBitmap, imageTextureId);
                                                }
                                            }
                                        });
                                    }

                                    // 3. 通过帧回调通知宿主（ArCamUIActivity.onCameraFrame 用于 FPS 等统计）
                                    deliverAndDrawFrame(new XCameraFrame());

                                    image.close();
                                } catch (Throwable e) {
                                    Log.e(TAG, "相机帧分析与点云图层更新错误: " + e.getMessage());
                                    try {
                                        image.close();
                                    } catch (Exception ignored) {}
                                }
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
        //Log.d(TAG, "正在连接 CameraX");
        return initializeCamera();
    }

    @Override
    protected void disconnectCamera() {
        //Log.d(TAG, "正在断开 CameraX");
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
        synchronized (mAnalyzeLock) {
            if (analyzerExecutor != null) {
                analyzerExecutor.shutdown();
                analyzerExecutor = null;
            }
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
        }
    }

    class CameraGLRender implements GLSurfaceView.Renderer {
        @Override
        public void onSurfaceCreated(GL10 gl, EGLConfig config) {
            Bitmap bitmap= Bitmap.createBitmap(GlobalConstant.RESOLUTION_WIDTH,GlobalConstant.RESOLUTION_HEIGHT, Bitmap.Config.ARGB_8888);
            imageTextureId= TextureUtils.loadTexture(bitmap,0);
            bitmap.recycle();
            ortho.init();
        }

        @Override
        public void onSurfaceChanged(GL10 gl, int width, int height) {
            Log.d(TAG, "触发 surfaceChanged 事件");
            ortho.onSurfaceChanged(width,height);
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
            GLES20.glClear( GLES20.GL_DEPTH_BUFFER_BIT | GLES20.GL_COLOR_BUFFER_BIT);
            ortho.onDrawFrame(imageTextureId);
        }
    }

    // 供 onCameraFrame 回调使用的帧载体（CameraX 路径无原生 Mat 地址，onCameraFrame 仅用于统计）
    private static class XCameraFrame implements CameraGLViewBase.CvCameraViewFrame {
        @Override
        public long rgba() { return 0; }
        @Override
        public long gray() { return 0; }
    }
}