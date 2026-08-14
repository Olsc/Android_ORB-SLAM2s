package com.orb.slam2s.ui;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.graphics.PixelFormat;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
import android.util.Log;
import android.view.Surface;
import android.view.SurfaceHolder;

import androidx.camera.core.Camera;
import androidx.camera.core.CameraSelector;
import androidx.camera.core.ImageAnalysis;
import androidx.camera.core.ImageProxy;
import androidx.camera.lifecycle.ProcessCameraProvider;
import androidx.core.content.ContextCompat;
import androidx.lifecycle.LifecycleOwner;

import com.orb.slam2s.compat.DeviceCompat_RokidGlass3;
import com.orb.slam2s.constant.GlobalConstant;
import com.orb.slam2s.rendering.gles.OrthoFilter;
import com.orb.slam2s.slamar.OpenCVBridge;
import com.orb.slam2s.utils.TextureUtils;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

// 相机 GL 视图，使用 CameraX + OpenCVBridge 处理帧
@SuppressWarnings("deprecation")
public class CameraGLView extends CameraGLViewBase {

    private static final String TAG = "JavaCameraView";

    private ProcessCameraProvider cameraProvider;
    private Camera cameraX;
    private ImageAnalysis imageAnalysis;
    private ExecutorService analyzerExecutor;

    private long rgbaMatAddr;     // native Mat 地址 (CV_8UC4)
    private long grayMatAddr;     // native Mat 地址 (CV_8UC1)

    private byte[] mYuvDataCache; // 复用缓冲区，避免每帧 GC 分配

    private OrthoFilter ortho;
    private final Context context;
    private volatile boolean mIsStopped = false;

    public CameraGLView(Context context, AttributeSet attrs) {
        super(context, attrs);
        this.context=context;
    }

    public void init(){
        setAspectRatio(GlobalConstant.RESOLUTION_WIDTH,GlobalConstant.RESOLUTION_HEIGHT);
        setEGLContextClientVersion(2);
        setEGLConfigChooser(8, 8, 8, 8, 16, 0);
        getHolder().setFormat(PixelFormat.TRANSLUCENT);
        setZOrderOnTop(false);

        setRenderer(new CameraGLRender());

        setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
        setPreserveEGLContextOnPause(true);
        ortho=new OrthoFilter(context);
    }
    protected boolean initializeCamera(int width, int height) {
        Log.d(TAG, "初始化 CameraX");
        try {
            com.google.common.util.concurrent.ListenableFuture<ProcessCameraProvider> future = ProcessCameraProvider.getInstance(getContext());
            future.addListener(() -> {
                try {
                    cameraProvider = future.get();

                    mFrameWidth = GlobalConstant.RESOLUTION_WIDTH;
                    mFrameHeight = GlobalConstant.RESOLUTION_HEIGHT;
                    AllocateCache();

                    mIsStopped = false;
                    analyzerExecutor = Executors.newSingleThreadExecutor();

                    ImageAnalysis.Builder builder = new ImageAnalysis.Builder()
                            .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                            .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
                            .setTargetResolution(new android.util.Size(mFrameWidth, mFrameHeight));

                    imageAnalysis = builder.build();
                    imageAnalysis.setAnalyzer(analyzerExecutor, new ImageAnalysis.Analyzer() {
                        @Override
                        public void analyze(ImageProxy image) {
                            if (mIsStopped || isWebTickerRunning || analyzerExecutor == null || analyzerExecutor.isShutdown()) {
                                image.close();
                                return;
                            }
                            try {
                                int w = image.getWidth();
                                int h = image.getHeight();
                                int format = image.getFormat();
                                ImageProxy.PlaneProxy[] planes = image.getPlanes();
                                if (planes == null || planes.length == 0) {
                                    image.close();
                                    return;
                                }

                                if (format == ImageFormat.YUV_420_888 || format == ImageFormat.NV21 || planes.length >= 2) {
                                    // YUV 格式：Y-plane 本身就是灰度图
                                    ImageProxy.PlaneProxy yPlane = planes[0];
                                    java.nio.ByteBuffer yBuf = yPlane.getBuffer();
                                    if (yBuf == null) { image.close(); return; }
                                    int rowStride = yPlane.getRowStride();

                                    // 创建 RGBA Mat（仅首次）
                                    if (rgbaMatAddr == 0) {
                                        rgbaMatAddr = OpenCVBridge.nativeCreateMat(h, w, OpenCVBridge.CV_8UC4);  // CV_8UC4
                                    }
                                    // 创建 Gray Mat（仅首次）
                                    if (grayMatAddr == 0) {
                                        grayMatAddr = OpenCVBridge.nativeCreateMat(h, w, OpenCVBridge.CV_8UC1);  // CV_8UC1
                                    }

                                    if (yBuf.isDirect() && rowStride == w) {
                                        // 零拷贝直传 JNI，无需 Java 字节数组拷贝与循环
                                        OpenCVBridge.nativeDirectYPlaneToMats(rgbaMatAddr, grayMatAddr, yBuf, w, h);
                                    } else {
                                        // 备用兼容路径：逐行拷贝去除填充
                                        int requiredSize = w * h;
                                        if (mYuvDataCache == null || mYuvDataCache.length < requiredSize) {
                                            mYuvDataCache = new byte[requiredSize];
                                        }
                                        int bufPos = yBuf.position();
                                        for (int row = 0; row < h; row++) {
                                            yBuf.position(bufPos + row * rowStride);
                                            yBuf.get(mYuvDataCache, row * w, w);
                                        }
                                        OpenCVBridge.nativeYPlaneToMats(rgbaMatAddr, grayMatAddr, mYuvDataCache, w, h);
                                    }

                                    image.close();
                                } else {
                                    // RGBA 或其他格式 — 原有逻辑
                                    java.nio.ByteBuffer buf = planes[0].getBuffer();
                                    if (buf == null) { image.close(); return; }

                                    // 创建 RGBA Mat（仅首次）
                                    if (rgbaMatAddr == 0) {
                                        rgbaMatAddr = OpenCVBridge.nativeCreateMat(h, w, OpenCVBridge.CV_8UC4);  // CV_8UC4
                                    }
                                    // 将 ByteBuffer 直接写入 native Mat（零 Java 中间拷贝）
                                    OpenCVBridge.nativePutBuffer(rgbaMatAddr, buf);

                                    // 数据拷贝到 native 层后立即关闭 Image，释放 CameraX 缓冲区
                                    image.close();

                                    // 创建 Gray Mat（仅首次）
                                    if (grayMatAddr == 0) {
                                        grayMatAddr = OpenCVBridge.nativeCreateMat(h, w, OpenCVBridge.CV_8UC1);  // CV_8UC1
                                    }
                                    // RGBA→Gray 纯 C++ 层完成，无需 Java 介入
                                    OpenCVBridge.nativeRGBA2Gray(rgbaMatAddr, grayMatAddr);
                                }

                                // 如果是右横屏 (ROTATION_270)，需要旋转180度补偿
                                if (GlobalConstant.DISPLAY_ROTATION == Surface.ROTATION_270) {
                                    OpenCVBridge.nativeRotate180(rgbaMatAddr);
                                    OpenCVBridge.nativeRotate180(grayMatAddr);
                                }

                                // 针对特定设备的兼容性处理
                                DeviceCompat_RokidGlass3.checkAndFlipFrame(rgbaMatAddr);
                                DeviceCompat_RokidGlass3.checkAndFlipFrame(grayMatAddr);

                                if (!mIsStopped) {
                                    deliverAndDrawFrame(new XCameraFrame(rgbaMatAddr, grayMatAddr));
                                }
                            } catch (Throwable e) {
                                Log.e(TAG, "分析错误: " + e.getMessage());
                                try { image.close(); } catch (Exception ignored) {}
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
        Log.d(TAG, "正在连接 CameraX");
        if (!initializeCamera(width, height)) return false;
        return true;
    }

    private android.os.HandlerThread mWebThread;
    private android.os.Handler mWebHandler;
    private volatile boolean isWebTickerRunning = false;

    private final Runnable webTickerRunnable = new Runnable() {
        @Override
        public void run() {
            if (isWebTickerRunning && !mIsStopped) {
                int w = GlobalConstant.RESOLUTION_WIDTH;
                int h = GlobalConstant.RESOLUTION_HEIGHT;
                if (rgbaMatAddr == 0) {
                    rgbaMatAddr = OpenCVBridge.nativeCreateMat(h, w, OpenCVBridge.CV_8UC4);
                    OpenCVBridge.nativeMatSetTo(rgbaMatAddr, 0, 0, 0, 255);
                }
                if (grayMatAddr == 0) {
                    grayMatAddr = OpenCVBridge.nativeCreateMat(h, w, OpenCVBridge.CV_8UC1);
                    OpenCVBridge.nativeMatSetTo(grayMatAddr, 0, 0, 0, 0);
                }
                deliverAndDrawFrame(new XCameraFrame(rgbaMatAddr, grayMatAddr));
                if (mWebHandler != null && isWebTickerRunning) {
                    mWebHandler.postDelayed(this, 33);
                }
            }
        }
    };

    // 开启 Web 模式渲染脉冲，关闭物理相机硬件传感器，由后台 HandlerThread 独立驱动
    public void startWebModeTicker() {
        Log.d(TAG, "开启 Web 模式后台渲染脉冲，关闭物理相机传感器");
        isWebTickerRunning = true;
        pauseCameraSensor();

        int w = GlobalConstant.RESOLUTION_WIDTH;
        int h = GlobalConstant.RESOLUTION_HEIGHT;
        if (rgbaMatAddr == 0) {
            rgbaMatAddr = OpenCVBridge.nativeCreateMat(h, w, OpenCVBridge.CV_8UC4);
        }
        OpenCVBridge.nativeMatSetTo(rgbaMatAddr, 0, 0, 0, 255);

        if (grayMatAddr == 0) {
            grayMatAddr = OpenCVBridge.nativeCreateMat(h, w, OpenCVBridge.CV_8UC1);
        }
        OpenCVBridge.nativeMatSetTo(grayMatAddr, 0, 0, 0, 0);

        if (mWebThread == null || !mWebThread.isAlive()) {
            mWebThread = new android.os.HandlerThread("WebRenderThread");
            mWebThread.start();
            mWebHandler = new android.os.Handler(mWebThread.getLooper());
        }
        mWebHandler.removeCallbacks(webTickerRunnable);
        mWebHandler.post(webTickerRunnable);
    }

    // 停止 Web 模式渲染脉冲，恢复物理相机硬件传感器
    public void stopWebModeTicker() {
        Log.d(TAG, "停止 Web 模式后台渲染脉冲，恢复物理相机传感器");
        isWebTickerRunning = false;
        if (mWebHandler != null) {
            mWebHandler.removeCallbacks(webTickerRunnable);
        }
        if (mWebThread != null) {
            mWebThread.quitSafely();
            mWebThread = null;
            mWebHandler = null;
        }
        resumeCameraSensor();
    }

    // 暂停物理相机硬件传感器，Web 模式下释放物理硬件与指示灯
    public void pauseCameraSensor() {
        Log.d(TAG, "Web模式：关闭物理相机硬件传感器");
        if (cameraProvider != null) {
            new android.os.Handler(android.os.Looper.getMainLooper()).post(() -> {
                try {
                    cameraProvider.unbindAll();
                } catch (Exception e) {
                    Log.e(TAG, "pauseCameraSensor error: " + e.getMessage());
                }
            });
        }
    }

    // 恢复物理相机硬件传感器，切回本地 SLAM 模式
    public void resumeCameraSensor() {
        Log.d(TAG, "恢复物理相机硬件传感器");
        if (mIsStopped) return;
        new android.os.Handler(android.os.Looper.getMainLooper()).post(() -> {
            try {
                if (cameraProvider != null) {
                    cameraProvider.unbindAll();
                }
                initializeCamera(getWidth(), getHeight());
            } catch (Exception e) {
                Log.e(TAG, "resumeCameraSensor error: " + e.getMessage());
            }
        });
    }

    @Override
    protected void disconnectCamera() {
        Log.d(TAG, "正在断开 CameraX");
        mIsStopped = true;
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
            final ExecutorService exec = analyzerExecutor;
            analyzerExecutor = null;
            exec.execute(new Runnable() {
                @Override
                public void run() {
                    if (rgbaMatAddr != 0) { OpenCVBridge.nativeReleaseMat(rgbaMatAddr); rgbaMatAddr = 0; }
                    if (grayMatAddr != 0) { OpenCVBridge.nativeReleaseMat(grayMatAddr); grayMatAddr = 0; }
                }
            });
            exec.shutdown();
        } else {
            if (rgbaMatAddr != 0) { OpenCVBridge.nativeReleaseMat(rgbaMatAddr); rgbaMatAddr = 0; }
            if (grayMatAddr != 0) { OpenCVBridge.nativeReleaseMat(grayMatAddr); grayMatAddr = 0; }
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

    private static class XCameraFrame implements CvCameraViewFrame {
        private final long rgbaAddr;
        private final long grayAddr;
        XCameraFrame(long rgbaAddr, long grayAddr) {
            this.rgbaAddr = rgbaAddr;
            this.grayAddr = grayAddr;
        }
        @Override
        public long rgba() { return rgbaAddr; }
        @Override
        public long gray() { return grayAddr; }
    }
}