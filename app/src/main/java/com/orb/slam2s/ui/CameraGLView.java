package com.orb.slam2s.ui;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.PixelFormat;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.os.Build;
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
import com.orb.slam2s.utils.TextureUtils;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

/**
 * 该类是 OpenCV 与 Java 摄像头之间的桥接视图实现。
 * 它依赖于基类中已有的功能，并只实现必要的函数：
 * connectCamera - 打开 Java 摄像头并设置 PreviewCallback（预览回调）。
 * disconnectCamera - 关闭摄像头并停止预览。
 * 当摄像头通过回调传来帧数据时，这些帧会通过 OpenCV 进行处理，转换为 RGBA32 格式，
 * 并在必要时传递给外部回调以进行进一步处理。
 */

@SuppressWarnings("deprecation")
public class CameraGLView extends CameraGLViewBase {

    private static final int MAGIC_TEXTURE_ID = 10;
    private static final String TAG = "JavaCameraView";

    private ProcessCameraProvider cameraProvider;
    private Camera cameraX;
    private ImageAnalysis imageAnalysis;
    private ExecutorService analyzerExecutor;

    private byte[] rgbaBuffer;
    private Mat rgbaMatCache;
    private Mat grayMatCache;

    private OrthoFilter ortho;
    private Context context;
    private final Object mAnalyzeLock = new Object();

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
        if(Build.VERSION.SDK_INT>=Build.VERSION_CODES.HONEYCOMB){
            setPreserveEGLContextOnPause(true);
        }
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
                                    ImageProxy.PlaneProxy plane = image.getPlanes()[0];
                                    java.nio.ByteBuffer buf = plane.getBuffer();
                                    if (buf == null) { image.close(); return; }
                                    int size = buf.remaining();
                                    if (rgbaBuffer == null || rgbaBuffer.length != size) {
                                        rgbaBuffer = new byte[size];
                                    }
                                    buf.get(rgbaBuffer);

                                    // 数据复制完成后立即关闭Image，释放缓冲区，防止ImageReader卡死
                                    image.close();

                                    if (rgbaMatCache == null || rgbaMatCache.cols() != w || rgbaMatCache.rows() != h) {
                                        if (rgbaMatCache != null) rgbaMatCache.release();
                                        rgbaMatCache = new Mat(h, w, CvType.CV_8UC4);
                                    }
                                    rgbaMatCache.put(0, 0, rgbaBuffer);

                                    if (grayMatCache == null || grayMatCache.cols() != w || grayMatCache.rows() != h) {
                                        if (grayMatCache != null) grayMatCache.release();
                                        grayMatCache = new Mat(h, w, CvType.CV_8UC1);
                                    }
                                    Imgproc.cvtColor(rgbaMatCache, grayMatCache, Imgproc.COLOR_RGBA2GRAY);

                                    // 如果是右横屏 (ROTATION_270)，需要旋转180度补偿
                                    if (GlobalConstant.DISPLAY_ROTATION == Surface.ROTATION_270) {
                                        org.opencv.core.Core.rotate(rgbaMatCache, rgbaMatCache, org.opencv.core.Core.ROTATE_180);
                                        org.opencv.core.Core.rotate(grayMatCache, grayMatCache, org.opencv.core.Core.ROTATE_180);
                                    }

                                    // 针对特定设备的兼容性处理
                                    DeviceCompat_RokidGlass3.checkAndFlipFrame(rgbaMatCache);
                                    DeviceCompat_RokidGlass3.checkAndFlipFrame(grayMatCache);

                                    deliverAndDrawFrame(new XCameraFrame(rgbaMatCache, grayMatCache));
                                } catch (Throwable e) {
                                    Log.e(TAG, "分析错误: " + e.getMessage());
                                    // 确保发生异常时也能关闭image (如果还没关闭)
                                    try { image.close(); } catch (Exception ignored) {}
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
        Log.d(TAG, "正在连接 CameraX");
        if (!initializeCamera(width, height)) return false;
        return true;
    }

    @Override
    protected void disconnectCamera() {
        /* 1. 我们需要停止更新帧的线程
         * 2. 停止相机并释放它
         */
        Log.d(TAG, "正在断开 CameraX");
        if (cameraProvider != null) {
            cameraProvider.unbindAll();
        }
        synchronized (mAnalyzeLock) {
            if (analyzerExecutor != null) {
                analyzerExecutor.shutdown();
                analyzerExecutor = null;
            }
            if (rgbaMatCache != null) { rgbaMatCache.release(); rgbaMatCache = null; }
            if (grayMatCache != null) { grayMatCache.release(); grayMatCache = null; }
            rgbaBuffer = null;
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
            // 更新正交投影以处理可能的旋转
            ortho.onSurfaceChanged(gl,width,height);
            synchronized(mSyncObject) {
                if (!mSurfaceExist) {
                    mSurfaceExist = true;
                    checkCurrentState();
                } else {
                    /** 表面已更改。我们需要停止相机并使用新参数重新启动 */
                    mSurfaceExist = false;
                    checkCurrentState();
                    /* 现在使用新表面 */
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
        private final Mat rgba;
        private final Mat gray;
        XCameraFrame(Mat rgba, Mat gray) {
            this.rgba = rgba;
            this.gray = gray;
        }
        @Override
        public Mat rgba() { return rgba; }
        @Override
        public Mat gray() { return gray; }
    }
}
