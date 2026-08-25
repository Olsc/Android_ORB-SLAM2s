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
package com.orb.slam2s.camera;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.PixelFormat;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.opengl.Matrix;
import android.os.Handler;
import android.os.Looper;
import android.util.AttributeSet;
import android.util.Log;
import android.util.Size;
import android.view.SurfaceHolder;
import android.view.View;

import androidx.annotation.NonNull;
import androidx.camera.core.Camera;
import androidx.camera.core.CameraSelector;
import androidx.camera.core.FocusMeteringAction;
import androidx.camera.core.ImageAnalysis;
import androidx.camera.core.ImageProxy;
import androidx.camera.core.MeteringPoint;
import androidx.camera.core.MeteringPointFactory;
import androidx.camera.core.SurfaceOrientedMeteringPointFactory;
import androidx.camera.lifecycle.ProcessCameraProvider;
import androidx.core.content.ContextCompat;
import androidx.lifecycle.LifecycleOwner;

import com.google.common.util.concurrent.ListenableFuture;
import com.orb.slam2s.constant.GlobalConstant;
import com.orb.slam2s.device.DeviceCompat;
import com.orb.slam2s.graphics.AspectGLSurfaceView;
import com.orb.slam2s.graphics.GLPassThroughRenderer;
import com.orb.slam2s.graphics.GLPointCloudRenderer;
import com.orb.slam2s.graphics.GLUtils;
import com.orb.slam2s.ipc.SharedMemoryBuffer;
import com.orb.slam2s.ipc.SlamIPCClient;

import java.nio.ByteBuffer;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

// 相机预览与图像流摄取视图（基于 CameraX 采集、双缓冲灰度转换、IPC 帧推送与 OpenGL 视图呈现）
public class CameraPreviewView extends AspectGLSurfaceView {

    private static final String TAG = "CameraPreviewView";

    private static final int STATE_STOPPED = 0;
    private static final int STATE_STARTED = 1;

    private int mState = STATE_STOPPED;
    private boolean mSurfaceExist;
    private boolean mEnabled;
    private final Object mSyncLock = new Object();

    private int mFrameWidth;
    private int mFrameHeight;
    private Bitmap mCacheBitmap;
    private int mImageTextureId;

    private ProcessCameraProvider mCameraProvider;
    private Camera mCameraX;
    private ImageAnalysis mImageAnalysis;
    private ExecutorService mAnalyzerExecutor;
    private ExecutorService mIpcSendExecutor;

    private byte[][] mYuvSendBuffers; // 灰度帧发送双缓冲（复用避免 GC）
    private byte[][] mRgbaBuffers;    // 相机 RGBA 帧双缓冲
    private int mSendPing;            // 当前发送缓冲索引 (0/1)
    private int[] mCachePixels;       // 预览位图像素数组

    private GLPassThroughRenderer mPassThroughRenderer;
    private GLPointCloudRenderer mPointCloudRenderer;

    // 点云读取缓冲（共享内存直接读取，零 Binder）
    private final float[] mPointCloudBuffer = new float[SharedMemoryBuffer.POINTCLOUD_MAX_BYTES / 4];
    private final float[] mTempMvp = new float[48];
    private final float[] mVPMatrix = new float[16];

    // SLAM(RDF: 右-下-前) → GL(RUB: 右-上-后) 坐标系变换矩阵
    private static final float[] RDF_TO_RUB = {
            1,  0,  0, 0,
            0, -1,  0, 0,
            0,  0, -1, 0,
            0,  0,  0, 1
    };

    private final Context mContext;
    private SlamIPCClient mSlamIPCClient;
    private volatile boolean mPendingDetectPlane;
    private final AtomicBoolean mIsIpcProcessing = new AtomicBoolean(false);

    private FrameListener mFrameListener;

    public interface FrameListener {
        void onCameraStarted(int width, int height);
        void onCameraStopped();
        void onCameraFrame();
    }

    public CameraPreviewView(Context context) {
        this(context, null);
    }

    public CameraPreviewView(Context context, AttributeSet attrs) {
        super(context, attrs);
        this.mContext = context;
    }

    public void setSlamIPCClient(SlamIPCClient client) {
        this.mSlamIPCClient = client;
    }

    public void setFrameListener(FrameListener listener) {
        this.mFrameListener = listener;
    }

    public void requestPlaneDetection() {
        mPendingDetectPlane = true;
    }

    public void init() {
        setAspectRatio(GlobalConstant.RESOLUTION_WIDTH, GlobalConstant.RESOLUTION_HEIGHT);
        setEGLContextClientVersion(2);
        setEGLConfigChooser(8, 8, 8, 8, 16, 0);
        getHolder().setFormat(PixelFormat.TRANSLUCENT);
        setZOrderOnTop(false);

        setRenderer(new CameraRenderer());
        setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
        setPreserveEGLContextOnPause(true);

        mPassThroughRenderer = new GLPassThroughRenderer(mContext);
    }

    public void enableView() {
        synchronized (mSyncLock) {
            mEnabled = true;
            checkCurrentState();
        }
    }

    public void disableView() {
        synchronized (mSyncLock) {
            mEnabled = false;
            checkCurrentState();
        }
    }

    private void checkCurrentState() {
        int targetState;
        if (mEnabled && mSurfaceExist && getVisibility() == View.VISIBLE) {
            targetState = STATE_STARTED;
        } else {
            targetState = STATE_STOPPED;
        }

        if (targetState != mState) {
            if (targetState == STATE_STARTED) {
                connectCamera();
                if (mFrameListener != null) {
                    mFrameListener.onCameraStarted(mFrameWidth, mFrameHeight);
                }
            } else {
                disconnectCamera();
                if (mCacheBitmap != null && !mCacheBitmap.isRecycled()) {
                    mCacheBitmap.recycle();
                }
                if (mFrameListener != null) {
                    mFrameListener.onCameraStopped();
                }
            }
            mState = targetState;
        }
    }

    private void connectCamera() {
        try {
            ListenableFuture<ProcessCameraProvider> future = ProcessCameraProvider.getInstance(getContext());
            future.addListener(() -> {
                try {
                    mCameraProvider = future.get();

                    mFrameWidth = GlobalConstant.RESOLUTION_WIDTH;
                    mFrameHeight = GlobalConstant.RESOLUTION_HEIGHT;
                    mCacheBitmap = Bitmap.createBitmap(mFrameWidth, mFrameHeight, Bitmap.Config.ARGB_8888);

                    mAnalyzerExecutor = Executors.newSingleThreadExecutor();
                    mIpcSendExecutor = Executors.newSingleThreadExecutor();

                    ImageAnalysis.Builder builder = new ImageAnalysis.Builder()
                            .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                            .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
                            .setTargetResolution(new Size(mFrameWidth, mFrameHeight));

                    mImageAnalysis = builder.build();
                    mImageAnalysis.setAnalyzer(mAnalyzerExecutor, new ImageAnalysis.Analyzer() {
                        @Override
                        public void analyze(@NonNull ImageProxy image) {
                            if (mAnalyzerExecutor == null || mAnalyzerExecutor.isShutdown()) {
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
                                ByteBuffer buf = rgbaPlane.getBuffer();
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
                                if (mRgbaBuffers[mSendPing] == null || mRgbaBuffers[mSendPing].length < rgbaSize) {
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
                                if (mYuvSendBuffers[mSendPing] == null || mYuvSendBuffers[mSendPing].length < requiredSize) {
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
                                    DeviceCompat.checkAndFlipFrame(mCachePixels, w, h);
                                    mCacheBitmap.setPixels(mCachePixels, 0, w, 0, 0, w, h);
                                    queueEvent(() -> {
                                        synchronized (mSyncLock) {
                                            if (mCacheBitmap != null && !mCacheBitmap.isRecycled()) {
                                                GLUtils.loadTexture(mCacheBitmap, mImageTextureId);
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

                                DeviceCompat.checkAndFlipFrame(yBuf, w, h);

                                if (mSlamIPCClient != null && mSlamIPCClient.isConnected()) {
                                    if (mIsIpcProcessing.compareAndSet(false, true)) {
                                        final byte[] sendBuffer = yBuf;
                                        mSendPing ^= 1;
                                        final int frameW = w;
                                        final int frameH = h;
                                        if (mIpcSendExecutor != null && !mIpcSendExecutor.isShutdown()) {
                                            mIpcSendExecutor.execute(() -> {
                                                try {
                                                    mSlamIPCClient.sendFrameData(sendBuffer, frameW, frameH);
                                                    if (mPendingDetectPlane) {
                                                        mPendingDetectPlane = false;
                                                        mSlamIPCClient.detectPlane();
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

                                if (mFrameListener != null) {
                                    mFrameListener.onCameraFrame();
                                }

                                image.close();
                            } catch (Throwable e) {
                                Log.e(TAG, "相机帧分析异常: " + e.getMessage());
                                try {
                                    image.close();
                                } catch (Exception ignored) {}
                            }
                        }
                    });

                    CameraSelector selector = new CameraSelector.Builder()
                            .requireLensFacing(CameraSelector.LENS_FACING_BACK)
                            .build();

                    mCameraProvider.unbindAll();
                    mCameraX = mCameraProvider.bindToLifecycle((LifecycleOwner) getContext(), selector, mImageAnalysis);
                } catch (Exception e) {
                    Log.e(TAG, "CameraX 初始化失败: " + e.getMessage());
                }
            }, ContextCompat.getMainExecutor(getContext()));
        } catch (Exception ex) {
            Log.e(TAG, "初始化 CameraX 异常: " + ex.getMessage());
        }
    }

    private void disconnectCamera() {
        if (mCameraProvider != null) {
            final ProcessCameraProvider provider = mCameraProvider;
            new Handler(Looper.getMainLooper()).post(() -> {
                try {
                    provider.unbindAll();
                } catch (Exception e) {
                    Log.e(TAG, "unbindAll error: " + e.getMessage());
                }
            });
        }
        if (mAnalyzerExecutor != null) {
            mAnalyzerExecutor.shutdown();
            mAnalyzerExecutor = null;
        }
        if (mIpcSendExecutor != null) {
            mIpcSendExecutor.shutdown();
            mIpcSendExecutor = null;
        }
    }

    public void autoFocusCenter() {
        try {
            if (mCameraX == null) return;
            MeteringPointFactory factory = new SurfaceOrientedMeteringPointFactory(getWidth(), getHeight());
            MeteringPoint point = factory.createPoint(getWidth() / 2f, getHeight() / 2f);
            FocusMeteringAction action = new FocusMeteringAction.Builder(point)
                    .setAutoCancelDuration(3, TimeUnit.SECONDS)
                    .build();
            mCameraX.getCameraControl().startFocusAndMetering(action);
        } catch (Exception e) {
            Log.e(TAG, "居中自动对焦错误: " + e.getMessage());
        }
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        super.surfaceDestroyed(holder);
        synchronized (mSyncLock) {
            mSurfaceExist = false;
            checkCurrentState();
            if (mPassThroughRenderer != null) {
                mPassThroughRenderer.destroy();
            }
            if (mPointCloudRenderer != null) {
                mPointCloudRenderer.destroy();
                mPointCloudRenderer = null;
            }
        }
    }

    private class CameraRenderer implements GLSurfaceView.Renderer {
        @Override
        public void onSurfaceCreated(GL10 gl, EGLConfig config) {
            Bitmap bitmap = Bitmap.createBitmap(GlobalConstant.RESOLUTION_WIDTH, GlobalConstant.RESOLUTION_HEIGHT, Bitmap.Config.ARGB_8888);
            mImageTextureId = GLUtils.loadTexture(bitmap, 0);
            bitmap.recycle();

            mPassThroughRenderer.init();

            mPointCloudRenderer = new GLPointCloudRenderer();
            mPointCloudRenderer.init();
        }

        @Override
        public void onSurfaceChanged(GL10 gl, int width, int height) {
            mPassThroughRenderer.onSurfaceChanged(width, height);
            synchronized (mSyncLock) {
                mSurfaceExist = true;
                checkCurrentState();
            }
        }

        @Override
        public void onDrawFrame(GL10 gl) {
            GLES20.glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
            GLES20.glClear(GLES20.GL_DEPTH_BUFFER_BIT | GLES20.GL_COLOR_BUFFER_BIT);

            mPassThroughRenderer.onDrawFrame(mImageTextureId);

            if (mSlamIPCClient != null && mSlamIPCClient.isConnected() && mPointCloudRenderer != null) {
                int floats = mSlamIPCClient.readPointCloud(mPointCloudBuffer, mPointCloudBuffer.length);
                if (floats > 0 && mSlamIPCClient.readMvp(mTempMvp)) {
                    // 点云在相机坐标系 (RDF)，直接由 ProjectionMatrix * RDF_TO_RUB 投影
                    Matrix.multiplyMM(mVPMatrix, 0, mTempMvp, 32, RDF_TO_RUB, 0);
                    mPointCloudRenderer.updatePoints(mPointCloudBuffer, floats);
                    GLES20.glDisable(GLES20.GL_DEPTH_TEST);
                    mPointCloudRenderer.draw(mVPMatrix);
                    GLES20.glEnable(GLES20.GL_DEPTH_TEST);
                }
            }
        }
    }
}