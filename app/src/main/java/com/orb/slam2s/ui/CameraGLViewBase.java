package com.orb.slam2s.ui;

import android.content.Context;
import android.graphics.Bitmap;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;

import com.orb.slam2s.rendering.gles.GLRootView;
import com.orb.slam2s.utils.TextureUtils;

import com.orb.slam2s.slamar.OpenCVBridge;

// 相机与 OpenCV 交互的基础类：控制相机启停、处理帧并回调监听器，将结果绘制到屏幕
// 客户端应实现 CvCameraViewListener
public abstract class CameraGLViewBase extends GLRootView{

    private static final String TAG = "CameraGLViewBase";
    private static final int STOPPED = 0;
    private static final int STARTED = 1;

    private int mState = STOPPED;
    private Bitmap mCacheBitmap;
    private CvCameraViewListener2 mListener;
    protected boolean mSurfaceExist;
    protected final Object mSyncObject = new Object();

    protected int mFrameWidth;
    protected int mFrameHeight;
    protected boolean mEnabled;

    protected int imageTextureId;
    public CameraGLViewBase(Context context) {
        super(context);
    }

    public CameraGLViewBase(Context context, AttributeSet attrs) {
        super(context, attrs);

        int count = attrs.getAttributeCount();
        Log.d(TAG, "属性计数: " + Integer.valueOf(count));
    }

    public interface CvCameraViewListener2 {
        // 相机预览启动时调用，此后帧将通过 onCameraFrame() 回调传递给客户端
        public void onCameraViewStarted(int width, int height);

        // 相机预览停止时调用，此后不再传递帧
        public void onCameraViewStopped();

        // 帧传递回调，返回修改后的帧用于屏幕显示
        // TODO: 传递指定帧格式的参数（BPP、YUV或RGB等）
        public long onCameraFrame(CvCameraViewFrame inputFrame);
    };

    // 相机单帧的抽象表示，用于 onCameraFrame 回调
    // 注意：不要在该回调之外使用此接口对象
    public interface CvCameraViewFrame {

        // 返回带帧的 RGBA native Mat 地址
        public long rgba();

        // 返回带帧的单通道灰度 native Mat 地址
        public long gray();
    };

    // 启用相机连接，surface 可用后才会传递 onCameraViewStarted 回调
    public void enableView() {
        synchronized(mSyncObject) {
            mEnabled = true;
            checkCurrentState();
        }
    }

    // 禁用相机连接并停止传递帧，即使 surface 视图仍在屏幕上
    public void disableView() {
        synchronized(mSyncObject) {
            mEnabled = false;
            checkCurrentState();
        }
    }

    // 设置相机帧回调监听器
    public void setCvCameraViewListener(CvCameraViewListener2 listener) {
        mListener = listener;
    }

    // 当持有 mSyncObject 锁时调用
    protected void checkCurrentState() {
        Log.d(TAG, "调用checkCurrentState");
        int targetState;

        if (mEnabled && mSurfaceExist && getVisibility() == View.VISIBLE) {
            targetState = STARTED;
        } else {
            targetState = STOPPED;
        }

        if (targetState != mState) {
            // 检测到状态变化，需退出当前状态并进入目标状态
            processExitState(mState);
            mState = targetState;
            processEnterState(mState);
        }
    }

    private void processEnterState(int state) {
        Log.d(TAG, "调用processEnterState: " + state);
        switch(state) {
        case STARTED:
            onEnterStartedState();
            if (mListener != null) {
                mListener.onCameraViewStarted(mFrameWidth, mFrameHeight);
            }
            break;
        case STOPPED:
            onEnterStoppedState();
            if (mListener != null) {
                mListener.onCameraViewStopped();
            }
            break;
        };
    }

    private void processExitState(int state) {
        Log.d(TAG, "调用processExitState: " + state);
        switch(state) {
        case STARTED:
            onExitStartedState();
            break;
        case STOPPED:
            onExitStoppedState();
            break;
        };
    }

    private void onEnterStoppedState() {
        // 无需操作
    }

    private void onExitStoppedState() {
        // 无需操作
    }

    // 注意：在Android 4.1.x上，bitmap构造函数和相机连接的顺序很重要
    // Bitmap必须在surface之前构造
    private void onEnterStartedState() {
        Log.d(TAG, "调用onEnterStartedState");
        // 连接相机
        if (!connectCamera(getWidth(), getHeight())) {
            Log.d(TAG, "onEnterStartedState: 连接相机失败。");
        }
    }

    private void onExitStartedState() {
        disconnectCamera();
        if (mCacheBitmap != null) {
            mCacheBitmap.recycle();
        }
    }

    // 将有效帧通过回调传递给外部客户端并显示在屏幕上
    protected void deliverAndDrawFrame(CvCameraViewFrame frame) {
        long modifiedAddr;

        if (mListener != null) {
            modifiedAddr = mListener.onCameraFrame(frame);
        } else {
            modifiedAddr = frame.rgba();
        }

        boolean bmpValid = true;
        if (modifiedAddr != 0) {
            synchronized (mSyncObject) {
                if (mCacheBitmap != null && !mCacheBitmap.isRecycled()) {
                    try {
                        // 通过 JNI 将 native Mat 转为 Bitmap（比 Utils.matToBitmap 更快）
                        OpenCVBridge.nativeMatToBitmap(modifiedAddr, mCacheBitmap);
                    } catch(Exception e) {
                        Log.e(TAG, "nativeMatToBitmap抛出异常: " + e.getMessage());
                        bmpValid = false;
                    }
                } else {
                    bmpValid = false;
                }
            }
        }

        if (bmpValid && mCacheBitmap != null) {
            // 将 mCacheBitmap 发送到纹理
            queueEvent(new Runnable() {
                @Override
                public void run() {
                    synchronized (mSyncObject) {
                        if (mCacheBitmap != null && !mCacheBitmap.isRecycled()) {
                            TextureUtils.loadTexture(mCacheBitmap, imageTextureId);
                        }
                    }
                }
            });
        }

        //Martin: 使用画布绘制位图大约需要40-50毫秒

    }

    // 初始化相机，并须将 mFrameWidth 与 mFrameHeight 设为相机帧大小
    protected abstract boolean connectCamera(int width, int height);

    // 断开并释放相机对象，当持有 syncObject 锁时调用
    protected abstract void disconnectCamera();

    // 注意：在Android 4.1.x上，必须在SurfaceTexture构造函数之前调用该函数！
    protected void AllocateCache()
    {
        mCacheBitmap = Bitmap.createBitmap(mFrameWidth, mFrameHeight, Bitmap.Config.ARGB_8888);
    }

}