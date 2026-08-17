package com.orb.slam2s.ui;

import android.content.Context;
import android.graphics.Bitmap;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;

import com.orb.slam2s.rendering.gles.GLRootView;

// 相机与 SDK 交互的基础类：控制相机启停、处理帧并回调监听器，将结果绘制到屏幕
public abstract class CameraGLViewBase extends GLRootView {

    private static final String TAG = "CameraGLViewBase";
    private static final int STOPPED = 0;
    private static final int STARTED = 1;

    private int mState = STOPPED;
    protected Bitmap mCacheBitmap;
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
    }

    public interface CvCameraViewListener2 {
        // 相机预览启动时调用，width/height 为传递的帧宽高
        public void onCameraViewStarted(int width, int height);

        // 相机预览因某种原因停止时调用
        public void onCameraViewStopped();

        // 每帧回调（宿主用于 FPS 等统计；帧内容由 CameraGLView 通过 XCameraFrame 提供）
        public long onCameraFrame(CvCameraViewFrame inputFrame);
    }

    // 相机单帧的抽象表示（CameraX 路径无原生 Mat 地址，rgba()/gray() 返回 0，仅作回调载体）
    public interface CvCameraViewFrame {
        public long rgba();
        public long gray();
    }

    // 启用相机连接，surface 可用后才会传递 onCameraViewStarted 回调
    public void enableView() {
        synchronized (mSyncObject) {
            mEnabled = true;
            checkCurrentState();
        }
    }

    // 禁用相机连接并停止传递帧，即使 surface 视图仍在屏幕上
    public void disableView() {
        synchronized (mSyncObject) {
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
        int targetState;

        if (mEnabled && mSurfaceExist && getVisibility() == View.VISIBLE) {
            targetState = STARTED;
        } else {
            targetState = STOPPED;
        }

        if (targetState != mState) {
            processExitState(mState);
            mState = targetState;
            processEnterState(mState);
        }
    }

    private void processEnterState(int state) {
        switch (state) {
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
        }
    }

    private void processExitState(int state) {
        switch (state) {
            case STARTED:
                onExitStartedState();
                break;
            case STOPPED:
                onExitStoppedState();
                break;
        }
    }

    private void onEnterStoppedState() {
    }

    private void onExitStoppedState() {
    }

    private void onEnterStartedState() {
        if (!connectCamera(getWidth(), getHeight())) {
            Log.e(TAG, "onEnterStartedState: 连接相机失败。");
        }
    }

    private void onExitStartedState() {
        disconnectCamera();
        if (mCacheBitmap != null) {
            mCacheBitmap.recycle();
        }
    }

    // 子类处理完一帧后调用，把帧交给宿主回调（onCameraFrame），用于 FPS 等统计
    protected void deliverAndDrawFrame(CvCameraViewFrame frame) {
        if (mListener != null) {
            mListener.onCameraFrame(frame);
        }
    }

    // 初始化相机，并须将 mFrameWidth 与 mFrameHeight 设为相机帧大小
    protected abstract boolean connectCamera(int width, int height);

    // 断开并释放相机对象，当持有 syncObject 锁时调用
    protected abstract void disconnectCamera();

    protected void AllocateCache() {
        mCacheBitmap = Bitmap.createBitmap(mFrameWidth, mFrameHeight, Bitmap.Config.ARGB_8888);
    }
}