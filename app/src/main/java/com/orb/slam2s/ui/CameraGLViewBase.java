package com.orb.slam2s.ui;

import android.content.Context;
import android.graphics.Bitmap;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;

import com.orb.slam2s.rendering.gles.GLRootView;

/**
 * 这是一个基础类，实现与相机和OpenCV库的交互。
 * 它的主要职责是控制何时可以启用相机，处理帧，
 * 调用外部监听器对帧进行任何调整，然后将结果帧绘制到屏幕上。
 * 客户端应实现CvCameraViewListener。
 */

public abstract class CameraGLViewBase extends GLRootView{

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
        /**
         * 当相机预览启动时调用此方法。
         * @param width - 将传递的帧的宽度
         * @param height - 将传递的帧的高度
         */
        public void onCameraViewStarted(int width, int height);

        /**
         * 当相机预览因某种原因停止时调用此方法。
         */
        public void onCameraViewStopped();

        /**
         * 每帧回调（宿主用于 FPS 等统计；帧内容由 CameraGLView 通过 XCameraFrame 提供）。
         */
        public long onCameraFrame(CvCameraViewFrame inputFrame);
    };

    /**
     * 相机单帧的抽象表示（CameraX 路径无原生 Mat 地址，rgba()/gray() 返回 0，仅作回调载体）。
     */
    public interface CvCameraViewFrame {
        public long rgba();
        public long gray();
    };

    /**
     * 此方法提供给客户端，以便他们可以启用相机连接。
     * 实际的onCameraViewStarted回调只有在此方法被调用且surface可用后才会传递
     */
    public void enableView() {
        synchronized(mSyncObject) {
            mEnabled = true;
            checkCurrentState();
        }
    }

    /**
     * 此方法提供给客户端，以便他们可以禁用相机连接并停止
     * 传递帧，即使surface视图本身未被销毁且仍留在屏幕上
     */
    public void disableView() {
        synchronized(mSyncObject) {
            mEnabled = false;
            checkCurrentState();
        }
    }

    /**
     *
     * @param listener
     */

    public void setCvCameraViewListener(CvCameraViewListener2 listener) {
        mListener = listener;
    }

    /**
     * 当子类处理完一帧后调用，把帧交给宿主回调（onCameraFrame），用于 FPS 等统计。
     * @param frame - 要传递的当前帧
     */
    protected void deliverAndDrawFrame(CvCameraViewFrame frame) {
        if (mListener != null) {
            mListener.onCameraFrame(frame);
        }
    }

    /**
     * 当持有mSyncObject锁时调用
     */
    protected void checkCurrentState() {
        //Log.d(TAG, "调用checkCurrentState");
        int targetState;

        if (mEnabled && mSurfaceExist && getVisibility() == View.VISIBLE) {
            targetState = STARTED;
        } else {
            targetState = STOPPED;
        }

        if (targetState != mState) {
            /* 检测到状态变化。需要退出当前状态并进入目标状态 */
            processExitState(mState);
            mState = targetState;
            processEnterState(mState);
        }
    }

    private void processEnterState(int state) {
        //Log.d(TAG, "调用processEnterState: " + state);
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
        //Log.d(TAG, "调用processExitState: " + state);
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
        /* 无需操作 */
    }

    private void onExitStoppedState() {
        /* 无需操作 */
    }

    // 注意：在Android 4.1.x上，bitmap构造函数和相机连接的顺序很重要
    // Bitmap必须在surface之前构造
    private void onEnterStartedState() {
        //Log.d(TAG, "调用onEnterStartedState");
        /* 连接相机 */
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

    /**
     * 调用此方法应执行具体操作以初始化相机。
     * 约定：作为此方法的结果，变量mFrameWidth和mFrameHeight必须
     * 使用将传递给外部处理器的相机帧大小进行初始化。
     * @param width - 此SurfaceView的宽度
     * @param height - 此SurfaceView的高度
     */
    protected abstract boolean connectCamera(int width, int height);

    /**
     * 断开连接并释放连接到此表面视图的特定相机对象。
     * 当持有syncObject锁时调用
     */
    protected abstract void disconnectCamera();

    // 注意：在Android 4.1.x上，必须在SurfaceTexture构造函数之前调用该函数！
    protected void AllocateCache()
    {
        mCacheBitmap = Bitmap.createBitmap(mFrameWidth, mFrameHeight, Bitmap.Config.ARGB_8888);
    }
}