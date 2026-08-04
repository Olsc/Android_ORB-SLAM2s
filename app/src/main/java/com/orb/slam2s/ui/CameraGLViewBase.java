package com.orb.slam2s.ui;

import android.content.Context;
import android.graphics.Bitmap;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;

import com.orb.slam2s.rendering.gles.GLRootView;
import com.orb.slam2s.utils.TextureUtils;

import com.orb.slam2s.slamar.OpenCVBridge;

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
        /**
         * 当相机预览启动时调用此方法。在此方法调用后，帧将通过onCameraFrame()回调传递给客户端。
         * @param width - 将传递的帧的宽度
         * @param height - 将传递的帧的高度
         */
        public void onCameraViewStarted(int width, int height);

        /**
         * 当相机预览因某种原因停止时调用此方法。
         * 在调用此方法后，将不会通过onCameraFrame()回调传递帧。
         */
        public void onCameraViewStopped();

        /**
         * 当需要传递帧时调用此方法。
         * 返回值是一个修改后的帧，需要显示在屏幕上。
         * TODO: 传递指定帧格式的参数(BPP, YUV或RGB等)
         */
        public long onCameraFrame(CvCameraViewFrame inputFrame);
    };

    /**
     * 此类接口是相机单帧的抽象表示，用于onCameraFrame回调
     * 注意：不要在onCameraFrame回调之外使用表示此接口的对象！
     */
    public interface CvCameraViewFrame {

        /**
         * 此方法返回带有帧的RGBA native Mat 地址
         */
        public long rgba();

        /**
         * 此方法返回带有帧的单通道灰度 native Mat 地址
         */
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
     * 当持有mSyncObject锁时调用
     */
    protected void checkCurrentState() {
        Log.d(TAG, "调用checkCurrentState");
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
        /* 无需操作 */
    }

    private void onExitStoppedState() {
        /* 无需操作 */
    }

    // 注意：在Android 4.1.x上，bitmap构造函数和相机连接的顺序很重要
    // Bitmap必须在surface之前构造
    private void onEnterStartedState() {
        Log.d(TAG, "调用onEnterStartedState");
        /* 连接相机 */
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

    /**
     * 当子类具有有效对象并希望将其传递给外部客户端(通过回调)并
     * 然后显示在屏幕上时，应调用此方法。
     * @param frame - 要传递的当前帧
     */
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
            //将mCacheBitmap发送到纹理。

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