package com.orb.slam2s.ui;

import android.content.Context;
import android.graphics.Bitmap;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;

import com.orb.slam2s.constant.GlobalConstant;
import com.orb.slam2s.rendering.gles.GLRootView;
import com.orb.slam2s.utils.TextureUtils;

import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Size;

import java.util.List;

/**
 * 这是一个基础类，实现与相机和OpenCV库的交互。
 * 它的主要职责是控制何时可以启用相机，处理帧，
 * 调用外部监听器对帧进行任何调整，然后将结果帧绘制到屏幕上。
 * 客户端应实现CvCameraViewListener。
 */

public abstract class CameraGLViewBase extends GLRootView{

    private static final String TAG = "CameraGLViewBase";
    private static final int MAX_UNSPECIFIED = -1;
    private static final int STOPPED = 0;
    private static final int STARTED = 1;

    private int mState = STOPPED;
    private Bitmap mCacheBitmap;
    private CvCameraViewListener2 mListener;
    protected boolean mSurfaceExist;
    protected Object mSyncObject = new Object();

    protected int mFrameWidth;
    protected int mFrameHeight;
    protected int mMaxHeight;
    protected int mMaxWidth;
    protected int mPreviewFormat = RGBA;
    protected boolean mEnabled;

    public static final int RGBA = 1;
    public static final int GRAY = 2;

    protected int imageTextureId;
    public CameraGLViewBase(Context context) {
        super(context);
        mMaxWidth = MAX_UNSPECIFIED;
        mMaxHeight = MAX_UNSPECIFIED;
    }

    public CameraGLViewBase(Context context, AttributeSet attrs) {
        super(context, attrs);

        int count = attrs.getAttributeCount();
        Log.d(TAG, "属性计数: " + Integer.valueOf(count));

        mMaxWidth = MAX_UNSPECIFIED;
        mMaxHeight = MAX_UNSPECIFIED;
    }
    
    public interface CvCameraViewListener {
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
        public Mat onCameraFrame(Mat inputFrame);
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
        public Mat onCameraFrame(CvCameraViewFrame inputFrame);
    };

    protected class CvCameraViewListenerAdapter implements CvCameraViewListener2  {
        public CvCameraViewListenerAdapter(CvCameraViewListener oldStypeListener) {
            mOldStyleListener = oldStypeListener;
        }

        public void onCameraViewStarted(int width, int height) {
            mOldStyleListener.onCameraViewStarted(width, height);
        }

        public void onCameraViewStopped() {
            mOldStyleListener.onCameraViewStopped();
        }

        public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
             Mat result = null;
             switch (mPreviewFormat) {
                case RGBA:
                    result = mOldStyleListener.onCameraFrame(inputFrame.rgba());
                    break;
                case GRAY:
                    result = mOldStyleListener.onCameraFrame(inputFrame.gray());
                    break;
                default:
                    Log.e(TAG, "无效的帧格式！仅支持RGBA和灰度格式！");
            };

            return result;
        }

        public void setFrameFormat(int format) {
            mPreviewFormat = format;
        }

        private int mPreviewFormat = RGBA;
        private CvCameraViewListener mOldStyleListener;
    };

    /**
     * 此类接口是相机单帧的抽象表示，用于onCameraFrame回调
     * 注意：不要在onCameraFrame回调之外使用表示此接口的对象！
     */
    public interface CvCameraViewFrame {

        /**
         * 此方法返回带有帧的RGBA Mat
         */
        public Mat rgba();

        /**
         * 此方法返回带有帧的单通道灰度Mat
         */
        public Mat gray();
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
     * 此方法设置允许的相机帧的最大大小。在选择大小时，
     * 将选择小于或等于设定大小的最大尺寸。
     * 例如 - 我们设置setMaxFrameSize(200,200)，我们有176x152和320x240尺寸。
     * 预览帧将选择176x152尺寸。
     * 当需要出于某种原因限制预览帧大小时，此方法很有用(例如用于视频录制)
     * @param maxWidth - 相机帧允许的最大宽度
     * @param maxHeight - 相机帧允许的最大高度
     */
    public void setMaxFrameSize(int maxWidth, int maxHeight) {
        mMaxWidth = maxWidth;
        mMaxHeight = maxHeight;
    }

    public void SetCaptureFormat(int format)
    {
        mPreviewFormat = format;
        if (mListener instanceof CvCameraViewListenerAdapter) {
            CvCameraViewListenerAdapter adapter = (CvCameraViewListenerAdapter) mListener;
            adapter.setFrameFormat(mPreviewFormat);
        }
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
        Mat modified;

        if (mListener != null) {
            modified = mListener.onCameraFrame(frame);
        } else {
            modified = frame.rgba();
        }

        boolean bmpValid = true;
        if (modified != null) {
            synchronized (mSyncObject) {
                if (mCacheBitmap != null && !mCacheBitmap.isRecycled()) {
                    try {
                        //这很快。
                        Utils.matToBitmap(modified, mCacheBitmap);
                    } catch(Exception e) {
                        Log.e(TAG, "Mat类型: " + modified);
                        Log.e(TAG, "Bitmap类型: " + mCacheBitmap.getWidth() + "*" + mCacheBitmap.getHeight());
                        Log.e(TAG, "Utils.matToBitmap()抛出异常: " + e.getMessage());
                        bmpValid = false;
                    }
                } else {
                    bmpValid = false;
                }
            }
        }

        //Log.d("JNI_", "转换完成");
        if (bmpValid && mCacheBitmap != null) {
            //将mCacheBitmap发送到纹理。

            queueEvent(new Runnable() {
                @Override
                public void run() {
                    synchronized (mSyncObject) {
                        if (mCacheBitmap != null && !mCacheBitmap.isRecycled()) {
                            //Log.d("JNI_", "发送图像：textureId "+imageTextureId);
                            TextureUtils.loadTexture(mCacheBitmap, imageTextureId);
                        }
                    }
                }
            });
        }

        //Martin: 使用画布绘制位图大约需要40-50毫秒
        //Log.d("JNI_", "绘制完成");

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

    public interface ListItemAccessor {
        public int getWidth(Object obj);
        public int getHeight(Object obj);
    };

    /**
     * 子类可以调用此辅助方法来选择相机预览大小。
     * 它会遍历支持的预览大小列表，并选择最大的一个，
     * 该大小同时适合通过setMaxFrameSize()设置的值和为此视图分配的表面帧
     * @param supportedSizes
     * @param surfaceWidth
     * @param surfaceHeight
     * @return 最佳帧大小
     */
    protected Size calculateCameraFrameSize(List<?> supportedSizes, ListItemAccessor accessor, int surfaceWidth, int surfaceHeight) {
        int calcWidth = 0;
        int calcHeight = 0;

        int maxAllowedWidth = (mMaxWidth != MAX_UNSPECIFIED && mMaxWidth < surfaceWidth)? mMaxWidth : surfaceWidth;
        int maxAllowedHeight = (mMaxHeight != MAX_UNSPECIFIED && mMaxHeight < surfaceHeight)? mMaxHeight : surfaceHeight;

        for (Object size : supportedSizes) {
            int width = accessor.getWidth(size);
            int height = accessor.getHeight(size);

            if (width <= maxAllowedWidth && height <= maxAllowedHeight) {
                if (width >= calcWidth && height >= calcHeight) {
                    calcWidth = (int) width;
                    calcHeight = (int) height;
                }
            }
        }
        return new Size(GlobalConstant.RESOLUTION_WIDTH,GlobalConstant.RESOLUTION_HEIGHT);
        //return new Size(计算宽度, 计算高度);
    }
}
