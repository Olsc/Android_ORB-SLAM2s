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
package com.orb.slam2s.ui;

import android.app.AlertDialog;
import android.content.ComponentCallbacks2;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.graphics.Point;
import android.graphics.Rect;
import android.opengl.GLSurfaceView;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.text.format.Formatter;
import android.util.Log;
import android.view.Display;
import android.hardware.display.DisplayManager;
import android.view.MotionEvent;
import android.view.Surface;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ProgressBar;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;

import android.view.ViewGroup;

import androidx.activity.ComponentActivity;
import androidx.activity.OnBackPressedCallback;
import androidx.annotation.NonNull;
import androidx.annotation.OptIn;
import androidx.camera.camera2.interop.ExperimentalCamera2Interop;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowCompat;
import androidx.core.view.WindowInsetsCompat;
import androidx.core.view.WindowInsetsControllerCompat;

import com.orb.slam2s.R;
import com.orb.slam2s.camera.CameraPreviewView;
import com.orb.slam2s.constant.GlobalConstant;
import com.orb.slam2s.graphics.AspectGLSurfaceView;
import com.orb.slam2s.graphics.DisplayCorrection;
import com.orb.slam2s.graphics.GlbModelRenderer;
import com.orb.slam2s.ipc.SlamIPCClient;
import com.orb.slam2s.sensors.OrientationSensor;
import com.orb.slam2s.util.FpsCalculator;
import com.orb.slam2s.util.MapManager;
import com.orb.slam2s.util.SensorFusionTracker;
import com.orb.slam2s.util.TouchGestureHelper;

import androidx.camera.camera2.interop.Camera2CameraInfo;
import androidx.camera.core.CameraInfo;
import androidx.camera.core.CameraSelector;

import java.text.SimpleDateFormat;
import java.io.File;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

// SLAM AR 主控制 Activity（调度相机、3D 模型渲染、3DOF 追踪、地图管理与交互控制）
public class MainActivity extends ComponentActivity
        implements CameraPreviewView.FrameListener, CameraPreviewView.DisplayCorrectionListener {

    private static final String TAG = "MainActivity";

    private CameraPreviewView mCameraPreviewView;
    private AspectGLSurfaceView mArObjectView;
    private boolean mIsInitFinished = false;

    private SlamIPCClient mSlamIPCClient;
    private MapManager mMapManager;
    private TouchGestureHelper mTouchHelper;
    private GlbModelRenderer mModelRenderer;

    private FpsCalculator mFpsCalculator;
    private TextView mFpsTextView;
    private TextView mTextMapStats;
    private Button mBtnCreateArObject;
    private Button mBtnSaveMap;
    private Button mBtnLoadMap;
    private Button mBtnMapList;
    private Button mBtnTogglePointCloud;
    private Button mBtnSwitchCamera;
    private Button mBtnMirrorVertical;
    private Button mBtnMirrorHorizontal;
    private Button mBtn3DofCube;

    private final Handler mUiHandler = new Handler(Looper.getMainLooper());
    private AlertDialog mLoadingDialog;

    // 地图统计轮询（1Hz）：仅在 onResume~onPause 之间运行，避免后台无谓唤醒与 IPC 占用
    private final Runnable mMapStatsUpdater = new Runnable() {
        @Override
        public void run() {
            if (mSlamIPCClient != null && mTextMapStats != null) {
                int[] stats = mSlamIPCClient.getMapStats();
                if (stats != null && stats.length == 3) {
                    final String statsText = getString(R.string.map_stats_format,
                            stats[0], stats[1], stats[2] > 0 ? getString(R.string.map_stats_plane_yes)
                                    : getString(R.string.map_stats_plane_no));
                    runOnUiThread(() -> mTextMapStats.setText(statsText));
                }
            }
            mUiHandler.postDelayed(this, 1000);
        }
    };

    private static final String KEY_3DOF_MODE = "is_3dof_mode";

    // 虚拟摇杆
    private VirtualJoystickView mJoystickView;

    // 3DOF 传感器辅助定位跟踪 (默认开启，若检测到硬件)
    private OrientationSensor mOrientationSensor;
    private SensorFusionTracker mSensorFusionTracker;
    private boolean mIs3DofMode = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.d(TAG, "onCreate: 初始化 SLAM 主界面");

        lockCurrentOrientation();
        computeScreenResolution();

        setContentView(R.layout.ar_ui_content);
        initViewsAndServices();

        // 恢复进程被回收前的用户状态（如 3DOF 模式）
        if (savedInstanceState != null && savedInstanceState.containsKey(KEY_3DOF_MODE)) {
            boolean savedMode = savedInstanceState.getBoolean(KEY_3DOF_MODE, true);
            if (savedMode != mIs3DofMode) {
                set3DofMode(savedMode, false);
            }
        }

        getOnBackPressedDispatcher().addCallback(this, new OnBackPressedCallback(true) {
            @Override
            public void handleOnBackPressed() {
                showExitConfirmDialog();
            }
        });
    }

    private void showExitConfirmDialog() {
        if (isFinishing() || isDestroyed()) return;
        new AlertDialog.Builder(this)
                .setTitle(R.string.dialog_exit_title)
                .setMessage(R.string.dialog_exit_message)
                .setPositiveButton(R.string.action_exit, (dialog, which) -> {
                    Log.d(TAG, "用户确认退出应用");
                    finish();
                })
                .setNegativeButton(R.string.button_cancel, null)
                .show();
    }

    @SuppressWarnings("deprecation")
    private void computeScreenResolution() {
        WindowManager wm = (WindowManager) getSystemService(Context.WINDOW_SERVICE);
        if (wm == null) return;

        Point size = new Point();
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            Rect bounds = wm.getCurrentWindowMetrics().getBounds();
            size.x = bounds.width();
            size.y = bounds.height();
        } else {
            wm.getDefaultDisplay().getRealSize(size);
        }

        int screenWidth = size.x;
        int screenHeight = size.y;
        Log.d(TAG, "屏幕分辨率: " + screenWidth + "x" + screenHeight);

        GlobalConstant.computeOptimalResolution(screenWidth, screenHeight);
        Log.d(TAG, "选择相机分辨率: " + GlobalConstant.RESOLUTION_WIDTH + "x" + GlobalConstant.RESOLUTION_HEIGHT);
    }

    private File getSlamDir() {
        File dir = getExternalFilesDir("SLAM");
        return dir != null ? dir : new File(getFilesDir(), "SLAM");
    }

    // 读取当前显示方向，取值对应 Surface.ROTATION_0/90/180/270
    @SuppressWarnings("deprecation")
    private int readDisplayRotation() {
        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
                Display display = getDisplay();
                if (display != null) return display.getRotation();
            }
            WindowManager wm = (WindowManager) getSystemService(Context.WINDOW_SERVICE);
            if (wm != null) return wm.getDefaultDisplay().getRotation();
        } catch (Exception e) {
            Log.e(TAG, "读取显示方向失败: " + e.getMessage());
        }
        return Surface.ROTATION_90;
    }

    // 将实际显示方向同步给 SLAM 侧的传感器重映射（3DOF 用）；
    // onCreate 读到的值可能偏旧，故在 onResume 与 onDisplayChanged 时再对齐一次。
    private void refreshDisplayRotation() {
        int rotation = readDisplayRotation();
        if (rotation != GlobalConstant.DISPLAY_ROTATION) {
            Log.d(TAG, "显示方向更新: " + GlobalConstant.DISPLAY_ROTATION + " -> " + rotation);
            GlobalConstant.setDisplayRotation(rotation);
        }
    }

    // 监听显示方向变化（横屏左右切换/多屏），保持 3DOF 的传感器坐标系与显示方向一致
    private DisplayManager mDisplayManager;
    private boolean mDisplayListenerRegistered;
    private final DisplayManager.DisplayListener mDisplayListener = new DisplayManager.DisplayListener() {
        @Override
        public void onDisplayAdded(int displayId) {
        }

        @Override
        public void onDisplayRemoved(int displayId) {
        }

        @Override
        public void onDisplayChanged(int displayId) {
            refreshDisplayRotation();
        }
    };

    private void registerDisplayListener() {
        if (mDisplayListenerRegistered) return;
        if (mDisplayManager == null) {
            mDisplayManager = (DisplayManager) getSystemService(Context.DISPLAY_SERVICE);
        }
        if (mDisplayManager == null) return;
        mDisplayManager.registerDisplayListener(mDisplayListener, mUiHandler);
        mDisplayListenerRegistered = true;
    }

    private void unregisterDisplayListener() {
        if (!mDisplayListenerRegistered || mDisplayManager == null) return;
        mDisplayManager.unregisterDisplayListener(mDisplayListener);
        mDisplayListenerRegistered = false;
    }

    @SuppressWarnings("deprecation")
    private void lockCurrentOrientation() {
        try {
            WindowManager wm = (WindowManager) getSystemService(Context.WINDOW_SERVICE);
            if (wm != null) {
                int rotation = readDisplayRotation();
                GlobalConstant.setDisplayRotation(rotation);

                if (rotation == Surface.ROTATION_270) {
                    setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_REVERSE_LANDSCAPE);
                    Log.d(TAG, "锁定为右横屏方向 (REVERSE_LANDSCAPE)");
                } else {
                    setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
                    Log.d(TAG, "锁定为左横屏方向 (LANDSCAPE)");
                }
            }
        } catch (Exception e) {
            Log.e(TAG, "锁定方向失败: " + e.getMessage());
        }
    }

    private void initViewsAndServices() {
        WindowCompat.setDecorFitsSystemWindows(getWindow(), false);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        WindowInsetsControllerCompat controller = WindowCompat.getInsetsController(getWindow(), getWindow().getDecorView());
        controller.setAppearanceLightNavigationBars(false);
        controller.setAppearanceLightStatusBars(false);

        setupWindowInsets();

        mSlamIPCClient = new SlamIPCClient(this);
        mMapManager = new MapManager(this);

        mCameraPreviewView = findViewById(R.id.my_fake_glsurface_view);
        mCameraPreviewView.setVisibility(View.VISIBLE);
        mCameraPreviewView.setFrameListener(this);
        mCameraPreviewView.setDisplayCorrectionListener(this);
        mCameraPreviewView.setSlamIPCClient(mSlamIPCClient);
        mCameraPreviewView.init();

        mIsInitFinished = false;
        mTouchHelper = new TouchGestureHelper(this);

        mBtn3DofCube = findViewById(R.id.btn_3dof_cube);
        if (mBtn3DofCube != null) {
            mBtn3DofCube.setOnClickListener(v -> toggle3DofMode());
        }

        init3DofTracker();
        initFilamentModel();

        View touchView = findViewById(R.id.touch_panel);
        touchView.setClickable(true);
        touchView.setOnTouchListener((v, event) -> {
            boolean handled = mTouchHelper.handleTouchEvent(event);
            if (event.getAction() == MotionEvent.ACTION_UP) {
                v.performClick();
            }
            return handled;
        });

        touchView.setOnClickListener(v -> {
            if (mCameraPreviewView != null) {
                Log.d(TAG, "onClick: CameraX 自动对焦");
                mCameraPreviewView.autoFocusCenter();
            }
        });

        mFpsTextView = findViewById(R.id.text_fps);
        mTextMapStats = findViewById(R.id.text_map_stats);
        mFpsCalculator = new FpsCalculator();

        startMapStatsUpdater();

        mBtnCreateArObject = findViewById(R.id.btn_create_ar_object);
        mBtnSaveMap = findViewById(R.id.btn_save_map);
        mBtnLoadMap = findViewById(R.id.btn_load_map);
        mBtnMapList = findViewById(R.id.btn_map_list);

        mBtnCreateArObject.setOnClickListener(v -> {
            Log.d(TAG, "点击按钮：创建 AR 物体");
            showToast(getString(R.string.hint_request_sent));
            if (mCameraPreviewView != null) {
                mCameraPreviewView.requestPlaneDetection();
            }
        });

        mBtnSaveMap.setOnClickListener(v -> showSaveMapDialog());
        mBtnLoadMap.setOnClickListener(v -> showMapListDialog(false));

        if (mBtnMapList != null) {
            mBtnMapList.setOnClickListener(v -> showMapListDialog(true));
        }

        mBtnTogglePointCloud = findViewById(R.id.btn_toggle_pointcloud);
        if (mBtnTogglePointCloud != null) {
            mBtnTogglePointCloud.setOnClickListener(v -> togglePointCloudDisplay());
        }

        mBtnSwitchCamera = findViewById(R.id.btn_switch_camera);
        if (mBtnSwitchCamera != null) {
            mBtnSwitchCamera.setOnClickListener(v -> switchCamera());
            mBtnSwitchCamera.setOnLongClickListener(v -> {
                showCameraSelectDialog();
                return true;
            });
        }

        initMirrorControls();

        Button btnGroupAr = findViewById(R.id.btn_group_ar);
        if (btnGroupAr != null) {
            btnGroupAr.setOnClickListener(v -> toggleExclusiveMenu(R.id.group_ar));
        }
        Button btnGroupMap = findViewById(R.id.btn_group_map);
        if (btnGroupMap != null) {
            btnGroupMap.setOnClickListener(v -> toggleExclusiveMenu(R.id.group_map));
        }
        Button btnGroupDisplay = findViewById(R.id.btn_group_display);
        if (btnGroupDisplay != null) {
            btnGroupDisplay.setOnClickListener(v -> toggleExclusiveMenu(R.id.group_display));
        }

        initJoystick();
    }

    private void setupWindowInsets() {
        View rootView = findViewById(R.id.ar_ui_root);
        if (rootView == null) {
            rootView = findViewById(android.R.id.content);
        }
        if (rootView == null) return;

        ViewCompat.setOnApplyWindowInsetsListener(rootView, (v, windowInsets) -> {
            Insets insets = windowInsets.getInsets(
                    WindowInsetsCompat.Type.systemBars() | WindowInsetsCompat.Type.displayCutout()
            );

            // 动态避让虚拟摇杆 (left/start + bottom)
            View joystickView = findViewById(R.id.joystick_view);
            if (joystickView != null && joystickView.getLayoutParams() instanceof ViewGroup.MarginLayoutParams) {
                ViewGroup.MarginLayoutParams lp = (ViewGroup.MarginLayoutParams) joystickView.getLayoutParams();
                int baseStart = (int) (16 * getResources().getDisplayMetrics().density);
                int baseBottom = (int) (16 * getResources().getDisplayMetrics().density);
                lp.setMarginStart(baseStart + insets.left);
                lp.bottomMargin = baseBottom + insets.bottom;
                joystickView.setLayoutParams(lp);
            }

            // 动态避让右下角操作面板 (right/end + bottom)
            View actionPanel = findViewById(R.id.action_panel);
            if (actionPanel != null && actionPanel.getLayoutParams() instanceof ViewGroup.MarginLayoutParams) {
                ViewGroup.MarginLayoutParams lp = (ViewGroup.MarginLayoutParams) actionPanel.getLayoutParams();
                int baseEnd = (int) (8 * getResources().getDisplayMetrics().density);
                int baseBottom = (int) (8 * getResources().getDisplayMetrics().density);
                lp.setMarginEnd(baseEnd + insets.right);
                lp.bottomMargin = baseBottom + insets.bottom;
                actionPanel.setLayoutParams(lp);
            }

            // 动态避让左上角 FPS/地图信息面板 (left/start + top)
            View infoPanel = findViewById(R.id.info_panel);
            if (infoPanel != null && infoPanel.getLayoutParams() instanceof ViewGroup.MarginLayoutParams) {
                ViewGroup.MarginLayoutParams lp = (ViewGroup.MarginLayoutParams) infoPanel.getLayoutParams();
                int baseStart = (int) (12 * getResources().getDisplayMetrics().density);
                int baseTop = (int) (12 * getResources().getDisplayMetrics().density);
                lp.setMarginStart(baseStart + insets.left);
                lp.topMargin = baseTop + insets.top;
                infoPanel.setLayoutParams(lp);
            }

            return windowInsets;
        });
    }

    private void toggleExclusiveMenu(int groupId) {
        View ga = findViewById(R.id.group_ar);
        View gm = findViewById(R.id.group_map);
        View gd = findViewById(R.id.group_display);
        View target = findViewById(groupId);
        boolean visible = target != null && target.getVisibility() == View.VISIBLE;
        if (ga != null) ga.setVisibility(View.GONE);
        if (gm != null) gm.setVisibility(View.GONE);
        if (gd != null) gd.setVisibility(View.GONE);
        if (!visible && target != null) target.setVisibility(View.VISIBLE);
    }

    private void showSaveMapDialog() {
        final EditText input = new EditText(this);
        input.setHint(getString(R.string.input_map_name));

        String defaultName = "map_" + new SimpleDateFormat("MMdd_HHmm", Locale.getDefault()).format(new Date());
        input.setText(defaultName);

        new AlertDialog.Builder(this)
                .setTitle(getString(R.string.dialog_save_map))
                .setView(input)
                .setPositiveButton(getString(R.string.btn_save), (dialog, which) -> {
                    String mapName = input.getText().toString().trim();
                    if (mapName.isEmpty()) mapName = defaultName;

                    final String resDir = getSlamDir().getAbsolutePath() + "/maps/" + mapName + ".bin";
                    if (mSlamIPCClient != null) {
                        mSlamIPCClient.saveMap(resDir);
                    }
                    showToast(getString(R.string.hint_map_saved, mapName));
                })
                .setNegativeButton(getString(R.string.button_cancel), null)
                .show();
    }

    private void showMapListDialog(final boolean showManage) {
        final ArrayList<MapManager.MapInfo> maps = mMapManager.getAllMaps();

        if (maps.isEmpty()) {
            showToast(getString(R.string.hint_no_maps));
            return;
        }

        String[] mapNames = new String[maps.size()];
        for (int i = 0; i < maps.size(); i++) {
            MapManager.MapInfo info = maps.get(i);
            mapNames[i] = info.name + " (" + Formatter.formatFileSize(this, info.fileSize) + ")";
        }

        if (!showManage) {
            final boolean[] checkedItems = new boolean[maps.size()];
            new AlertDialog.Builder(this)
                    .setTitle(getString(R.string.dialog_select_map))
                    .setMultiChoiceItems(mapNames, checkedItems, (dialog, which, isChecked) -> checkedItems[which] = isChecked)
                    .setPositiveButton(getString(R.string.action_load), (dialog, which) -> {
                        int loadedCount = 0;
                        for (int i = 0; i < maps.size(); i++) {
                            if (checkedItems[i]) {
                                boolean append = (loadedCount > 0);
                                final String resDir = getSlamDir().getAbsolutePath() + "/maps/" + maps.get(i).name + ".bin";
                                if (mSlamIPCClient != null) {
                                    mSlamIPCClient.loadMapWithId(resDir, loadedCount, append);
                                }
                                loadedCount++;
                            }
                        }
                        if (loadedCount > 0) {
                            showToast(getResources().getQuantityString(R.plurals.hint_maps_loaded, loadedCount, loadedCount));
                        }
                    })
                    .setNeutralButton(getString(R.string.button_cancel), null)
                    .show();
        } else {
            new AlertDialog.Builder(this)
                    .setTitle(getString(R.string.dialog_map_manage))
                    .setItems(mapNames, (dialog, which) -> {
                        final MapManager.MapInfo selectedMap = maps.get(which);
                        showMapOptionsDialog(selectedMap);
                    })
                    .setNegativeButton(getString(R.string.button_cancel), null)
                    .show();
        }
    }

    private void showMapOptionsDialog(final MapManager.MapInfo mapInfo) {
        String[] options = { getString(R.string.action_load), getString(R.string.action_delete),
                getString(R.string.action_view_details) };
        new AlertDialog.Builder(this)
                .setTitle(mapInfo.name)
                .setItems(options, (dialog, which) -> {
                    switch (which) {
                        case 0:
                            final String resDir = getSlamDir().getAbsolutePath() + "/maps/" + mapInfo.name + ".bin";
                            if (mSlamIPCClient != null) {
                                mSlamIPCClient.loadMapWithId(resDir, 0, false);
                            }
                            showToast(getString(R.string.hint_map_loaded, mapInfo.name));
                            break;
                        case 1:
                            new AlertDialog.Builder(MainActivity.this)
                                     .setTitle(getString(R.string.dialog_confirm_delete))
                                    .setMessage(getString(R.string.dialog_confirm_delete_message, mapInfo.name))
                                    .setPositiveButton(getString(R.string.action_delete), (d, w) -> {
                                        if (mMapManager.deleteMap(mapInfo.name)) {
                                            showToast(getString(R.string.hint_map_deleted));
                                        } else {
                                            showToast(getString(R.string.hint_map_delete_failed));
                                        }
                                    })
                                    .setNegativeButton(getString(R.string.button_cancel), null)
                                    .show();
                            break;
                        case 2:
                            showMapDetails(mapInfo);
                            break;
                    }
                })
                .setNegativeButton(getString(R.string.button_back), null)
                .show();
    }

    private void showMapDetails(MapManager.MapInfo mapInfo) {
        String details = getString(R.string.map_details_name, mapInfo.name) + "\n" +
                getString(R.string.map_details_keyframes, mapInfo.keyFrames) + "\n" +
                getString(R.string.map_details_mappoints, mapInfo.mapPoints) + "\n" +
                getString(R.string.map_details_size, mapInfo.fileSize / 1024) + "\n" +
                getString(R.string.map_details_time, new SimpleDateFormat("yyyy-MM-dd HH:mm:ss",
                        Locale.getDefault()).format(new Date(mapInfo.createTime)))
                + "\n" +
                getString(R.string.map_details_plane, mapInfo.hasPlane ? getString(R.string.map_details_plane_yes)
                        : getString(R.string.map_details_plane_no));

        new AlertDialog.Builder(this)
                .setTitle(getString(R.string.dialog_map_details))
                .setMessage(details)
                .setPositiveButton(getString(R.string.button_ok), null)
                .show();
    }

    private void initFilamentModel() {
        Log.d(TAG, "initFilamentModel: 初始化原生 C++ GLB 渲染器");

        final AspectGLSurfaceView glRootView = findViewById(R.id.ar_object_view_gles2_obj);
        glRootView.setAspectRatio(GlobalConstant.RESOLUTION_WIDTH, GlobalConstant.RESOLUTION_HEIGHT);
        mArObjectView = glRootView;

        mModelRenderer = GlbModelRenderer.newInstance()
                .setArObjectView(glRootView)
                .setSlamIPCClient(mSlamIPCClient)
                .setOrientationSensor(mOrientationSensor)
                .setSensorFusionTracker(mSensorFusionTracker)
                .setContext(this)
                .setModelPath("model.glb")
                .setInitSize(0.20f)
                .setDrawStateListener(shouldDraw -> runOnUiThread(() -> {
                    if (mJoystickView != null) {
                        mJoystickView.setVisibility(shouldDraw ? View.VISIBLE : View.GONE);
                    }
                }))
                .init(mTouchHelper);

        // 若首帧之前已得到矫正，先同步给 AR 层
        applyDisplayCorrection(mCameraPreviewView != null ? mCameraPreviewView.getDisplayCorrection() : null);
    }

    // 应用显示层矫正：更新 AR 视图宽高比与模型投影，与预览位图、点云保持同一变换。
    @Override
    public void onDisplayCorrectionChanged(DisplayCorrection correction) {
        applyDisplayCorrection(correction);
    }

    private void applyDisplayCorrection(DisplayCorrection correction) {
        if (correction == null) return;
        if (mArObjectView != null) {
            mArObjectView.setAspectRatio(correction.displayWidth, correction.displayHeight);
        }
        if (mModelRenderer != null) {
            mModelRenderer.setDisplayCorrection(correction);
        }
    }

    @Override
    protected void onPause() {
        Log.d(TAG, "onPause: 暂停摄像头视图");
        super.onPause();
        unregisterDisplayListener();
        stopMapStatsUpdater();
        if (mCameraPreviewView != null) {
            mCameraPreviewView.disableView();
        }

        if (mIs3DofMode && mOrientationSensor != null) {
            mOrientationSensor.stop();
        }

        if (mModelRenderer != null) {
            mModelRenderer.onPause();
        }

        if (mSlamIPCClient != null) {
            mSlamIPCClient.unbindService();
        }
    }

    @Override
    protected void onResume() {
        Log.d(TAG, "onResume: 准备启动");
        super.onResume();

        // window 已 attach，此时读到的是真实显示方向（onCreate 阶段可能还是旧值）
        refreshDisplayRotation();
        registerDisplayListener();

        if (mModelRenderer != null) {
            mModelRenderer.onResume();
        }

        if (mSlamIPCClient != null) {
            mSlamIPCClient.bindService();
        }

        if (!mIsInitFinished) {
            mIsInitFinished = true;
            initSLAMAsync();
        } else {
            mCameraPreviewView.enableView();
        }

        if (mIs3DofMode && mOrientationSensor != null) {
            mOrientationSensor.start(this);
        }

        startMapStatsUpdater();
    }

    @Override
    protected void onSaveInstanceState(@NonNull Bundle outState) {
        super.onSaveInstanceState(outState);
        outState.putBoolean(KEY_3DOF_MODE, mIs3DofMode);
    }

    @Override
    public void onTrimMemory(int level) {
        super.onTrimMemory(level);
        Log.d(TAG, "onTrimMemory: level=" + level);
        // UI 不可见或系统内存偏低时，停止 1Hz 地图统计轮询，释放相关占用
        if (level >= ComponentCallbacks2.TRIM_MEMORY_UI_HIDDEN) {
            stopMapStatsUpdater();
            mUiHandler.removeCallbacksAndMessages(null);
        }
    }

    private void initSLAMAsync() {
        showLoadingDialog(getString(R.string.loading_slam_init), getString(R.string.loading_slam_wait));

        new Thread(() -> {
            try {
                final String resDir = getSlamDir().getAbsolutePath() + "/";

                if (mSlamIPCClient != null) {
                    mSlamIPCClient.bindService();
                    mSlamIPCClient.initSLAM(resDir);
                }

                runOnUiThread(() -> {
                    dismissLoadingDialog();
                    showToast(getString(R.string.slam_init_complete));
                    mCameraPreviewView.enableView();
                });

            } catch (final Exception e) {
                Log.e(TAG, "SLAM 初始化失败: " + e.getMessage(), e);
                runOnUiThread(() -> {
                    dismissLoadingDialog();
                    showToast(getString(R.string.slam_init_failed, e.getMessage()));
                });
            }
        }).start();
    }

    private void showLoadingDialog(String title, String message) {
        runOnUiThread(() -> {
            if (mLoadingDialog != null && mLoadingDialog.isShowing()) {
                mLoadingDialog.dismiss();
            }

            LinearLayout container = new LinearLayout(MainActivity.this);
            container.setOrientation(LinearLayout.HORIZONTAL);
            int padding = (int) (16 * getResources().getDisplayMetrics().density);
            container.setPadding(padding, padding, padding, padding);

            ProgressBar progressBar = new ProgressBar(MainActivity.this);
            progressBar.setIndeterminate(true);

            TextView msgView = new TextView(MainActivity.this);
            msgView.setText(message);
            msgView.setTextColor(0xFF000000);
            msgView.setTextSize(16);
            LinearLayout.LayoutParams textLp = new LinearLayout.LayoutParams(
                    LinearLayout.LayoutParams.WRAP_CONTENT,
                    LinearLayout.LayoutParams.WRAP_CONTENT);
            textLp.leftMargin = padding / 2;
            container.addView(progressBar);
            container.addView(msgView, textLp);

            mLoadingDialog = new AlertDialog.Builder(MainActivity.this)
                    .setTitle(title)
                    .setView(container)
                    .setCancelable(false)
                    .create();
            mLoadingDialog.show();
        });
    }

    private void dismissLoadingDialog() {
        runOnUiThread(() -> {
            if (mLoadingDialog != null && mLoadingDialog.isShowing()) {
                mLoadingDialog.dismiss();
                mLoadingDialog = null;
            }
        });
    }

    @Override
    protected void onDestroy() {
        Log.d(TAG, "onDestroy: 释放资源");

        if (mModelRenderer != null) {
            mModelRenderer.destroy();
            mModelRenderer = null;
        }

        super.onDestroy();
        if (mCameraPreviewView != null) {
            mCameraPreviewView.disableView();
        }

        mUiHandler.removeCallbacksAndMessages(null);

        if (mSlamIPCClient != null) {
            mSlamIPCClient.unbindService();
        }

        dismissLoadingDialog();
    }

    @Override
    public void onCameraStarted(int width, int height) {
        Log.d(TAG, "onCameraStarted: 摄像头视图启动，宽度=" + width + " 高度=" + height);
        if (mSlamIPCClient != null) {
            mSlamIPCClient.updateResolution(width, height);
        }
    }

    @Override
    public void onCameraStopped() {
        Log.d(TAG, "onCameraStopped: 摄像头视图停止");
    }

    @Override
    public void onCameraUnavailable(int cameraCount) {
        Log.w(TAG, "onCameraUnavailable: 相机不可用，检测到相机数量=" + cameraCount);
        runOnUiThread(() -> {
            showToast(getString(R.string.hint_no_camera));
            if (mFpsTextView != null) {
                mFpsTextView.setText(getString(R.string.hint_no_camera));
            }
        });
    }

    @Override
    public void onCameraFrame() {
        mFpsCalculator.measure();
        if (mFpsCalculator.isUpdated()) {
            final String fpsText = mFpsCalculator.getText();
            runOnUiThread(() -> mFpsTextView.setText(fpsText));
        }
    }

    private void showToast(final String str) {
        runOnUiThread(() -> Toast.makeText(MainActivity.this, str, Toast.LENGTH_LONG).show());
    }

    private void startMapStatsUpdater() {
        mUiHandler.removeCallbacks(mMapStatsUpdater);
        mUiHandler.postDelayed(mMapStatsUpdater, 1000);
    }

    private void stopMapStatsUpdater() {
        mUiHandler.removeCallbacks(mMapStatsUpdater);
    }

    private void initJoystick() {
        mJoystickView = findViewById(R.id.joystick_view);
        if (mJoystickView != null) {
            mJoystickView.setOnJoystickListener((angleDeg, intensity) -> {
                if (mModelRenderer != null && intensity > 0.01f) {
                    float speed = 3.0f * intensity;
                    float yawDelta = (float) Math.cos(Math.toRadians(angleDeg)) * speed;
                    float pitchDelta = -(float) Math.sin(Math.toRadians(angleDeg)) * speed;

                    // 摇杆位于屏幕坐标系，模型旋转作用在世界系，
                    // 显示层镜像或旋转时需同步翻转方向，否则推向右却向左转
                    DisplayCorrection correction = mCameraPreviewView != null
                            ? mCameraPreviewView.getDisplayCorrection() : null;
                    if (correction != null) {
                        yawDelta *= correction.screenXSign();
                        pitchDelta *= correction.screenYSign();
                    }
                    mModelRenderer.addUserRotation(yawDelta, pitchDelta);
                }
            });
            Log.d(TAG, "虚拟摇杆初始化完成");
        }
    }

    // 画面镜像控制：切换显示层的上下与左右镜像，
    // 开关状态由 CameraPreviewView 持久化，这里仅同步按钮状态与文案。
    private void initMirrorControls() {
        mBtnMirrorVertical = findViewById(R.id.btn_mirror_vertical);
        mBtnMirrorHorizontal = findViewById(R.id.btn_mirror_horizontal);

        boolean mirrorVertical = mCameraPreviewView != null && mCameraPreviewView.isMirrorVertical();
        boolean mirrorHorizontal = mCameraPreviewView != null && mCameraPreviewView.isMirrorHorizontal();
        updateMirrorButton(mBtnMirrorVertical, R.string.btn_mirror_vertical, mirrorVertical);
        updateMirrorButton(mBtnMirrorHorizontal, R.string.btn_mirror_horizontal, mirrorHorizontal);

        if (mBtnMirrorVertical != null) {
            mBtnMirrorVertical.setOnClickListener(v ->
                    setMirrorVertical(mCameraPreviewView == null || !mCameraPreviewView.isMirrorVertical()));
        }
        if (mBtnMirrorHorizontal != null) {
            mBtnMirrorHorizontal.setOnClickListener(v ->
                    setMirrorHorizontal(mCameraPreviewView == null || !mCameraPreviewView.isMirrorHorizontal()));
        }
    }

    private void setMirrorVertical(boolean enabled) {
        if (mCameraPreviewView != null) {
            mCameraPreviewView.setMirrorVertical(enabled);
        }
        updateMirrorButton(mBtnMirrorVertical, R.string.btn_mirror_vertical, enabled);
    }

    private void setMirrorHorizontal(boolean enabled) {
        if (mCameraPreviewView != null) {
            mCameraPreviewView.setMirrorHorizontal(enabled);
        }
        updateMirrorButton(mBtnMirrorHorizontal, R.string.btn_mirror_horizontal, enabled);
    }

    // 镜像按钮文案：开启时加 ✓ 前缀，与点云按钮的激活反馈保持一致
    private void updateMirrorButton(Button button, int labelRes, boolean enabled) {
        if (button == null) return;
        String label = getString(labelRes);
        button.setText(enabled ? "✓" + label : label);
    }

    private void togglePointCloudDisplay() {
        if (mSlamIPCClient != null && mBtnTogglePointCloud != null) {
            boolean currentState = mSlamIPCClient.isPointCloudDisplayEnabled();
            boolean newState = !currentState;
            mSlamIPCClient.setPointCloudDisplay(newState);

            if (newState) {
                mBtnTogglePointCloud.setText(getString(R.string.btn_pointcloud_enabled));
                showToast(getString(R.string.hint_pointcloud_enabled));
                Log.d(TAG, "点云显示已启用");
            } else {
                mBtnTogglePointCloud.setText(getString(R.string.btn_pointcloud_disabled));
                showToast(getString(R.string.hint_pointcloud_disabled));
                Log.d(TAG, "点云显示已禁用");
            }
        } else {
            Log.e(TAG, "无法切换点云显示：SlamIPCClient 未连接");
        }
    }

    private void switchCamera() {
        if (mCameraPreviewView == null) return;
        CameraPreviewView.CameraSwitchResult result = mCameraPreviewView.switchCamera();
        if (result.status == CameraPreviewView.CameraSwitchResult.STATUS_SUCCESS) {
            if (result.lensFacing == CameraSelector.LENS_FACING_BACK) {
                showToast(getString(R.string.hint_camera_switched_back));
            } else if (result.lensFacing == CameraSelector.LENS_FACING_FRONT) {
                showToast(getString(R.string.hint_camera_switched_front));
            } else {
                showToast(getString(R.string.hint_camera_switched_other, result.cameraId != null ? result.cameraId : ""));
            }
        } else if (result.status == CameraPreviewView.CameraSwitchResult.STATUS_SINGLE_CAMERA) {
            showToast(getString(R.string.hint_camera_switch_single));
        } else {
            showToast(getString(R.string.hint_camera_switch_failed));
        }
    }

    @OptIn(markerClass = ExperimentalCamera2Interop.class)
    private void showCameraSelectDialog() {
        if (mCameraPreviewView == null) return;
        List<CameraInfo> cameraList = mCameraPreviewView.getAvailableCameraInfos();
        if (cameraList == null || cameraList.isEmpty()) {
            showToast(getString(R.string.hint_no_camera));
            return;
        }
        if (cameraList.size() == 1) {
            showToast(getString(R.string.hint_camera_switch_single));
            return;
        }

        int currentIndex = mCameraPreviewView.getCurrentCameraIndex();
        String[] cameraNames = new String[cameraList.size()];
        for (int i = 0; i < cameraList.size(); i++) {
            CameraInfo info = cameraList.get(i);
            String id = String.valueOf(i);
            try {
                id = Camera2CameraInfo.from(info).getCameraId();
            } catch (Throwable ignored) {}

            String name;
            if (info.getLensFacing() == CameraSelector.LENS_FACING_BACK) {
                name = getString(R.string.camera_facing_back, id);
            } else if (info.getLensFacing() == CameraSelector.LENS_FACING_FRONT) {
                name = getString(R.string.camera_facing_front, id);
            } else {
                name = getString(R.string.camera_facing_other, id);
            }
            if (i == currentIndex) {
                name += " (" + getString(R.string.icon_in_use) + ")";
            }
            cameraNames[i] = name;
        }

        new AlertDialog.Builder(this)
                .setTitle(R.string.dialog_select_camera)
                .setSingleChoiceItems(cameraNames, currentIndex, (dialog, which) -> {
                    dialog.dismiss();
                    if (which != currentIndex) {
                        CameraPreviewView.CameraSwitchResult result = mCameraPreviewView.switchCameraTo(which);
                        if (result.status == CameraPreviewView.CameraSwitchResult.STATUS_SUCCESS) {
                            if (result.lensFacing == CameraSelector.LENS_FACING_BACK) {
                                showToast(getString(R.string.hint_camera_switched_back));
                            } else if (result.lensFacing == CameraSelector.LENS_FACING_FRONT) {
                                showToast(getString(R.string.hint_camera_switched_front));
                            } else {
                                showToast(getString(R.string.hint_camera_switched_other, result.cameraId != null ? result.cameraId : ""));
                            }
                        } else {
                            showToast(getString(R.string.hint_camera_switch_failed));
                        }
                    }
                })
                .setNegativeButton(R.string.button_cancel, null)
                .show();
    }

    private void init3DofTracker() {
        mOrientationSensor = new OrientationSensor();
        mSensorFusionTracker = new SensorFusionTracker();

        // 确保获取传感器控制按钮引用
        if (mBtn3DofCube == null) {
            mBtn3DofCube = findViewById(R.id.btn_3dof_cube);
            if (mBtn3DofCube != null) {
                mBtn3DofCube.setOnClickListener(v -> toggle3DofMode());
            }
        }

        // 多级回退检测：具备任意一种姿态来源即可开启，不再强制要求磁力计/旋转矢量
        OrientationSensor.Mode mode = mOrientationSensor.detectMode(this);
        Log.d(TAG, "3DOF 姿态来源检测结果: " + mode);

        if (mode == OrientationSensor.Mode.NONE) {
            Log.w(TAG, "设备没有任何可用于 3DOF 的姿态传感器硬件，隐藏传感器按钮");
            mIs3DofMode = false;
            mSensorFusionTracker.set3DofAssistEnabled(false);
            if (mBtn3DofCube != null) {
                mBtn3DofCube.setVisibility(View.GONE);
            }
            return;
        }

        // 检测到对应硬件：默认启动传感器辅助定位
        mIs3DofMode = true;
        mSensorFusionTracker.setRotationDof(mOrientationSensor.getRotationDof());
        mOrientationSensor.start(this);
        mSensorFusionTracker.set3DofAssistEnabled(true);

        if (mBtn3DofCube != null) {
            mBtn3DofCube.setVisibility(View.VISIBLE);
            mBtn3DofCube.setText(getString(R.string.btn_3dof_on));
        }

        Log.d(TAG, "3DOF 传感器与融合跟踪器初始化完成 (默认启动，姿态来源: " + mode + ")");
    }

    private void toggle3DofMode() {
        set3DofMode(!mIs3DofMode, true);
    }

    private void set3DofMode(boolean enabled, boolean showToast) {
        if (mOrientationSensor == null || mSensorFusionTracker == null) {
            if (showToast) {
                showToast(getString(R.string.hint_3dof_unavailable));
            }
            if (mBtn3DofCube != null) {
                mBtn3DofCube.setVisibility(View.GONE);
            }
            return;
        }

        if (enabled) {
            // 开启前重新确认姿态来源，若无传感器硬件则隐藏按钮并提示
            if (mOrientationSensor.detectMode(this) == OrientationSensor.Mode.NONE) {
                if (showToast) {
                    showToast(getString(R.string.hint_3dof_unavailable));
                }
                if (mBtn3DofCube != null) {
                    mBtn3DofCube.setVisibility(View.GONE);
                }
                mIs3DofMode = false;
                mSensorFusionTracker.set3DofAssistEnabled(false);
                return;
            }
            mIs3DofMode = true;
            mSensorFusionTracker.setRotationDof(mOrientationSensor.getRotationDof());
            mOrientationSensor.start(this);
            mSensorFusionTracker.set3DofAssistEnabled(true);

            if (mBtn3DofCube != null) {
                mBtn3DofCube.setVisibility(View.VISIBLE);
                mBtn3DofCube.setText(getString(R.string.btn_3dof_on));
            }
            if (showToast) {
                showSensorModeToast();
            }
            Log.d(TAG, "3DOF 传感器辅助定位已开启 (旋转防抖 + 丢失空间保活)");
        } else {
            mIs3DofMode = false;
            mOrientationSensor.stop();
            mSensorFusionTracker.set3DofAssistEnabled(false);

            if (mBtn3DofCube != null) {
                mBtn3DofCube.setText(getString(R.string.btn_3dof_off));
            }
            if (showToast) {
                showToast(getString(R.string.hint_3dof_assisted_off));
            }
            Log.d(TAG, "3DOF 辅助定位已关闭，恢复纯视觉 SLAM");
        }
    }

    // 提示当前实际使用的姿态来源，并对回退模式的局限性给出说明
    private void showSensorModeToast() {
        OrientationSensor.Mode mode = mOrientationSensor.getMode();
        StringBuilder sb = new StringBuilder(getString(R.string.hint_3dof_assisted_on));
        sb.append('\n').append(getString(R.string.sensor_mode_active, getString(getSensorModeNameRes(mode))));
        int noticeRes = getSensorModeNoticeRes(mode);
        if (noticeRes != 0) {
            sb.append('\n').append(getString(noticeRes));
        }
        showToast(sb.toString());
    }

    private int getSensorModeNameRes(OrientationSensor.Mode mode) {
        if (mode == null) return R.string.sensor_mode_none;
        switch (mode) {
            case ROTATION_VECTOR:
                return R.string.sensor_mode_rotation_vector;
            case GAME_ROTATION_VECTOR:
                return R.string.sensor_mode_game_rotation_vector;
            case ACCEL_MAG:
                return R.string.sensor_mode_accel_mag;
            case GYROSCOPE:
                return R.string.sensor_mode_gyroscope;
            case ACCEL_ONLY:
                return R.string.sensor_mode_accel_only;
            default:
                return R.string.sensor_mode_none;
        }
    }

    private int getSensorModeNoticeRes(OrientationSensor.Mode mode) {
        if (mode == null) return R.string.sensor_mode_notice_none;
        switch (mode) {
            case GAME_ROTATION_VECTOR:
            case GYROSCOPE:
                return R.string.sensor_mode_notice_fallback;
            case ACCEL_ONLY:
                return R.string.sensor_mode_notice_accel_only;
            case NONE:
                return R.string.sensor_mode_notice_none;
            default:
                return 0;
        }
    }
}