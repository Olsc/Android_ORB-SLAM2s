package com.orb.slam2s.ui;

import android.content.Context;
import android.content.pm.ActivityInfo;
import android.graphics.Point;
import android.opengl.GLSurfaceView;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.text.format.Formatter;
import android.util.Log;
import android.view.Display;
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

import androidx.activity.OnBackPressedCallback;
import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;

import com.orb.slam2s.R;
import com.orb.slam2s.camera.CameraPreviewView;
import com.orb.slam2s.constant.GlobalConstant;
import com.orb.slam2s.graphics.AspectSurfaceView;
import com.orb.slam2s.graphics.FilamentModelRenderer;
import com.orb.slam2s.graphics.ThreeDofCubeRenderer;
import com.orb.slam2s.ipc.SlamIPCClient;
import com.orb.slam2s.sensors.OrientationSensor;
import com.orb.slam2s.util.FpsCalculator;
import com.orb.slam2s.util.MapManager;
import com.orb.slam2s.util.TouchGestureHelper;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Locale;

// SLAM AR 主控制 Activity（调度相机、3D 模型渲染、3DOF 追踪、地图管理与交互控制）
public class MainActivity extends AppCompatActivity implements CameraPreviewView.FrameListener {

    private static final String TAG = "MainActivity";

    private CameraPreviewView mCameraPreviewView;
    private boolean mIsInitFinished = false;

    private SlamIPCClient mSlamIPCClient;
    private MapManager mMapManager;
    private TouchGestureHelper mTouchHelper;
    private FilamentModelRenderer mModelRenderer;

    private FpsCalculator mFpsCalculator;
    private TextView mFpsTextView;
    private TextView mTextMapStats;
    private Button mBtnCreateArObject;
    private Button mBtnSaveMap;
    private Button mBtnLoadMap;
    private Button mBtnMapList;
    private Button mBtnTogglePointCloud;
    private Button mBtn3DofCube;

    private final Handler mUiHandler = new Handler(Looper.getMainLooper());
    private AlertDialog mLoadingDialog;

    // 虚拟摇杆
    private VirtualJoystickView mJoystickView;

    // 3DOF 空间跟踪
    private OrientationSensor mOrientationSensor;
    private GLSurfaceView mThreeDofGLView;
    private ThreeDofCubeRenderer mThreeDofRenderer;
    private boolean mIs3DofMode = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.d(TAG, "onCreate: 初始化 SLAM 主界面");

        lockCurrentOrientation();
        computeScreenResolution();

        setContentView(R.layout.ar_ui_content);
        initViewsAndServices();

        getOnBackPressedDispatcher().addCallback(this, new OnBackPressedCallback(true) {
            @Override
            public void handleOnBackPressed() {
                Log.d(TAG, "handleOnBackPressed: 退出程序");
                finish();
            }
        });
    }

    private void computeScreenResolution() {
        WindowManager wm = (WindowManager) getSystemService(Context.WINDOW_SERVICE);
        if (wm != null) {
            Point size = new Point();
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
                Display display = getDisplay();
                if (display != null) {
                    Point realSize = new Point();
                    display.getRealSize(realSize);
                    size.x = realSize.x;
                    size.y = realSize.y;
                }
            } else {
                wm.getDefaultDisplay().getRealSize(size);
            }

            int screenWidth = size.x;
            int screenHeight = size.y;
            Log.d(TAG, "屏幕分辨率: " + screenWidth + "x" + screenHeight);

            GlobalConstant.computeOptimalResolution(screenWidth, screenHeight);
            Log.d(TAG, "选择相机分辨率: " + GlobalConstant.RESOLUTION_WIDTH + "x" + GlobalConstant.RESOLUTION_HEIGHT);
        }
    }

    private void lockCurrentOrientation() {
        try {
            WindowManager wm = (WindowManager) getSystemService(Context.WINDOW_SERVICE);
            if (wm != null) {
                int rotation;
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
                    Display display = getDisplay();
                    rotation = (display != null) ? display.getRotation() : Surface.ROTATION_90;
                } else {
                    rotation = wm.getDefaultDisplay().getRotation();
                }

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
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        mSlamIPCClient = new SlamIPCClient(this);
        mMapManager = new MapManager(this, mSlamIPCClient);

        mCameraPreviewView = findViewById(R.id.my_fake_glsurface_view);
        mCameraPreviewView.setVisibility(View.VISIBLE);
        mCameraPreviewView.setFrameListener(this);
        mCameraPreviewView.setSlamIPCClient(mSlamIPCClient);
        mCameraPreviewView.init();

        mIsInitFinished = false;
        mTouchHelper = new TouchGestureHelper(this);

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

        mBtn3DofCube = findViewById(R.id.btn_3dof_cube);
        if (mBtn3DofCube != null) {
            mBtn3DofCube.setOnClickListener(v -> toggle3DofMode());
        }

        initJoystick();
        init3DofTracker();
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

                    final String resDir = getExternalFilesDir("SLAM").getAbsolutePath() + "/maps/" + mapName + ".bin";
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
                                final String resDir = getExternalFilesDir("SLAM").getAbsolutePath() + "/maps/" + maps.get(i).name + ".bin";
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
                            final String resDir = getExternalFilesDir("SLAM").getAbsolutePath() + "/maps/" + mapInfo.name + ".bin";
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
        Log.d(TAG, "initFilamentModel: 初始化 Filament GLB 渲染器");

        final AspectSurfaceView glRootView = findViewById(R.id.ar_object_view_gles2_obj);
        glRootView.setAspectRatio(GlobalConstant.RESOLUTION_WIDTH, GlobalConstant.RESOLUTION_HEIGHT);

        mModelRenderer = FilamentModelRenderer.newInstance()
                .setArObjectView(glRootView)
                .setSlamIPCClient(mSlamIPCClient)
                .setContext(this)
                .setModelPath("model.glb")
                .setInitSize(0.20f)
                .setDrawStateListener(shouldDraw -> runOnUiThread(() -> {
                    if (mJoystickView != null) {
                        mJoystickView.setVisibility(shouldDraw ? View.VISIBLE : View.GONE);
                    }
                }))
                .init(mTouchHelper);
    }

    @Override
    protected void onPause() {
        Log.d(TAG, "onPause: 暂停摄像头视图");
        super.onPause();
        if (mCameraPreviewView != null) {
            mCameraPreviewView.disableView();
        }

        if (mIs3DofMode && mOrientationSensor != null) {
            mOrientationSensor.stop();
            if (mThreeDofGLView != null) {
                mThreeDofGLView.onPause();
            }
        }

        if (mSlamIPCClient != null) {
            mSlamIPCClient.unbindService();
        }
    }

    @Override
    protected void onResume() {
        Log.d(TAG, "onResume: 准备启动");
        super.onResume();

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
            if (mThreeDofGLView != null) {
                mThreeDofGLView.onResume();
            }
        }
    }

    private void initSLAMAsync() {
        showLoadingDialog(getString(R.string.loading_slam_init), getString(R.string.loading_slam_wait));

        new Thread(() -> {
            try {
                final String resDir = getExternalFilesDir("SLAM").getAbsolutePath() + "/";

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
        final Runnable updater = new Runnable() {
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
        mUiHandler.postDelayed(updater, 1000);
    }

    private void initJoystick() {
        mJoystickView = findViewById(R.id.joystick_view);
        if (mJoystickView != null) {
            mJoystickView.setOnJoystickListener((angleDeg, intensity) -> {
                if (mModelRenderer != null && intensity > 0.01f) {
                    float speed = 3.0f * intensity;
                    float yawDelta = (float) Math.cos(Math.toRadians(angleDeg)) * speed;
                    float pitchDelta = -(float) Math.sin(Math.toRadians(angleDeg)) * speed;
                    mModelRenderer.addUserRotation(yawDelta, pitchDelta);
                }
            });
            Log.d(TAG, "虚拟摇杆初始化完成");
        }
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

    private void init3DofTracker() {
        mOrientationSensor = new OrientationSensor();

        if (!mOrientationSensor.hasRequiredSensors(this)) {
            Log.w(TAG, "设备缺少 3DOF 所需的传感器");
            if (mBtn3DofCube != null) {
                mBtn3DofCube.setEnabled(false);
                mBtn3DofCube.setAlpha(0.5f);
            }
            return;
        }

        mThreeDofGLView = new GLSurfaceView(this);
        mThreeDofGLView.setEGLContextClientVersion(2);
        mThreeDofGLView.setEGLConfigChooser(8, 8, 8, 8, 16, 0);
        mThreeDofGLView.getHolder().setFormat(android.graphics.PixelFormat.TRANSLUCENT);
        mThreeDofGLView.setZOrderOnTop(true);

        mThreeDofRenderer = new ThreeDofCubeRenderer(this, mOrientationSensor);
        mThreeDofGLView.setRenderer(mThreeDofRenderer);
        mThreeDofGLView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);

        RelativeLayout.LayoutParams params = new RelativeLayout.LayoutParams(
                RelativeLayout.LayoutParams.MATCH_PARENT,
                RelativeLayout.LayoutParams.MATCH_PARENT);
        params.addRule(RelativeLayout.CENTER_IN_PARENT);

        RelativeLayout rootLayout = (RelativeLayout) findViewById(R.id.my_fake_glsurface_view).getParent();
        rootLayout.addView(mThreeDofGLView, 2, params);

        mThreeDofGLView.setVisibility(View.GONE);

        Log.d(TAG, "3DOF 传感器和渲染器初始化完成");
    }

    private void toggle3DofMode() {
        if (mOrientationSensor == null || mThreeDofRenderer == null) {
            showToast(getString(R.string.hint_3dof_unavailable));
            return;
        }

        if (!mIs3DofMode) {
            mIs3DofMode = true;
            mOrientationSensor.start(this);
            mThreeDofGLView.setVisibility(View.VISIBLE);
            mThreeDofGLView.onResume();

            mThreeDofRenderer.spawnCubeAtDistance(5.0f);

            if (mBtn3DofCube != null) {
                mBtn3DofCube.setText(getString(R.string.btn_3dof_close));
            }
            showToast(getString(R.string.hint_3dof_spawned));
            Log.d(TAG, "3DOF 模式已启动");
        } else {
            mIs3DofMode = false;
            mOrientationSensor.stop();
            mThreeDofRenderer.hideCube();
            mThreeDofGLView.onPause();
            mThreeDofGLView.setVisibility(View.GONE);

            if (mBtn3DofCube != null) {
                mBtn3DofCube.setText(getString(R.string.btn_3dof));
            }
            showToast(getString(R.string.hint_3dof_closed));
            Log.d(TAG, "3DOF 模式已关闭");
        }
    }
}
