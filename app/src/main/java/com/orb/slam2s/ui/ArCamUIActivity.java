package com.orb.slam2s.ui;

/**
 * Created by Ads on 2017/3/9.
 * 由Olsc于2025/8/25开始进行修改
 */

import android.content.Context;
import android.content.pm.ActivityInfo;
import android.graphics.Bitmap;
import android.graphics.Point;
import android.opengl.GLSurfaceView;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.Display;
import android.view.MotionEvent;
import android.view.Surface;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.activity.OnBackPressedCallback;
import androidx.appcompat.app.AppCompatActivity;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.MultiFormatWriter;
import com.google.zxing.WriterException;
import com.google.zxing.common.BitMatrix;
import com.orb.slam2s.R;
import com.orb.slam2s.constant.GlobalConstant;
import com.orb.slam2s.ipc.SlamIPCClient;
import com.orb.slam2s.rendering.gles.FilamentAspectSurfaceView;
import com.orb.slam2s.rendering.render.ModelRendererWrapper;
import com.orb.slam2s.rendering.render.ThreeDofCubeRenderer;
import com.orb.slam2s.sensors.OrientationSensor;
import com.orb.slam2s.server.WebServer;
import com.orb.slam2s.slamar.NativeHelper;
import com.orb.slam2s.utils.FpsMeter;
import com.orb.slam2s.utils.TouchHelper;

import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.Enumeration;

@SuppressWarnings("deprecation")
public class ArCamUIActivity extends AppCompatActivity implements
        CameraGLViewBase.CvCameraViewListener2 {

    private static final String TAG = "SlamCamActivity";

    private CameraGLView mOpenCvCameraView;
    private boolean initFinished;

    private NativeHelper nativeHelper;
    private SlamIPCClient slamIPCClient;
    private NativeHelper.MapManager mapManager;
    private TouchHelper touchHelper;
    private ModelRendererWrapper modelRendererWrapper;

    private FpsMeter mFpsMeter = null;
    private TextView fpsText;
    private TextView textMapStats;
    private Button btnCreateArObject;
    private Button btnSaveMap;
    private Button btnLoadMap;
    private Button btnMapList;
    private Button btnTogglePointCloud;
    private Button btnToggleSlam;

    private Button btn3DofCube;
    private final android.os.Handler uiHandler = new android.os.Handler();
    private androidx.appcompat.app.AlertDialog loadingDialog;

    // 拖动相关变量
    private float qrDX, qrDY;

    // Web Server 相关 UI
    private View floatingQrWindow;
    private android.widget.ImageView ivQrCode;
    private android.widget.TextView tvWebUrl;

    private WebServer webServer;
    private Button btnStartWeb;
    private boolean isWebRunning = false;

    // 摇杆控制AR物体旋转
    private JoystickView joystickView;

    // 3DOF功能相关
    private OrientationSensor orientationSensor;
    private GLSurfaceView threeDofGLView;
    private ThreeDofCubeRenderer threeDofRenderer;
    private boolean is3DofMode = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.d(TAG, "onCreate: 初始化Activity");

        lockCurrentOrientation();
        computeScreenResolution();

        setContentView(R.layout.ar_ui_content);
        initView();
        getOnBackPressedDispatcher().addCallback(this, new OnBackPressedCallback(true) {
            @Override
            public void handleOnBackPressed() {
                Log.d(TAG, "onBackPressed: 退出程序");
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

    private void initView() {
        Log.d(TAG, "initView: 初始化视图与组件");

        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        nativeHelper = new NativeHelper(this);
        mapManager = new NativeHelper.MapManager(this, nativeHelper);
        slamIPCClient = new SlamIPCClient(this);

        mOpenCvCameraView = findViewById(R.id.my_fake_glsurface_view);
        mOpenCvCameraView.setVisibility(View.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);
        mOpenCvCameraView.setSlamIPCClient(slamIPCClient);
        mOpenCvCameraView.init();

        initFinished = false;

        touchHelper = new TouchHelper(this);

        initGLES20Model();

        View touchView = findViewById(R.id.touch_panel);
        touchView.setClickable(true);
        touchView.setOnTouchListener((v, event) -> {
            boolean handled = touchHelper.handleTouchEvent(event);
            if (event.getAction() == MotionEvent.ACTION_UP) {
                v.performClick();
            }
            return handled;
        });

        touchView.setOnClickListener(v -> {
            if (mOpenCvCameraView != null) {
                Log.d(TAG, "onClick: CameraX 自动对焦");
                mOpenCvCameraView.autoFocusCenter();
            }
        });

        btnStartWeb = findViewById(R.id.btn_start_web);
        if (btnStartWeb != null) {
            btnStartWeb.setOnClickListener(v -> toggleWebServer());
        }

        fpsText = findViewById(R.id.text_fps);
        textMapStats = findViewById(R.id.text_map_stats);
        mFpsMeter = new FpsMeter();

        startMapStatsUpdater();

        btnCreateArObject = findViewById(R.id.btn_create_ar_object);
        btnSaveMap = findViewById(R.id.btn_save_map);
        btnLoadMap = findViewById(R.id.btn_load_map);
        btnMapList = findViewById(R.id.btn_map_list);

        btnCreateArObject.setOnClickListener(v -> {
            Log.d(TAG, "点击按钮：创建AR物体");
            showHint(getString(R.string.hint_request_sent));
            if (mOpenCvCameraView != null) {
                mOpenCvCameraView.requestPlaneDetection();
            }
        });

        btnSaveMap.setOnClickListener(v -> showSaveMapDialog());
        btnLoadMap.setOnClickListener(v -> showMapListDialog(false));

        if (btnMapList != null) {
            btnMapList.setOnClickListener(v -> showMapListDialog(true));
        }

        btnTogglePointCloud = findViewById(R.id.btn_toggle_pointcloud);
        if (btnTogglePointCloud != null) {
            btnTogglePointCloud.setOnClickListener(v -> togglePointCloudDisplay());
        }

        floatingQrWindow = findViewById(R.id.floating_qr_window);
        ivQrCode = findViewById(R.id.iv_qr_code);
        tvWebUrl = findViewById(R.id.tv_web_url);
        View qrHeader = findViewById(R.id.qr_window_header);
        View btnCloseQr = findViewById(R.id.btn_close_qr);

        if (btnCloseQr != null) {
            btnCloseQr.setOnClickListener(v -> {
                if (isWebRunning) {
                    toggleWebServer();
                } else {
                    if (floatingQrWindow != null)
                        floatingQrWindow.setVisibility(View.GONE);
                }
            });
        }

        if (qrHeader != null && floatingQrWindow != null) {
            qrHeader.setOnTouchListener((view, event) -> {
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        qrDX = floatingQrWindow.getX() - event.getRawX();
                        qrDY = floatingQrWindow.getY() - event.getRawY();
                        break;
                    case MotionEvent.ACTION_MOVE:
                        floatingQrWindow.animate()
                                .x(event.getRawX() + qrDX)
                                .y(event.getRawY() + qrDY)
                                .setDuration(0)
                                .start();
                        break;
                    case MotionEvent.ACTION_UP:
                        view.performClick();
                        return false;
                    default:
                        return false;
                }
                return true;
            });
        }

        Button btnGroupAr = findViewById(R.id.btn_group_ar);
        if (btnGroupAr != null) {
            btnGroupAr.setOnClickListener(v -> toggleExclusive(R.id.group_ar));
        }
        Button btnGroupMap = findViewById(R.id.btn_group_map);
        if (btnGroupMap != null) {
            btnGroupMap.setOnClickListener(v -> toggleExclusive(R.id.group_map));
        }
        Button btnGroupSlam = findViewById(R.id.btn_group_slam);
        if (btnGroupSlam != null) {
            btnGroupSlam.setOnClickListener(v -> toggleExclusive(R.id.group_slam));
        }
        Button btnGroupDisplay = findViewById(R.id.btn_group_display);
        if (btnGroupDisplay != null) {
            btnGroupDisplay.setOnClickListener(v -> toggleExclusive(R.id.group_display));
        }

        btnToggleSlam = findViewById(R.id.btn_toggle_slam);
        if (btnToggleSlam != null) {
            btnToggleSlam.setOnClickListener(v -> toggleSLAM());
        }

        btn3DofCube = findViewById(R.id.btn_3dof_cube);
        if (btn3DofCube != null) {
            btn3DofCube.setOnClickListener(v -> spawn3DofCube());
        }

        initJoystick();
        init3DofSensor();
    }

    private void toggleExclusive(int groupId) {
        View ga = findViewById(R.id.group_ar);
        View gm = findViewById(R.id.group_map);
        View gs = findViewById(R.id.group_slam);
        View gd = findViewById(R.id.group_display);
        View target = findViewById(groupId);
        boolean visible = target != null && target.getVisibility() == View.VISIBLE;
        if (ga != null) ga.setVisibility(View.GONE);
        if (gm != null) gm.setVisibility(View.GONE);
        if (gs != null) gs.setVisibility(View.GONE);
        if (gd != null) gd.setVisibility(View.GONE);
        if (!visible && target != null) target.setVisibility(View.VISIBLE);
    }

    private void showSaveMapDialog() {
        final android.widget.EditText input = new android.widget.EditText(this);
        input.setHint(getString(R.string.input_map_name));

        String defaultName = "map_" + new java.text.SimpleDateFormat("MMdd_HHmm",
                java.util.Locale.getDefault()).format(new java.util.Date());
        input.setText(defaultName);

        new androidx.appcompat.app.AlertDialog.Builder(this)
                .setTitle(getString(R.string.dialog_save_map))
                .setView(input)
                .setPositiveButton(getString(R.string.btn_save), (dialog, which) -> {
                    String mapName = input.getText().toString().trim();
                    if (mapName.isEmpty()) mapName = defaultName;

                    final String resDir = getExternalFilesDir("SLAM").getAbsolutePath() + "/maps/" + mapName + ".bin";
                    if (slamIPCClient != null) {
                        slamIPCClient.saveMap(resDir);
                    }
                    showHint(getString(R.string.hint_map_saved, mapName));
                })
                .setNegativeButton(getString(R.string.button_cancel), null)
                .show();
    }

    private void showMapListDialog(final boolean showManage) {
        final java.util.ArrayList<NativeHelper.MapManager.MapInfo> maps = mapManager.getAllMaps();

        if (maps.isEmpty()) {
            showHint(getString(R.string.hint_no_maps));
            return;
        }

        String[] mapNames = new String[maps.size()];
        for (int i = 0; i < maps.size(); i++) {
            NativeHelper.MapManager.MapInfo info = maps.get(i);
            mapNames[i] = info.name + "\n" +
                    getString(R.string.map_stats_keyframes, info.keyFrames) + " | " +
                    getString(R.string.map_stats_mappoints, info.mapPoints) + " | " +
                    getString(R.string.map_stats_size, info.fileSize / 1024);
        }

        if (!showManage) {
            final boolean[] checkedItems = new boolean[maps.size()];
            new androidx.appcompat.app.AlertDialog.Builder(this)
                    .setTitle(getString(R.string.dialog_select_map))
                    .setMultiChoiceItems(mapNames, checkedItems, (dialog, which, isChecked) -> checkedItems[which] = isChecked)
                    .setPositiveButton(getString(R.string.action_load), (dialog, which) -> {
                        int loadedCount = 0;
                        for (int i = 0; i < maps.size(); i++) {
                            if (checkedItems[i]) {
                                boolean append = (loadedCount > 0);
                                final String resDir = getExternalFilesDir("SLAM").getAbsolutePath() + "/maps/" + maps.get(i).name + ".bin";
                                if (slamIPCClient != null) {
                                    slamIPCClient.loadMapWithId(resDir, loadedCount, append);
                                }
                                loadedCount++;
                            }
                        }
                        if (loadedCount > 0) {
                            showHint(getResources().getQuantityString(R.plurals.hint_maps_loaded, loadedCount, loadedCount));
                        }
                    })
                    .setNeutralButton(getString(R.string.button_cancel), null)
                    .show();
        } else {
            new androidx.appcompat.app.AlertDialog.Builder(this)
                    .setTitle(getString(R.string.dialog_map_manage))
                    .setItems(mapNames, (dialog, which) -> {
                        final NativeHelper.MapManager.MapInfo selectedMap = maps.get(which);
                        showMapOptionsDialog(selectedMap);
                    })
                    .setNegativeButton(getString(R.string.button_cancel), null)
                    .show();
        }
    }

    private void showMapOptionsDialog(final NativeHelper.MapManager.MapInfo mapInfo) {
        String[] options = { getString(R.string.action_load), getString(R.string.action_delete),
                getString(R.string.action_view_details) };
        new androidx.appcompat.app.AlertDialog.Builder(this)
                .setTitle(mapInfo.name)
                .setItems(options, (dialog, which) -> {
                    switch (which) {
                        case 0:
                            final String resDir = getExternalFilesDir("SLAM").getAbsolutePath() + "/maps/" + mapInfo.name + ".bin";
                            if (slamIPCClient != null) {
                                slamIPCClient.loadMapWithId(resDir, 0, false);
                            }
                            showHint(getString(R.string.hint_map_loaded, mapInfo.name));
                            break;
                        case 1:
                            new androidx.appcompat.app.AlertDialog.Builder(ArCamUIActivity.this)
                                    .setTitle(getString(R.string.dialog_confirm_delete))
                                    .setMessage(getString(R.string.dialog_confirm_delete_message, mapInfo.name))
                                    .setPositiveButton(getString(R.string.action_delete), (d, w) -> {
                                        if (mapManager.deleteMap(mapInfo.name)) {
                                            showHint(getString(R.string.hint_map_deleted));
                                        } else {
                                            showHint(getString(R.string.hint_map_delete_failed));
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

    private void showMapDetails(NativeHelper.MapManager.MapInfo mapInfo) {
        String details = getString(R.string.map_details_name, mapInfo.name) + "\n" +
                getString(R.string.map_details_keyframes, mapInfo.keyFrames) + "\n" +
                getString(R.string.map_details_mappoints, mapInfo.mapPoints) + "\n" +
                getString(R.string.map_details_size, mapInfo.fileSize / 1024) + "\n" +
                getString(R.string.map_details_time, new java.text.SimpleDateFormat("yyyy-MM-dd HH:mm:ss",
                        java.util.Locale.getDefault()).format(new java.util.Date(mapInfo.createTime)))
                + "\n" +
                getString(R.string.map_details_plane, mapInfo.hasPlane ? getString(R.string.map_details_plane_yes)
                        : getString(R.string.map_details_plane_no));

        new androidx.appcompat.app.AlertDialog.Builder(this)
                .setTitle(getString(R.string.dialog_map_details))
                .setMessage(details)
                .setPositiveButton(getString(R.string.button_ok), null)
                .show();
    }

    private void initGLES20Model() {
        Log.d(TAG, "initGLES20Model: 初始化GLB模型渲染器");

        final FilamentAspectSurfaceView glRootView = findViewById(R.id.ar_object_view_gles2_obj);
        glRootView.setAspectRatio(GlobalConstant.RESOLUTION_WIDTH, GlobalConstant.RESOLUTION_HEIGHT);

        modelRendererWrapper = ModelRendererWrapper.newInstance()
                .setArObjectView(glRootView)
                .setNativeHelper(nativeHelper)
                .setSlamIPCClient(slamIPCClient)
                .setContext(this)
                .setModelPath("model.glb")
                .setInitSize(0.20f)
                .setDrawStateListener(shouldDraw -> runOnUiThread(() -> {
                    if (joystickView != null) {
                        joystickView.setVisibility(shouldDraw ? View.VISIBLE : View.GONE);
                    }
                }))
                .init(touchHelper);
    }

    @Override
    protected void onPause() {
        Log.d(TAG, "onPause: 暂停摄像头视图");
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();

        if (is3DofMode && orientationSensor != null) {
            orientationSensor.stop();
            if (threeDofGLView != null) {
                threeDofGLView.onPause();
            }
        }

        if (isWebRunning && webServer != null) {
            webServer.stop();
        }

        if (slamIPCClient != null) {
            slamIPCClient.unbindService();
        }
    }

    @Override
    protected void onResume() {
        Log.d(TAG, "onResume: 准备启动");
        super.onResume();

        if (slamIPCClient != null) {
            slamIPCClient.bindService();
        }

        if (!initFinished) {
            initFinished = true;
            initSLAMAsync();
        } else {
            mOpenCvCameraView.enableView();
        }

        if (is3DofMode && orientationSensor != null) {
            orientationSensor.start(this);
            if (threeDofGLView != null) {
                threeDofGLView.onResume();
            }
        }

        if (isWebRunning && webServer != null) {
            webServer.start();
        }
    }

    private void initSLAMAsync() {
        showLoadingDialog(getString(R.string.loading_slam_init), getString(R.string.loading_slam_wait));

        new Thread(() -> {
            try {
                final String resDir = getExternalFilesDir("SLAM").getAbsolutePath() + "/";
                Log.d(TAG, "SLAM资源目录: " + resDir);
                Log.d(TAG, "绑定 SLAM 独立进程服务...");

                if (slamIPCClient != null) {
                    slamIPCClient.bindService();
                    slamIPCClient.initSLAM(resDir);
                }

                runOnUiThread(() -> {
                    dismissLoadingDialog();
                    showHint(getString(R.string.slam_init_complete));
                    mOpenCvCameraView.enableView();
                });

            } catch (final Exception e) {
                Log.e(TAG, "SLAM初始化失败: " + e.getMessage(), e);
                runOnUiThread(() -> {
                    dismissLoadingDialog();
                    showHint(getString(R.string.slam_init_failed, e.getMessage()));
                });
            }
        }).start();
    }

    private void showLoadingDialog(String title, String message) {
        runOnUiThread(() -> {
            if (loadingDialog != null && loadingDialog.isShowing()) {
                loadingDialog.dismiss();
            }

            android.widget.LinearLayout container = new android.widget.LinearLayout(ArCamUIActivity.this);
            container.setOrientation(android.widget.LinearLayout.HORIZONTAL);
            int padding = (int) (16 * getResources().getDisplayMetrics().density);
            container.setPadding(padding, padding, padding, padding);

            android.widget.ProgressBar progressBar = new android.widget.ProgressBar(ArCamUIActivity.this);
            progressBar.setIndeterminate(true);

            android.widget.TextView msgView = new android.widget.TextView(ArCamUIActivity.this);
            msgView.setText(message);
            msgView.setTextColor(0xFF000000);
            msgView.setTextSize(16);
            android.widget.LinearLayout.LayoutParams textLp = new android.widget.LinearLayout.LayoutParams(
                    android.widget.LinearLayout.LayoutParams.WRAP_CONTENT,
                    android.widget.LinearLayout.LayoutParams.WRAP_CONTENT);
            textLp.leftMargin = padding / 2;
            container.addView(progressBar);
            container.addView(msgView, textLp);

            loadingDialog = new androidx.appcompat.app.AlertDialog.Builder(ArCamUIActivity.this)
                    .setTitle(title)
                    .setView(container)
                    .setCancelable(false)
                    .create();
            loadingDialog.show();
        });
    }

    private void dismissLoadingDialog() {
        runOnUiThread(() -> {
            if (loadingDialog != null && loadingDialog.isShowing()) {
                loadingDialog.dismiss();
                loadingDialog = null;
            }
        });
    }

    @Override
    protected void onDestroy() {
        Log.d(TAG, "onDestroy: 释放资源");

        if (webServer != null) {
            webServer.stop();
        }

        if (modelRendererWrapper != null) {
            modelRendererWrapper.destroy();
            modelRendererWrapper = null;
        }

        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();

        if (slamIPCClient != null) {
            slamIPCClient.unbindService();
        }

        dismissLoadingDialog();
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        Log.d(TAG, "onCameraViewStarted: 摄像头视图启动，宽度=" + width + " 高度=" + height);
        if (slamIPCClient != null) {
            slamIPCClient.updateResolution(width, height);
        }
    }

    @Override
    public void onCameraViewStopped() {
        Log.d(TAG, "onCameraViewStopped: 摄像头视图停止");
    }

    @Override
    public long onCameraFrame(CameraGLViewBase.CvCameraViewFrame inputFrame) {
        mFpsMeter.measure();
        runOnUiThread(() -> fpsText.setText(mFpsMeter.getText()));
        return 0;
    }

    private void showHint(final String str) {
        runOnUiThread(() -> Toast.makeText(ArCamUIActivity.this, str, Toast.LENGTH_LONG).show());
    }

    private void startMapStatsUpdater() {
        final Runnable updater = new Runnable() {
            @Override
            public void run() {
                if (slamIPCClient != null && textMapStats != null) {
                    int[] stats = slamIPCClient.getMapStats();
                    if (stats != null && stats.length == 3) {
                        final String statsText = getString(R.string.map_stats_format,
                                stats[0], stats[1], stats[2] > 0 ? getString(R.string.map_stats_plane_yes)
                                        : getString(R.string.map_stats_plane_no));
                        runOnUiThread(() -> textMapStats.setText(statsText));
                    }
                }
                uiHandler.postDelayed(this, 1000);
            }
        };
        uiHandler.postDelayed(updater, 1000);
    }

    private void initJoystick() {
        joystickView = findViewById(R.id.joystick_view);
        if (joystickView != null) {
            joystickView.setOnJoystickListener((angleDeg, intensity) -> {
                if (modelRendererWrapper != null && intensity > 0.01f) {
                    float speed = 3.0f * intensity;
                    float yawDelta = (float) Math.cos(Math.toRadians(angleDeg)) * speed;
                    float pitchDelta = -(float) Math.sin(Math.toRadians(angleDeg)) * speed;
                    modelRendererWrapper.addUserRotation(yawDelta, pitchDelta);
                }
            });
            Log.d(TAG, "摇杆初始化完成");
        }
    }

    private void togglePointCloudDisplay() {
        if (slamIPCClient != null && btnTogglePointCloud != null) {
            boolean currentState = slamIPCClient.isPointCloudDisplayEnabled();
            boolean newState = !currentState;
            slamIPCClient.setPointCloudDisplay(newState);

            if (newState) {
                btnTogglePointCloud.setText(getString(R.string.btn_pointcloud_enabled));
                showHint(getString(R.string.hint_pointcloud_enabled));
                Log.d(TAG, "点云显示已启用");
            } else {
                btnTogglePointCloud.setText(getString(R.string.btn_pointcloud_disabled));
                showHint(getString(R.string.hint_pointcloud_disabled));
                Log.d(TAG, "点云显示已禁用");
            }
        } else {
            Log.e(TAG, "无法切换点云显示：SlamIPCClient 未连接");
        }
    }

    private void toggleSLAM() {
        if (slamIPCClient != null && btnToggleSlam != null) {
            boolean currentState = slamIPCClient.isEnableSLAM();
            boolean newState = !currentState;
            slamIPCClient.setEnableSLAM(newState);

            if (newState) {
                btnToggleSlam.setText(getString(R.string.btn_slam));
                showHint(getString(R.string.hint_slam_enabled));
                Log.d(TAG, "SLAM已启用");
            } else {
                btnToggleSlam.setText(getString(R.string.btn_slam_disabled));
                showHint(getString(R.string.hint_slam_disabled));
                Log.d(TAG, "SLAM已关闭");
            }
        } else {
            Log.e(TAG, "无法切换SLAM：SlamIPCClient 未连接");
        }
    }

    private void init3DofSensor() {
        orientationSensor = new OrientationSensor();

        if (!orientationSensor.hasRequiredSensors(this)) {
            Log.w(TAG, "设备缺少3DOF所需的传感器");
            if (btn3DofCube != null) {
                btn3DofCube.setEnabled(false);
                btn3DofCube.setAlpha(0.5f);
            }
            return;
        }

        threeDofGLView = new GLSurfaceView(this);
        threeDofGLView.setEGLContextClientVersion(2);
        threeDofGLView.setEGLConfigChooser(8, 8, 8, 8, 16, 0);
        threeDofGLView.getHolder().setFormat(android.graphics.PixelFormat.TRANSLUCENT);
        threeDofGLView.setZOrderOnTop(true);

        threeDofRenderer = new ThreeDofCubeRenderer(this, orientationSensor, slamIPCClient);
        threeDofGLView.setRenderer(threeDofRenderer);
        threeDofGLView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);

        android.widget.RelativeLayout.LayoutParams params = new android.widget.RelativeLayout.LayoutParams(
                android.widget.RelativeLayout.LayoutParams.MATCH_PARENT,
                android.widget.RelativeLayout.LayoutParams.MATCH_PARENT);
        params.addRule(android.widget.RelativeLayout.CENTER_IN_PARENT);

        android.widget.RelativeLayout rootLayout = (android.widget.RelativeLayout) findViewById(
                R.id.my_fake_glsurface_view).getParent();
        rootLayout.addView(threeDofGLView, 2, params);

        threeDofGLView.setVisibility(View.GONE);

        Log.d(TAG, "3DOF传感器和渲染器初始化完成");
    }

    private void spawn3DofCube() {
        if (orientationSensor == null || threeDofRenderer == null) {
            showHint(getString(R.string.hint_3dof_unavailable));
            return;
        }

        if (!is3DofMode) {
            is3DofMode = true;
            orientationSensor.start(this);
            threeDofGLView.setVisibility(View.VISIBLE);
            threeDofGLView.onResume();

            threeDofRenderer.spawnCubeAtDistance(5.0f);

            if (btn3DofCube != null) {
                btn3DofCube.setText(getString(R.string.btn_3dof_close));
            }
            showHint(getString(R.string.hint_3dof_spawned));
            Log.d(TAG, "3DOF模式已启动");
        } else {
            is3DofMode = false;
            orientationSensor.stop();
            threeDofRenderer.hideCube();
            threeDofGLView.onPause();
            threeDofGLView.setVisibility(View.GONE);

            if (btn3DofCube != null) {
                btn3DofCube.setText(getString(R.string.btn_3dof));
            }
            showHint(getString(R.string.hint_3dof_closed));
            Log.d(TAG, "3DOF模式已关闭");
        }
    }

    private String getDeviceIpAddress() {
        try {
            for (Enumeration<NetworkInterface> en = NetworkInterface.getNetworkInterfaces(); en.hasMoreElements();) {
                NetworkInterface networkInterface = en.nextElement();
                for (Enumeration<InetAddress> enumInetAddress = networkInterface.getInetAddresses(); enumInetAddress
                        .hasMoreElements();) {
                    InetAddress inetAddress = enumInetAddress.nextElement();
                    if (!inetAddress.isLoopbackAddress() && inetAddress instanceof Inet4Address) {
                        return inetAddress.getHostAddress();
                    }
                }
            }
        } catch (Exception e) {
            Log.e(TAG, "获取IP地址失败: " + e.getMessage(), e);
        }
        return "127.0.0.1";
    }

    private Bitmap generateQrCode(String content) {
        try {
            int size = 512;
            BitMatrix bitMatrix = new MultiFormatWriter().encode(content, BarcodeFormat.QR_CODE, size, size);
            int width = bitMatrix.getWidth();
            int height = bitMatrix.getHeight();
            int[] pixels = new int[width * height];
            for (int y = 0; y < height; y++) {
                int offset = y * width;
                for (int x = 0; x < width; x++) {
                    pixels[offset + x] = bitMatrix.get(x, y) ? 0xFF000000 : 0xFFFFFFFF;
                }
            }
            Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
            bitmap.setPixels(pixels, 0, width, 0, 0, width, height);
            return bitmap;
        } catch (WriterException e) {
            Log.e(TAG, "生成二维码失败", e);
            return null;
        }
    }

    private void toggleWebServer() {
        if (!isWebRunning) {
            webServer = new WebServer(8080, nativeHelper, slamIPCClient, this);

            webServer.setOnFrameReceivedListener(frameData -> {
            });

            webServer.start();
            isWebRunning = true;

            if (modelRendererWrapper != null) {
                modelRendererWrapper.setDraw(false);
            }

            btnStartWeb.setText(getString(R.string.btn_web_server_close));

            String ipAddress = getDeviceIpAddress();
            String url = "https://" + ipAddress + ":8080";
            showHint(getString(R.string.hint_web_server_started, url));

            if (floatingQrWindow != null) {
                floatingQrWindow.setVisibility(View.VISIBLE);
                if (tvWebUrl != null) {
                    tvWebUrl.setText(url);
                }
                if (ivQrCode != null) {
                    Bitmap qrBitmap = generateQrCode(url);
                    if (qrBitmap != null) {
                        ivQrCode.setImageBitmap(qrBitmap);
                    }
                }
            }

            Log.d(TAG, "Web服务器已启动");
        } else {
            if (webServer != null) {
                webServer.stop();
            }
            isWebRunning = false;

            if (modelRendererWrapper != null) {
                modelRendererWrapper.setDraw(true);
            }

            if (floatingQrWindow != null) {
                floatingQrWindow.setVisibility(View.GONE);
            }

            btnStartWeb.setText(getString(R.string.btn_web_server_open));
            showHint(getString(R.string.hint_web_server_closed));
            Log.d(TAG, "Web服务器已关闭");
        }
    }
}