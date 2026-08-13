package com.orb.slam2s.ui;

/**
 * Created by Ads on 2017/3/9.
 * 由Olsc于2025/8/25开始进行修改
 */

import android.content.Context;
import android.content.pm.ActivityInfo;
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
import android.widget.TextView;
import android.widget.Toast;
import android.widget.Button;
import android.graphics.Bitmap;

import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.Enumeration;
import com.orb.slam2s.server.WebServer;

import com.orb.slam2s.constant.GlobalConstant;
import com.orb.slam2s.rendering.render.ModelRendererWrapper;
import com.orb.slam2s.rendering.render.ThreeDofCubeRenderer;
import com.orb.slam2s.sensors.OrientationSensor;
import com.orb.slam2s.slamar.NativeHelper;
import com.orb.slam2s.R;
import com.orb.slam2s.rendering.gles.FilamentAspectSurfaceView;
import com.orb.slam2s.utils.FpsMeter;
import com.orb.slam2s.utils.TouchHelper;

import com.orb.slam2s.slamar.OpenCVBridge;

import androidx.appcompat.app.AppCompatActivity;
import androidx.activity.OnBackPressedCallback;

import com.google.zxing.BarcodeFormat;
import com.google.zxing.MultiFormatWriter;
import com.google.zxing.WriterException;
import com.google.zxing.common.BitMatrix;

@SuppressWarnings("deprecation")
public class ArCamUIActivity extends AppCompatActivity implements
        CameraGLViewBase.CvCameraViewListener2 {

    private static final String TAG = "SlamCamActivity";

    private long mRgbaAddr;     // native Mat 地址 (CV_8UC4)
    private long mGrayAddr;     // native Mat 地址 (CV_8UC1)

    private CameraGLView mOpenCvCameraView;
    private boolean initFinished;

    private NativeHelper nativeHelper;
    private NativeHelper.MapManager mapManager;
    private TouchHelper touchHelper;
    private ModelRendererWrapper modelRendererWrapper;

    private boolean detectPlane;

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
    private boolean slamInitialized = false;

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

    // 浏览器图像帧相关 (Web 服务器使用)
    private volatile byte[] browserFrameData = null;
    private final Object browserFrameLock = new Object();
    private boolean useWebCamera = false; // Web模式：使用浏览器相机而不是本地相机
    private Thread webFrameProcessor;
    private volatile boolean isProcessingWebFrames = false;
    private boolean webWaitLogged = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.d(TAG, "onCreate: 初始化Activity");

        // 锁定为当前进入时的横屏方向，不再动态旋转
        lockCurrentOrientation();

        // 根据屏幕尺寸计算最佳分辨率
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

    // 根据屏幕实际分辨率计算最优相机处理分辨率
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

            // 计算最佳相机处理分辨率
            GlobalConstant.computeOptimalResolution(screenWidth, screenHeight);
            Log.d(TAG, "选择相机分辨率: " + GlobalConstant.RESOLUTION_WIDTH + "x" + GlobalConstant.RESOLUTION_HEIGHT);
        }
    }

    // 锁定当前横屏方向，防止后续旋转切换
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

                // 同步更新相机管道中的旋转标志
                GlobalConstant.setDisplayRotation(rotation);

                if (rotation == Surface.ROTATION_270) {
                    // 右横屏 (reverse landscape)
                    setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_REVERSE_LANDSCAPE);
                    Log.d(TAG, "锁定为右横屏方向 (REVERSE_LANDSCAPE)");
                } else {
                    // 默认为左横屏
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

        // 设置全屏与屏幕常亮
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        // 初始化NativeHelper，用于调用本地SLAM库
        nativeHelper = new NativeHelper(this);
        mapManager = new NativeHelper.MapManager(this, nativeHelper);

        // 初始化相机视图
        mOpenCvCameraView = (CameraGLView) findViewById(R.id.my_fake_glsurface_view);
        mOpenCvCameraView.setVisibility(View.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);
        mOpenCvCameraView.init();

        initFinished = false;

        // 触摸帮助类，用于处理手势
        touchHelper = new TouchHelper(this);

        // 初始化GLB模型渲染器
        initGLES20Model();

        // 设置触摸事件响应
        View touchView = findViewById(R.id.touch_panel);
        touchView.setClickable(true);
        touchView.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                boolean handled = touchHelper.handleTouchEvent(event);
                if (event.getAction() == MotionEvent.ACTION_UP) {
                    v.performClick();
                }
                return handled;
            }
        });

        // 点击触发相机自动对焦（Web模式下关闭对焦）
        touchView.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (!useWebCamera && mOpenCvCameraView != null) {
                    Log.d(TAG, "onClick: CameraX 自动对焦");
                    mOpenCvCameraView.autoFocusCenter();
                }
            }
        });

        // Web 服务器按钮
        btnStartWeb = findViewById(R.id.btn_start_web);
        if (btnStartWeb != null) {
            btnStartWeb.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    toggleWebServer();
                }
            });
        }

        // 帧率显示
        fpsText = findViewById(R.id.text_fps);
        textMapStats = findViewById(R.id.text_map_stats);
        mFpsMeter = new FpsMeter();

        // 启动地图状态更新线程
        startMapStatsUpdater();

        btnCreateArObject = findViewById(R.id.btn_create_ar_object);
        btnSaveMap = findViewById(R.id.btn_save_map);
        btnLoadMap = findViewById(R.id.btn_load_map);
        btnMapList = findViewById(R.id.btn_map_list);
        // 创建AR物体按钮（原检测平面功能）
        btnCreateArObject.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.d(TAG, "点击按钮：创建AR物体");
                detectPlane = true;
            }
        });

        btnSaveMap.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                // 显示保存地图对话框
                showSaveMapDialog();
            }
        });
        btnLoadMap.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                // 显示地图列表对话框
                showMapListDialog(false);
            }
        });

        // 添加地图列表按钮
        if (btnMapList != null) {
            btnMapList.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    showMapListDialog(true);
                }
            });
        }

        // 添加点云显示控制按钮
        btnTogglePointCloud = findViewById(R.id.btn_toggle_pointcloud);
        if (btnTogglePointCloud != null) {
            btnTogglePointCloud.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    togglePointCloudDisplay();
                }
            });
        }

        // Web QR Window 初始化
        floatingQrWindow = findViewById(R.id.floating_qr_window);
        ivQrCode = findViewById(R.id.iv_qr_code);
        tvWebUrl = findViewById(R.id.tv_web_url);
        View qrHeader = findViewById(R.id.qr_window_header);
        View btnCloseQr = findViewById(R.id.btn_close_qr);

        if (btnCloseQr != null) {
            btnCloseQr.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    if (isWebRunning) {
                        toggleWebServer();
                    } else {
                        if (floatingQrWindow != null)
                            floatingQrWindow.setVisibility(View.GONE);
                    }
                }
            });
        }

        // QR 窗口拖动功能
        if (qrHeader != null && floatingQrWindow != null) {
            qrHeader.setOnTouchListener(new View.OnTouchListener() {
                @Override
                public boolean onTouch(View view, MotionEvent event) {
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
                }
            });
        }

        // 分类折叠切换
        Button btnGroupAr = findViewById(R.id.btn_group_ar);
        if (btnGroupAr != null) {
            btnGroupAr.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    toggleExclusive(R.id.group_ar);
                }
            });
        }
        Button btnGroupMap = findViewById(R.id.btn_group_map);
        if (btnGroupMap != null) {
            btnGroupMap.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    toggleExclusive(R.id.group_map);
                }
            });
        }
        Button btnGroupSlam = findViewById(R.id.btn_group_slam);
        if (btnGroupSlam != null) {
            btnGroupSlam.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    toggleExclusive(R.id.group_slam);
                }
            });
        }
        Button btnGroupDisplay = findViewById(R.id.btn_group_display);
        if (btnGroupDisplay != null) {
            btnGroupDisplay.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    toggleExclusive(R.id.group_display);
                }
            });
        }

        // 添加 SLAM 开关控制按钮
        btnToggleSlam = findViewById(R.id.btn_toggle_slam);
        if (btnToggleSlam != null) {
            btnToggleSlam.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    toggleSLAM();
                }
            });
        }

        // 添加 3DOF 立方体按钮
        btn3DofCube = findViewById(R.id.btn_3dof_cube);
        if (btn3DofCube != null) {
            btn3DofCube.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    spawn3DofCube();
                }
            });
        }

        // 初始化摇杆控制
        initJoystick();

        // 初始化3DOF传感器
        init3DofSensor();
    }

    private void toggleExclusive(int groupId) {
        View ga = findViewById(R.id.group_ar);
        View gm = findViewById(R.id.group_map);
        View gs = findViewById(R.id.group_slam);
        View gd = findViewById(R.id.group_display);
        View target = findViewById(groupId);
        boolean visible = target != null && target.getVisibility() == View.VISIBLE;
        if (ga != null)
            ga.setVisibility(View.GONE);
        if (gm != null)
            gm.setVisibility(View.GONE);
        if (gs != null)
            gs.setVisibility(View.GONE);
        if (gd != null)
            gd.setVisibility(View.GONE);
        if (!visible && target != null)
            target.setVisibility(View.VISIBLE);
    }

    // 显示保存地图对话框
    private void showSaveMapDialog() {
        final android.widget.EditText input = new android.widget.EditText(this);
        input.setHint(getString(R.string.input_map_name));

        // 生成默认地图名
        String defaultName = "map_" + new java.text.SimpleDateFormat("MMdd_HHmm",
                java.util.Locale.getDefault()).format(new java.util.Date());
        input.setText(defaultName);

        new androidx.appcompat.app.AlertDialog.Builder(this)
                .setTitle(getString(R.string.dialog_save_map))
                .setView(input)
                .setPositiveButton(getString(R.string.btn_save), new android.content.DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(android.content.DialogInterface dialog, int which) {
                        String mapName = input.getText().toString().trim();
                        if (mapName.isEmpty())
                            mapName = defaultName;
                        mapManager.saveMap(mapName);
                    }
                })
                .setNegativeButton(getString(R.string.button_cancel), null)
                .show();
    }

    // 显示地图列表对话框
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
            // 加载模式：支持多选
            final boolean[] checkedItems = new boolean[maps.size()];
            new androidx.appcompat.app.AlertDialog.Builder(this)
                    .setTitle(getString(R.string.dialog_select_map))
                    .setMultiChoiceItems(mapNames, checkedItems,
                            new android.content.DialogInterface.OnMultiChoiceClickListener() {
                                @Override
                                public void onClick(android.content.DialogInterface dialog, int which,
                                        boolean isChecked) {
                                    checkedItems[which] = isChecked;
                                }
                            })
                    .setPositiveButton(getString(R.string.action_load),
                            new android.content.DialogInterface.OnClickListener() {
                                @Override
                                public void onClick(android.content.DialogInterface dialog, int which) {
                                    int loadedCount = 0;
                                    for (int i = 0; i < maps.size(); i++) {
                                        if (checkedItems[i]) {
                                            // 第一个地图清空旧数据，后续地图追加
                                            boolean append = (loadedCount > 0);
                                            // 使用递增的ID：0, 1, 2...
                                            mapManager.loadMapWithId(maps.get(i).name, loadedCount, append);
                                            loadedCount++;
                                        }
                                    }
                                    if (loadedCount > 0) {
                                        showHint(getResources().getQuantityString(R.plurals.hint_maps_loaded, loadedCount, loadedCount));
                                    }
                                }
                            })
                    .setNeutralButton(getString(R.string.button_cancel), null)
                    .show();
        } else {
            // 管理模式：单选操作
            new androidx.appcompat.app.AlertDialog.Builder(this)
                    .setTitle(getString(R.string.dialog_map_manage))
                    .setItems(mapNames, new android.content.DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(android.content.DialogInterface dialog, int which) {
                            final NativeHelper.MapManager.MapInfo selectedMap = maps.get(which);
                            showMapOptionsDialog(selectedMap);
                        }
                    })
                    .setNegativeButton(getString(R.string.button_cancel), null)
                    .show();
        }
    }

    // 显示地图操作对话框
    private void showMapOptionsDialog(final NativeHelper.MapManager.MapInfo mapInfo) {
        String[] options = { getString(R.string.action_load), getString(R.string.action_delete),
                getString(R.string.action_view_details) };
        new androidx.appcompat.app.AlertDialog.Builder(this)
                .setTitle(mapInfo.name)
                .setItems(options, new android.content.DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(android.content.DialogInterface dialog, int which) {
                        switch (which) {
                            case 0: // 加载
                                mapManager.loadMap(mapInfo.name);
                                break;
                            case 1: // 删除
                                new androidx.appcompat.app.AlertDialog.Builder(ArCamUIActivity.this)
                                        .setTitle(getString(R.string.dialog_confirm_delete))
                                        .setMessage(getString(R.string.dialog_confirm_delete_message, mapInfo.name))
                                        .setPositiveButton(getString(R.string.action_delete),
                                                new android.content.DialogInterface.OnClickListener() {
                                                    @Override
                                                    public void onClick(android.content.DialogInterface dialog,
                                                            int which) {
                                                        if (mapManager.deleteMap(mapInfo.name)) {
                                                            showHint(getString(R.string.hint_map_deleted));
                                                        } else {
                                                            showHint(getString(R.string.hint_map_delete_failed));
                                                        }
                                                    }
                                                })
                                        .setNegativeButton(getString(R.string.button_cancel), null)
                                        .show();
                                break;
                            case 2: // 查看详情
                                showMapDetails(mapInfo);
                                break;
                        }
                    }
                })
                .setNegativeButton(getString(R.string.button_back), null)
                .show();
    }

    // 显示地图详情
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
                .setContext(this)
                .setModelPath("model.glb")
                .setInitSize(0.20f)
                .setDrawStateListener(new ModelRendererWrapper.DrawStateListener() {
                    @Override
                    public void onDrawStateChanged(boolean shouldDraw) {
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                if (joystickView != null) {
                                    joystickView.setVisibility(shouldDraw ? View.VISIBLE : View.GONE);
                                }
                            }
                        });
                    }
                })
                .init(touchHelper);

        nativeHelper.addOnMVPUpdatedCallback(modelRendererWrapper);
    }

    @Override
    protected void onPause() {
        Log.d(TAG, "onPause: 暂停摄像头视图");
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();

        // 暂停3DOF
        if (is3DofMode && orientationSensor != null) {
            orientationSensor.stop();
            if (threeDofGLView != null) {
                threeDofGLView.onPause();
            }
        }

        // 暂停Web服务：避免退后台后线程继续处理
        stopWebFrameProcessing();
        if (isWebRunning) {
            if (webServer != null) {
                webServer.stop();
            }
        }
    }

    @Override
    protected void onResume() {
        Log.d(TAG, "onResume: 准备启动");
        super.onResume();

        if (!initFinished) {
            initFinished = true;
            // 先初始化SLAM，成功后再启动相机
            initSLAMAsync();
        } else {
            // 已经初始化过，直接启动相机
            if (slamInitialized) {
                Log.d(TAG, "onResume: SLAM已初始化，启动摄像头");
                mOpenCvCameraView.enableView();
            }
        }

        // 恢复3DOF
        if (is3DofMode && orientationSensor != null) {
            orientationSensor.start(this);
            if (threeDofGLView != null) {
                threeDofGLView.onResume();
            }
        }

        // 恢复Web服务
        if (isWebRunning) {
            if (webServer != null) {
                webServer.start();
            }
        }
    }

    // 异步初始化 SLAM 系统，后台线程加载词汇表，避免阻塞主线程
    private void initSLAMAsync() {
        // 先显示加载对话框
        showLoadingDialog(getString(R.string.loading_slam_init), getString(R.string.loading_slam_wait));

        // 使用后台线程加载SLAM，避免阻塞主线程
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    final String resDir = getExternalFilesDir("SLAM").getAbsolutePath() + "/";
                    Log.d(TAG, "SLAM资源目录: " + resDir);
                    Log.d(TAG, "开始初始化SLAM（后台线程）...");

                    // 耗时操作：初始化SLAM（词汇表加载约1秒）
                    nativeHelper.initSLAM(resDir);

                    slamInitialized = true;

                    // 在主线程更新UI并启动相机
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            dismissLoadingDialog();
                            showHint(getString(R.string.slam_init_complete));

                            // SLAM初始化成功后，才启动相机
                            Log.d(TAG, "SLAM初始化完成，启动摄像头视图");
                            mOpenCvCameraView.enableView();
                        }
                    });

                } catch (final Exception e) {
                    Log.e(TAG, "SLAM初始化失败: " + e.getMessage(), e);
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            dismissLoadingDialog();
                            showHint(getString(R.string.slam_init_failed, e.getMessage()));
                        }
                    });
                }
            }
        }).start();
    }

    // 显示加载对话框（AlertDialog + ProgressBar）
    private void showLoadingDialog(String title, String message) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
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
            }
        });
    }

    // 关闭加载对话框
    private void dismissLoadingDialog() {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (loadingDialog != null && loadingDialog.isShowing()) {
                    loadingDialog.dismiss();
                    loadingDialog = null;
                }
            }
        });
    }

    @Override
    protected void onDestroy() {
        Log.d(TAG, "onDestroy: 释放资源");

        stopWebFrameProcessing();
        if (webServer != null) {
            webServer.stop();
        }

        // 销毁 Filament 资源
        if (modelRendererWrapper != null) {
            modelRendererWrapper.destroy();
            modelRendererWrapper = null;
        }

        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();

        // 清理加载对话框，防止内存泄漏
        dismissLoadingDialog();
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        Log.d(TAG, "onCameraViewStarted: 摄像头视图启动，宽度=" + width + " 高度=" + height);
        // 通知native层更新内参和投影矩阵
        if (nativeHelper != null) {
            nativeHelper.updateResolution(width, height);
        }
    }

    @Override
    public void onCameraViewStopped() {
        Log.d(TAG, "onCameraViewStopped: 摄像头视图停止");
        mRgbaAddr = 0;
        mGrayAddr = 0;
    }

    @Override
    public long onCameraFrame(CameraGLViewBase.CvCameraViewFrame inputFrame) {
        // Web模式：本地相机停止对 SLAM 的驱动，仅维持图形预览/占位，SLAM 由后台 webFrameProcessor 处理
        if (useWebCamera) {
            mRgbaAddr = inputFrame.rgba();
            return mRgbaAddr;
        }

        // 传统模式：使用本地相机进行SLAM
        mRgbaAddr = inputFrame.rgba();
        mGrayAddr = inputFrame.gray();

        // 确保SLAM已经初始化完成才处理帧
        if (initFinished && slamInitialized) {
            int trackingResult = nativeHelper.processCameraFrame(mGrayAddr, mRgbaAddr);

            if (detectPlane) {
                showHint(getString(R.string.hint_request_sent));
                Log.d(TAG, "onCameraFrame: 请求平面检测");
                int detectResult = nativeHelper.detectPlane();
                detectPlane = false;
                Log.d(TAG, "detectPlane 结果: " + detectResult);
            }
        }

        mFpsMeter.measure();
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                fpsText.setText(mFpsMeter.getText());
            }
        });
        return mRgbaAddr;
    }

    // 启动Web图像处理线程
    private void startWebFrameProcessing() {
        if (webFrameProcessor != null && webFrameProcessor.isAlive()) {
            return; // 已经在运行
        }

        isProcessingWebFrames = true;
        webWaitLogged = false;
        webFrameProcessor = new Thread(new Runnable() {
            @Override
            public void run() {
                Log.d(TAG, "Web图像处理线程已启动");
                long webRgbaAddr = 0;
                long webGrayAddr = 0;

                while (isProcessingWebFrames) {
                    try {
                        byte[] frameData = null;

                        // 获取最新的浏览器图像数据
                        synchronized (browserFrameLock) {
                            if (browserFrameData != null) {
                                frameData = browserFrameData.clone();
                            }
                        }

                        if (frameData != null && slamInitialized) {
                            // 解码JPEG图像
                            android.graphics.Bitmap browserBitmap = android.graphics.BitmapFactory.decodeByteArray(
                                    frameData, 0, frameData.length);

                            if (browserBitmap != null) {
                                // 仅在尺寸变化或未创建时分配 native Mat
                                if (webRgbaAddr == 0) {
                                    webRgbaAddr = OpenCVBridge.nativeCreateMat(
                                            browserBitmap.getHeight(), browserBitmap.getWidth(), OpenCVBridge.CV_8UC4);
                                    webGrayAddr = OpenCVBridge.nativeCreateMat(
                                            browserBitmap.getHeight(), browserBitmap.getWidth(), OpenCVBridge.CV_8UC1);
                                }

                                // 将 Bitmap 转为 native Mat
                                OpenCVBridge.nativeBitmapToMat(browserBitmap, webRgbaAddr);
                                OpenCVBridge.nativeRGBA2Gray(webRgbaAddr, webGrayAddr);
                                browserBitmap.recycle();

                                // 处理SLAM
                                int trackingResult = nativeHelper.processCameraFrame(
                                        webGrayAddr, webRgbaAddr);

                                // 更新FPS
                                mFpsMeter.measure();
                                final String fpsTextStr = mFpsMeter.getText() + getString(R.string.web_fps_label);
                                runOnUiThread(new Runnable() {
                                    @Override
                                    public void run() {
                                        ArCamUIActivity.this.fpsText.setText(fpsTextStr);
                                    }
                                });
                            } else {
                                Log.e(TAG, "Web线程：JPEG解码失败!");
                            }
                        } else {
                            if (!slamInitialized && !webWaitLogged) {
                                webWaitLogged = true; // 只打印一次，避免初始化期间刷屏
                                Log.w(TAG, "Web线程：等待SLAM初始化...");
                            }
                            // 没有数据时等待
                            final String waitText = getString(R.string.hint_waiting_browser_frame);
                            runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    ArCamUIActivity.this.fpsText.setText(waitText);
                                }
                            });
                        }

                        // 控制处理频率（约30fps）
                        Thread.sleep(33);

                    } catch (Exception e) {
                        Log.e(TAG, "Web图像处理错误", e);
                        try {
                            Thread.sleep(100);
                        } catch (InterruptedException ie) {
                            break;
                        }
                    }
                }

                // 清理 native Mat 资源
                if (webRgbaAddr != 0)
                    OpenCVBridge.nativeReleaseMat(webRgbaAddr);
                if (webGrayAddr != 0)
                    OpenCVBridge.nativeReleaseMat(webGrayAddr);
                Log.d(TAG, "Web图像处理线程已停止");
            }
        });
        webFrameProcessor.start();
    }

    // 停止Web图像处理线程
    private void stopWebFrameProcessing() {
        isProcessingWebFrames = false;
        if (webFrameProcessor != null) {
            webFrameProcessor.interrupt();
            try {
                webFrameProcessor.join(1000); // 等待最多1秒
            } catch (InterruptedException e) {
                Log.e(TAG, "等待Web图像处理线程结束时被中断", e);
            }
            webFrameProcessor = null;
        }
    }

    // 显示提示信息（UI线程）
    private void showHint(final String str) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(ArCamUIActivity.this, str, Toast.LENGTH_LONG).show();
            }
        });
    }

    // 启动地图状态更新器
    private void startMapStatsUpdater() {
        final Runnable updater = new Runnable() {
            @Override
            public void run() {
                if (nativeHelper != null && textMapStats != null) {
                    int[] stats = nativeHelper.getMapStats();
                    if (stats != null && stats.length == 3) {
                        final String statsText = getString(R.string.map_stats_format,
                                stats[0], stats[1], stats[2] > 0 ? getString(R.string.map_stats_plane_yes)
                                        : getString(R.string.map_stats_plane_no));
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                textMapStats.setText(statsText);
                            }
                        });
                    }
                }
                uiHandler.postDelayed(this, 1000); // 每秒更新一次
            }
        };
        uiHandler.postDelayed(updater, 1000);
    }

    // 初始化摇杆，用于控制 AR 物体的 Y 轴旋转
    private void initJoystick() {
        joystickView = findViewById(R.id.joystick_view);
        if (joystickView != null) {
            joystickView.setOnJoystickListener(new JoystickView.OnJoystickListener() {
                @Override
                public void onJoystickUpdate(float angleDeg, float intensity) {
                    if (modelRendererWrapper != null && intensity > 0.01f) {
                        // 水平方向(cos)控制Y轴旋转(yaw)，垂直方向(-sin)控制X轴旋转(pitch)
                        float speed = 3.0f * intensity;
                        float yawDelta = (float) Math.cos(Math.toRadians(angleDeg)) * speed;
                        float pitchDelta = -(float) Math.sin(Math.toRadians(angleDeg)) * speed;
                        modelRendererWrapper.addUserRotation(yawDelta, pitchDelta);
                    }
                }
            });
            Log.d(TAG, "摇杆初始化完成");
        }
    }

    // 切换点云显示状态
    private void togglePointCloudDisplay() {
        if (nativeHelper != null && btnTogglePointCloud != null) {
            boolean currentState = nativeHelper.isPointCloudDisplayEnabled();
            boolean newState = !currentState;
            nativeHelper.setPointCloudDisplay(newState);

            // 更新按钮文字
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
            Log.e(TAG, "无法切换点云显示：NativeHelper为null");
        }
    }

    // 切换 SLAM 开关状态
    private void toggleSLAM() {
        if (nativeHelper != null && btnToggleSlam != null) {
            boolean currentState = nativeHelper.isEnableSLAM();
            boolean newState = !currentState;
            nativeHelper.setEnableSLAM(newState);

            // 更新按钮文字和UI反馈
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
            Log.e(TAG, "无法切换SLAM：NativeHelper为null");
        }
    }

    // ========== 3DOF 功能 ==========

    // 初始化 3DOF 传感器和渲染器
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

        // 创建3DOF GLSurfaceView
        threeDofGLView = new GLSurfaceView(this);
        threeDofGLView.setEGLContextClientVersion(2);
        threeDofGLView.setEGLConfigChooser(8, 8, 8, 8, 16, 0);
        threeDofGLView.getHolder().setFormat(android.graphics.PixelFormat.TRANSLUCENT);
        threeDofGLView.setZOrderOnTop(true);

        // 创建渲染器
        threeDofRenderer = new ThreeDofCubeRenderer(this, orientationSensor, nativeHelper);
        threeDofGLView.setRenderer(threeDofRenderer);
        threeDofGLView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);

        // 添加到布局（覆盖在相机视图之上）
        android.widget.RelativeLayout.LayoutParams params = new android.widget.RelativeLayout.LayoutParams(
                android.widget.RelativeLayout.LayoutParams.MATCH_PARENT,
                android.widget.RelativeLayout.LayoutParams.MATCH_PARENT);
        params.addRule(android.widget.RelativeLayout.CENTER_IN_PARENT);

        android.widget.RelativeLayout rootLayout = (android.widget.RelativeLayout) findViewById(
                R.id.my_fake_glsurface_view).getParent();
        rootLayout.addView(threeDofGLView, 2, params); // 插入到相机视图之后

        threeDofGLView.setVisibility(View.GONE); // 默认隐藏

        Log.d(TAG, "3DOF传感器和渲染器初始化完成");
    }

    // 在视角前方 5 米处生成 3DOF 立方体
    private void spawn3DofCube() {
        if (orientationSensor == null || threeDofRenderer == null) {
            showHint(getString(R.string.hint_3dof_unavailable));
            return;
        }

        if (!is3DofMode) {
            // 启动3DOF模式
            is3DofMode = true;
            orientationSensor.start(this);
            threeDofGLView.setVisibility(View.VISIBLE);
            threeDofGLView.onResume();

            // 在前方5米处生成立方体
            threeDofRenderer.spawnCubeAtDistance(5.0f);

            if (btn3DofCube != null) {
                btn3DofCube.setText(getString(R.string.btn_3dof_close));
            }
            showHint(getString(R.string.hint_3dof_spawned));
            Log.d(TAG, "3DOF模式已启动");
        } else {
            // 关闭3DOF模式
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

    // 获取设备 IP 地址，失败时返回本地回环地址
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
        // 如果无法获取IP地址，返回本地回环地址作为备选
        return "127.0.0.1";
    }

    // 生成二维码位图
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
            webServer = new WebServer(8080, nativeHelper, this);

            // 设置接收浏览器图像帧的回调
            webServer.setOnFrameReceivedListener(new WebServer.OnFrameReceivedListener() {
                @Override
                public void onFrameReceived(byte[] frameData) {
                    // 更新浏览器图像数据
                    synchronized (browserFrameLock) {
                        browserFrameData = frameData;
                    }
                }
            });

            webServer.start();
            isWebRunning = true;
            useWebCamera = true; // 切换到Web模式：本地相机关闭硬件，仅处理 Web/浏览端画面
            if (mOpenCvCameraView != null) {
                mOpenCvCameraView.startWebModeTicker();
            }

            // Web模式下隐藏AR物体：浏览器相机SLAM姿态与本地相机坐标系不一致，
            // 继续渲染会导致物体漂浮在错误的空间位置（黑屏+漂浮物体的bug来源）
            if (modelRendererWrapper != null) {
                modelRendererWrapper.setDraw(false);
            }

            // 启动Web图像处理线程
            startWebFrameProcessing();

            btnStartWeb.setText(getString(R.string.btn_web_server_close));

            String ipAddress = getDeviceIpAddress();
            String url = "https://" + ipAddress + ":8080";
            showHint(getString(R.string.hint_web_server_started, url));

            // 显示二维码窗口
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

            Log.d(TAG, "Web服务器已启动，本地处理已停止，仅处理浏览器图像");
        } else {
            // 停止Web图像处理线程
            stopWebFrameProcessing();

            if (webServer != null) {
                webServer.stop();
            }
            isWebRunning = false;
            useWebCamera = false; // 切换回本地相机模式
            if (mOpenCvCameraView != null) {
                mOpenCvCameraView.stopWebModeTicker();
            }

            // 恢复AR物体渲染（本地相机重新驱动SLAM，坐标系恢复一致）
            if (modelRendererWrapper != null) {
                modelRendererWrapper.setDraw(true);
            }

            // 清空浏览器图像数据
            synchronized (browserFrameLock) {
                browserFrameData = null;
            }

            // 隐藏二维码窗口
            if (floatingQrWindow != null) {
                floatingQrWindow.setVisibility(View.GONE);
            }

            btnStartWeb.setText(getString(R.string.btn_web_server_open));
            showHint(getString(R.string.hint_web_server_closed));
            Log.d(TAG, "Web服务器已关闭，本地相机已恢复正常处理");
        }
    }
}