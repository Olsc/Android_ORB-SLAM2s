package com.orb.slam2s.ui;

/**
 * Created by Ads on 2017/3/9.
 * 由Olsc于2025/8/25开始进行修改
 */

import android.app.ActivityManager;
import android.content.pm.PackageManager;
import android.opengl.GLSurfaceView;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.Button;
import android.webkit.WebView;
import android.graphics.Bitmap;
import java.io.ByteArrayOutputStream;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.Enumeration;
import com.orb.slam2s.server.WebServer;
import org.opencv.android.Utils;

import com.orb.slam2s.constant.GlobalConstant;
import com.orb.slam2s.rendering.render.ObjRendererWrapper;
import com.orb.slam2s.rendering.render.ThreeDofCubeRenderer;
import com.orb.slam2s.sensors.OrientationSensor;
import com.orb.slam2s.slamar.NativeHelper;
import com.orb.slam2s.R;
import com.orb.slam2s.rendering.gles.GLRootView;
import com.orb.slam2s.utils.FpsMeter;
import com.orb.slam2s.utils.TouchHelper;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

import androidx.appcompat.app.AppCompatActivity;
import androidx.activity.OnBackPressedCallback;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

@SuppressWarnings("deprecation")
public class ArCamUIActivity extends AppCompatActivity implements
        CameraGLViewBase.CvCameraViewListener2 {

    private static final String TAG = "SlamCamActivity";

    private Mat mRgba;
    private Mat mGray;

    private CameraGLView mOpenCvCameraView;
    private boolean initFinished;

    private NativeHelper nativeHelper;
    private NativeHelper.MapManager mapManager;
    private TouchHelper touchHelper;

    private boolean detectPlane;

    private FpsMeter mFpsMeter = null;
    private TextView fpsText;
    private TextView textMapStats;
    private Button btnCreateArObject;
    private Button btnSaveMap;
    private Button btnLoadMap;
    private Button btnMapList;
    private Button btnResetSlam;
    private Button btnTogglePointCloud;
    private Button btnToggleSlam;
    private Button btnToggleOpticalFlow;
    private Button btnToggleLoopClosing;
    private Button btn3DofCube;
    private android.os.Handler uiHandler = new android.os.Handler();
    private androidx.appcompat.app.AlertDialog loadingDialog;
    private boolean slamInitialized = false;
    private boolean isOpticalFlowEnabled = false;
    private boolean isLoopClosingEnabled = false;

    // 调试界面相关
    private View floatingLogWindow;
    private RecyclerView logRecyclerView;
    private LogAdapter logAdapter;
    private Button btnToggleDebug;
    private LogCatReaderThread logCatThread;
    private boolean isDebugMode = false;

    // 拖动相关变量
    private float dX, dY;

    private WebServer webServer;
    private Button btnStartWeb;
    private boolean isWebRunning = false;
    private Thread pointsUpdater;

    // 3DOF功能相关
    private OrientationSensor orientationSensor;
    private GLSurfaceView threeDofGLView;
    private ThreeDofCubeRenderer threeDofRenderer;
    private boolean is3DofMode = false;

    // 浏览器图像帧相关
    private volatile byte[] browserFrameData = null;
    private final Object browserFrameLock = new Object();
    private boolean useWebCamera = false; // Web模式：使用浏览器相机而不是本地相机
    private Thread webFrameProcessor; // Web图像处理线程
    private volatile boolean isProcessingWebFrames = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.d(TAG, "onCreate: 初始化Activity");
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

        // 初始化渲染器（GLES20渲染OBJ模型）
        initGLES20Obj();

        // 设置触摸事件响应
        View touchView = findViewById(R.id.touch_panel);
        touchView.setClickable(true);
        touchView.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                return touchHelper.handleTouchEvent(event);
            }
        });

        // 点击触发相机自动对焦
        touchView.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.d(TAG, "onClick: CameraX 自动对焦");
                if (mOpenCvCameraView != null) {
                    mOpenCvCameraView.autoFocusCenter();
                }
            }
        });

        // Web Server Button
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
        mFpsMeter.setResolution(GlobalConstant.RESOLUTION_WIDTH, GlobalConstant.RESOLUTION_HEIGHT);

        // 启动地图状态更新线程
        startMapStatsUpdater();

        btnCreateArObject = findViewById(R.id.btn_create_ar_object);
        btnSaveMap = findViewById(R.id.btn_save_map);
        btnLoadMap = findViewById(R.id.btn_load_map);
        btnMapList = findViewById(R.id.btn_map_list);
        btnResetSlam = findViewById(R.id.btn_reset_slam);

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

        // 添加SLAM重置按钮
        if (btnResetSlam != null) {
            btnResetSlam.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    Log.d(TAG, "重置SLAM按钮被点击");
                    resetSLAM();
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

        // 调试按钮与浮动窗口初始化
        floatingLogWindow = findViewById(R.id.floating_log_window);
        logRecyclerView = findViewById(R.id.log_recycler_view);
        View logHeader = findViewById(R.id.log_window_header);
        View btnCloseLogs = findViewById(R.id.btn_close_logs);
        View btnClearLogs = findViewById(R.id.btn_clear_logs);

        if (logRecyclerView != null) {
            logRecyclerView.setLayoutManager(new LinearLayoutManager(this));
            logAdapter = new LogAdapter();
            logRecyclerView.setAdapter(logAdapter);
        }

        if (btnCloseLogs != null) {
            btnCloseLogs.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    toggleDebugMode();
                }
            });
        }

        if (btnClearLogs != null) {
            btnClearLogs.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    if (logAdapter != null)
                        logAdapter.clear();
                }
            });
        }

        // 实现拖动功能
        if (logHeader != null && floatingLogWindow != null) {
            logHeader.setOnTouchListener(new View.OnTouchListener() {
                @Override
                public boolean onTouch(View view, MotionEvent event) {
                    switch (event.getAction()) {
                        case MotionEvent.ACTION_DOWN:
                            dX = floatingLogWindow.getX() - event.getRawX();
                            dY = floatingLogWindow.getY() - event.getRawY();
                            break;
                        case MotionEvent.ACTION_MOVE:
                            floatingLogWindow.animate()
                                    .x(event.getRawX() + dX)
                                    .y(event.getRawY() + dY)
                                    .setDuration(0)
                                    .start();
                            break;
                        default:
                            return false;
                    }
                    return true;
                }
            });
        }

        btnToggleDebug = findViewById(R.id.btn_toggle_debug);
        if (btnToggleDebug != null) {
            btnToggleDebug.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    toggleDebugMode();
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

        // 添加 光流开关控制按钮
        btnToggleOpticalFlow = findViewById(R.id.btn_toggle_optical_flow);
        if (btnToggleOpticalFlow != null) {
            btnToggleOpticalFlow.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    toggleOpticalFlow();
                }
            });
        }

        // 添加 回环开关控制按钮
        btnToggleLoopClosing = findViewById(R.id.btn_toggle_loop_closing);
        if (btnToggleLoopClosing != null) {
            btnToggleLoopClosing.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    toggleLoopClosing();
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
                                        showHint(getString(R.string.hint_maps_loaded, loadedCount));
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

    private void initGLES20Obj() {
        Log.d(TAG, "initGLES20Obj: 初始化OBJ模型渲染器");

        final GLRootView glRootView = findViewById(R.id.ar_object_view_gles2_obj);
        glRootView.setAspectRatio(GlobalConstant.RESOLUTION_WIDTH, GlobalConstant.RESOLUTION_HEIGHT);

        ObjRendererWrapper objRendererWrapper = ObjRendererWrapper.newInstance()
                .setArObjectView(glRootView)
                .setNativeHelper(nativeHelper)
                .setContext(this)
                .setObjPath("model.obj")
                .setTexturePath("model.png")
                .setInitSize(0.20f)
                .init(touchHelper);

        nativeHelper.addOnMVPUpdatedCallback(objRendererWrapper);
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
    }

    /**
     * 异步初始化SLAM系统，完成后再启动相机
     * 在后台线程加载词汇表，避免阻塞主线程和UI冻结
     */
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
                    Log.d(TAG, "SLAM初始化完成");

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

    /**
     * 显示加载对话框（使用非弃用的AlertDialog + ProgressBar）
     */
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

    /**
     * 关闭加载对话框
     */
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

        // 停止Web图像处理线程
        stopWebFrameProcessing();

        if (webServer != null) {
            webServer.stop();
        }
        if (pointsUpdater != null) {
            pointsUpdater.interrupt();
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
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mGray = new Mat(height, width, CvType.CV_8UC1);
    }

    @Override
    public void onCameraViewStopped() {
        Log.d(TAG, "onCameraViewStopped: 摄像头视图停止");
        mRgba.release();
        mGray.release();
    }

    @Override
    public Mat onCameraFrame(CameraGLViewBase.CvCameraViewFrame inputFrame) {
        // Web模式：完全停止本地处理，显示黑屏
        if (useWebCamera) {
            // 创建黑屏Mat（如果还没创建）
            if (mRgba == null || mRgba.empty()) {
                mRgba = inputFrame.rgba();
            }
            // 填充黑色
            mRgba.setTo(new org.opencv.core.Scalar(0, 0, 0, 255));
            // 不更新fpsText，让Web线程来更新
            return mRgba;
        }

        // 传统模式：使用本地相机进行SLAM
        mRgba = inputFrame.rgba();
        mGray = inputFrame.gray();

        // 确保SLAM已经初始化完成才处理帧
        if (initFinished && slamInitialized) {
            int trackingResult = nativeHelper.processCameraFrame(mGray.getNativeObjAddr(), mRgba.getNativeObjAddr());

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
        return mRgba;
    }

    // 启动Web图像处理线程
    private void startWebFrameProcessing() {
        if (webFrameProcessor != null && webFrameProcessor.isAlive()) {
            return; // 已经在运行
        }

        isProcessingWebFrames = true;
        webFrameProcessor = new Thread(new Runnable() {
            @Override
            public void run() {
                Log.d(TAG, "Web图像处理线程已启动");
                Mat webRgba = null;
                Mat webGray = null;

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
                                // 初始化或调整Mat大小
                                if (webRgba == null || webRgba.cols() != browserBitmap.getWidth()
                                        || webRgba.rows() != browserBitmap.getHeight()) {
                                    if (webRgba != null)
                                        webRgba.release();
                                    if (webGray != null)
                                        webGray.release();
                                    webRgba = new Mat(browserBitmap.getHeight(), browserBitmap.getWidth(),
                                            org.opencv.core.CvType.CV_8UC4);
                                    webGray = new Mat(browserBitmap.getHeight(), browserBitmap.getWidth(),
                                            org.opencv.core.CvType.CV_8UC1);
                                }

                                // 转换为OpenCV Mat
                                org.opencv.android.Utils.bitmapToMat(browserBitmap, webRgba);
                                org.opencv.imgproc.Imgproc.cvtColor(webRgba, webGray,
                                        org.opencv.imgproc.Imgproc.COLOR_RGBA2GRAY);
                                browserBitmap.recycle();

                                // 处理SLAM
                                int trackingResult = nativeHelper.processCameraFrame(
                                        webGray.getNativeObjAddr(), webRgba.getNativeObjAddr());

                                // 更新FPS
                                mFpsMeter.measure();
                                final String fpsTextStr = mFpsMeter.getText() + " (Web)";
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
                            if (!slamInitialized) {
                                Log.w(TAG, "Web线程：等待SLAM初始化...");
                            }
                            // 没有数据时等待
                            final String waitText = "等待浏览器图像...";
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

                // 清理资源
                if (webRgba != null)
                    webRgba.release();
                if (webGray != null)
                    webGray.release();
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

    // 重置SLAM系统
    private void resetSLAM() {
        if (nativeHelper != null) {
            Log.i(TAG, "正在重置SLAM系统...");
            showHint(getString(R.string.hint_resetting_slam));

            // 重置平面检测状态
            detectPlane = false;

            // 调用Native层的重置方法
            nativeHelper.resetSLAM();

            showHint(getString(R.string.hint_slam_reset));
            Log.i(TAG, "SLAM系统重置完成");
        } else {
            Log.e(TAG, "无法重置SLAM：NativeHelper为null");
            showHint(getString(R.string.hint_reset_failed));
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

    // 切换调试模式
    private void toggleDebugMode() {
        isDebugMode = !isDebugMode;
        if (isDebugMode) {
            if (floatingLogWindow != null)
                floatingLogWindow.setVisibility(View.VISIBLE);
            if (btnToggleDebug != null)
                btnToggleDebug.setText(getString(R.string.btn_close_debug));
            startLogCatReader();
        } else {
            if (floatingLogWindow != null)
                floatingLogWindow.setVisibility(View.GONE);
            if (btnToggleDebug != null)
                btnToggleDebug.setText(getString(R.string.btn_debug));
            stopLogCatReader();
        }
    }

    private void startLogCatReader() {
        if (logCatThread == null) {
            logCatThread = new LogCatReaderThread();
            logCatThread.start();
        }
    }

    private void stopLogCatReader() {
        if (logCatThread != null) {
            logCatThread.stopReading();
            logCatThread = null;
        }
    }

    // 内部类：读取LogCat日志
    private class LogCatReaderThread extends Thread {
        private volatile boolean running = true;
        private Process logProcess;
        private java.io.BufferedReader reader;
        private final int myPid = android.os.Process.myPid();
        private final String pidStr = String.valueOf(myPid);

        @Override
        public void run() {
            try {
                // 清除之前的日志
                Runtime.getRuntime().exec("logcat -c");

                // 读取当前进程的日志
                // 尝试根据 PID 过滤以减少开销，但为了兼容性保留 Java 层过滤
                String cmd = "logcat -v time";
                logProcess = Runtime.getRuntime().exec(cmd);
                reader = new java.io.BufferedReader(new java.io.InputStreamReader(logProcess.getInputStream()));

                String line;
                while (running && (line = reader.readLine()) != null) {
                    if (!running)
                        break;

                    final String finalLine = line;
                    // 检查是否包含当前 PID，确保不过滤掉关键信息
                    // 许多设备上的 logcat -v time 输出格式为: MM-DD HH:MM:SS.mmm L/Tag(PID): Msg
                    // 因此检查 "(PID)" 或 " PID " 更加准确，但简单的 contains 也通常足够
                    if (finalLine.contains(pidStr)) {
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                updateLogView(finalLine);
                            }
                        });
                    }
                }
            } catch (Exception e) {
                e.printStackTrace();
            } finally {
                if (logProcess != null)
                    logProcess.destroy();
            }
        }

        public void stopReading() {
            running = false;
            if (logProcess != null)
                logProcess.destroy();
            interrupt();
        }
    }

    // 预编译正则模式，避免重复编译
    // 匹配 logcat -v time 的时间戳头部: "12-16 14:14:36.123 "
    // 格式: MM-DD HH:MM:SS.mmm (可能后面有空格)
    private static final java.util.regex.Pattern LOG_HEAD_PATTERN = java.util.regex.Pattern
            .compile("^\\d{2}-\\d{2}\\s+\\d{2}:\\d{2}:\\d{2}\\.\\d{3}\\s+");

    // 更新日志视图
    private void updateLogView(String line) {
        if (logAdapter != null) {
            logAdapter.addLog(line);
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

    /**
     * 初始化3DOF传感器和渲染器
     */
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

    /**
     * 在视角前方5米处生成3DOF立方体
     */
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

    /**
     * 获取设备的IP地址
     * 
     * @return 设备IP地址，如果获取失败则返回本地回环地址
     */
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
            useWebCamera = true; // 切换到Web模式：本地相机继续运行但不处理数据

            // 启动Web图像处理线程
            startWebFrameProcessing();

            btnStartWeb.setText(getString(R.string.btn_web_server_close));

            String ipAddress = getDeviceIpAddress();
            showHint("https://" + ipAddress + ":8080");
            Log.d(TAG, "Web服务器已启动，本地处理已停止，仅处理浏览器图像");
        } else {
            // 停止Web图像处理线程
            stopWebFrameProcessing();

            if (webServer != null) {
                webServer.stop();
            }
            isWebRunning = false;
            useWebCamera = false; // 切换回本地相机模式

            // 清空浏览器图像数据
            synchronized (browserFrameLock) {
                browserFrameData = null;
            }

            btnStartWeb.setText(getString(R.string.btn_web_server_open));
            showHint(getString(R.string.hint_web_server_closed));
            Log.d(TAG, "Web服务器已关闭，本地相机已恢复正常处理");
        }
    }

    private void toggleOpticalFlow() {
        if (nativeHelper != null) {
            isOpticalFlowEnabled = !isOpticalFlowEnabled;
            nativeHelper.setOpticalFlowEnabled(isOpticalFlowEnabled);
            updateOpticalFlowButton();
            showHint(getString(isOpticalFlowEnabled ? R.string.hint_optical_flow_enabled : R.string.hint_optical_flow_disabled));
        }
    }

    private void updateOpticalFlowButton() {
        if (btnToggleOpticalFlow == null) return;
        
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (isOpticalFlowEnabled) {
                    btnToggleOpticalFlow.setText(getString(R.string.btn_optical_flow_on));
                    btnToggleOpticalFlow.setBackgroundTintList(android.content.res.ColorStateList.valueOf(0xFF4CAF50)); // Green
                } else {
                    btnToggleOpticalFlow.setText(getString(R.string.btn_optical_flow_off));
                    btnToggleOpticalFlow.setBackgroundTintList(android.content.res.ColorStateList.valueOf(0xFF757575)); // Grey
                }
            }
        });
    }

    private void toggleLoopClosing() {
        if (nativeHelper != null) {
            isLoopClosingEnabled = !isLoopClosingEnabled;
            nativeHelper.setLoopClosingEnabled(isLoopClosingEnabled);
            updateLoopClosingButton();
            showHint(getString(isLoopClosingEnabled ? R.string.hint_loop_closing_enabled : R.string.hint_loop_closing_disabled));
        }
    }

    private void updateLoopClosingButton() {
        if (btnToggleLoopClosing == null) return;
        
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (isLoopClosingEnabled) {
                    btnToggleLoopClosing.setText(getString(R.string.btn_loop_closing_on));
                    btnToggleLoopClosing.setBackgroundTintList(android.content.res.ColorStateList.valueOf(0xFF4CAF50)); // Green
                } else {
                    btnToggleLoopClosing.setText(getString(R.string.btn_loop_closing_off));
                    btnToggleLoopClosing.setBackgroundTintList(android.content.res.ColorStateList.valueOf(0xFF757575)); // Grey
                }
            }
        });
    }
}
