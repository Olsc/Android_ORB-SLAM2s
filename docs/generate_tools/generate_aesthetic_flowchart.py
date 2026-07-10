
import graphviz
import os
import sys
_env_bin = os.path.join(os.path.dirname(sys.executable), 'Library', 'bin')
if os.path.isdir(_env_bin):
    os.environ['PATH'] = _env_bin + os.pathsep + os.environ.get('PATH', '')

def generate_flowchart(lang='en'):
    """
    生成指定语言（'en' 或 'zh'）的流程图。
    """
    
    # 本地化字符串辅助函数
    def t(en_text, zh_text):
        return zh_text if lang == 'zh' else en_text

    filename = f'aesthetic_visual_causal_flow_{lang}'
    
    # 创建有向图
    dot = graphviz.Digraph(
        'ORB_SLAM2s_Architecture', 
        comment=t('High-Fidelity Visual Causal Flow of Android_ORB-SLAM2s', 'Android_ORB-SLAM2s 高保真视觉因果流程图'),
        format='svg'
    )
    
    # 全局图表属性
    dot.attr(rankdir='TB') # 从上到下的流程
    dot.attr(compound='true') # 允许子图之间的连线
    # dot.attr(dpi='300') # 移除 DPI 设置以解决 SVG 尺寸/分辨率在不同设备上的显示问题
    dot.attr(splines='true') # 改为样条曲线 (Spline) 以支持连线标签 (Edge Labels) 并消除警告
    dot.attr(nodesep='0.2', ranksep='0.3') # 减少间距
    dot.attr(newrank='true') # 等级对齐
    dot.attr(concentrate='true') # 合并连线
    
    # 字体设置
    # 中文和英文使用支持度最广的字体列表，确保在 Windows (微软雅黑) 和 Linux (文泉驿微米黑) 均能完美渲染
    font_name = 'Microsoft YaHei, SimHei, WenQuanYi Micro Hei, Noto Sans CJK SC, sans-serif' if lang == 'zh' else 'Helvetica, Arial, sans-serif'
    dot.attr(fontname=font_name, fontsize='12')
    
    # 默认节点样式
    dot.attr('node', shape='record', style='filled', fontname=font_name, fontsize='10', penwidth='0.5', margin='0.05,0.05')
    dot.attr('edge', fontname=font_name, fontsize='9', color='#444444', penwidth='0.8', arrowsize='0.7')

    # 调色板
    COLOR_ANDROID = '#E1F5FE' # 浅蓝
    COLOR_JNI = '#FFF3E0'     # 浅橙
    COLOR_SLAM_TRACK = '#E8F5E9' # 浅绿
    COLOR_SLAM_MAP = '#F1F8E9'   # 更浅的绿
    COLOR_SLAM_LOOP = '#F9FBE7'  # 最浅的绿
    COLOR_DATA = '#F3E5F5'       # 浅紫
    COLOR_IO = '#ECEFF1'         # 浅灰

    # ==================== Android Java 层 ====================
    with dot.subgraph(name='cluster_0_Android') as android:
        android.attr(label=t('Android Application Layer (Java)', 'Android 应用层 (Java)'), 
                     color='#0277BD', style='rounded', bgcolor=COLOR_ANDROID)
        
        # 节点: UI Activity
        ui_label_title = "ArCamUIActivity"
        ui_label_body = t(
            "• onResume(): Init System<br/>• onCameraFrame(): Feed Video<br/>• UI Handling",
            "• onResume(): 系统初始化<br/>• onCameraFrame(): 传入视频流<br/>• UI Handling: 界面交互"
        )
        android.node('UI_Activity', f'''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
            <tr><td bgcolor="#0288D1"><font color="white"><b>{ui_label_title}</b></font></td></tr>
            <tr><td align="left">{ui_label_body}</td></tr>
        </table>>''', shape='plain')
        
        # 节点: CameraX 相机管道
        cam_title = t("Camera Pipeline", "相机管道")
        cam_body = t(
            "• CameraX ImageAnalysis<br/>• YUV → RGBA + Gray Mat<br/>• OpenCVBridge JNI",
            "• CameraX ImageAnalysis<br/>• YUV → RGBA + Gray 转换<br/>• OpenCVBridge JNI 调用"
        )
        android.node('CameraX_Pipe', f'''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
            <tr><td bgcolor="#0277BD"><font color="white"><b>{cam_title}</b></font></td></tr>
            <tr><td align="left">{cam_body}</td></tr>
        </table>>''', shape='plain')

        # 节点: Web 服务器
        web_title = t("Web Server (Optional)", "Web 服务器 (可选)")
        web_body = t(
            "• SSL/TLS (HTTPS:8080)<br/>• MJPEG Stream + Data API<br/>• Remote Frame Upload",
            "• SSL/TLS 加密 (HTTPS:8080)<br/>• MJPEG 流 + 数据 API<br/>• 远程图像帧上传"
        )
        android.node('Web_Server', f'''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
            <tr><td bgcolor="#0277BD"><font color="white"><b>{web_title}</b></font></td></tr>
            <tr><td align="left">{web_body}</td></tr>
        </table>>''', shape='plain')

        # 节点: 方向传感器
        sensor_title = t("OrientationSensor", "方向传感器 (OrientationSensor)")
        sensor_body = t(
            "• Accel + Gyro + Mag<br/>• Rotation Matrix",
            "• 加速计 + 陀螺仪 + 磁力计<br/>• 旋转矩阵 (Rotation Matrix)"
        )
        android.node('Sensor_IMU', f'''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
            <tr><td bgcolor="#0288D1"><font color="white"><b>{sensor_title}</b></font></td></tr>
            <tr><td align="left">{sensor_body}</td></tr>
        </table>>''', shape='plain')

        # 节点: 用户交互
        input_title = t("User Interactions", "用户交互")
        input_body = t(
            "1. Toggle SLAM / 3DOF<br/>2. Detect Plane<br/>3. Save/Load Map",
            "1. 切换 SLAM / 3DOF<br/>2. 检测平面 (Detect Plane)<br/>3. 保存/加载地图"
        )
        android.node('User_Input', f'''<<table border="0" cellborder="0" cellspacing="0" cellpadding="4">
            <tr><td><b>{input_title}</b></td></tr>
            <tr><td align="left">{input_body}</td></tr>
        </table>>''', shape='plain', fillcolor='#B3E5FC')

        # Android 内部连线
        android.edge('User_Input', 'UI_Activity', t('Trigger Event', '触发事件'))
        android.edge('CameraX_Pipe', 'UI_Activity', t('RGBA+Gray Frame', 'RGBA+Gray 帧'))
        android.edge('Sensor_IMU', 'UI_Activity', t('Sensor Data', '传感器数据'))
        android.edge('UI_Activity', 'Web_Server', t('Manual Start/Stop', '手动启停'), style='dashed')

    # ==================== JNI 桥接层 ====================
    with dot.subgraph(name='cluster_1_JNI') as jni:
        jni.attr(label=t('JNI Bridge (C++)', 'JNI 桥接层 (C++)'), 
                 color='#EF6C00', style='rounded', bgcolor=COLOR_JNI)
        
        # 节点: JNI 处理程序
        jni_process_body = t(
            "• Preprocess Image<br/>• Check SLAM Status<br/>• Manage Thread Locks",
            "• 图像预处理<br/>• 检查 SLAM 状态<br/>• 线程锁管理"
        )
        jni.node('JNI_Process', f'''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
            <tr><td bgcolor="#F57C00"><font color="white"><b>processImage()</b></font></td></tr>
            <tr><td align="left">{jni_process_body}</td></tr>
        </table>>''', shape='plain')

        # 节点: JNI 检测平面
        jni_detect_body = t(
            "• RANSAC Plane Fitting<br/>• Update Global Plane",
            "• RANSAC 平面拟合<br/>• 更新全局平面"
        )
        jni.node('JNI_Detect', f'''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
            <tr><td bgcolor="#F57C00"><font color="white"><b>detectPlane()</b></font></td></tr>
            <tr><td align="left">{jni_detect_body}</td></tr>
        </table>>''', shape='plain')

        # 节点: 3DOF 模块
        dof_title = t("3DOF Module", "3DOF 模块")
        jni.node('JNI_3DOF', f'''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
            <tr><td bgcolor="#FF7043"><font color="white"><b>{dof_title}</b></font></td></tr>
            <tr><td align="left">• compute3DofMVP()<br/>• calculate3DofInsertionPoint()</td></tr>
        </table>>''', shape='plain')

        # 节点: 全局状态
        state_title = t("Global State", "全局状态")
        state_body = t(
            "• gArObjects<br/>• pPlane<br/>• Tcw (Current Pose)",
            "• gArObjects<br/>• pPlane<br/>• Tcw (当前位姿)"
        )
        jni.node('Global_State', f'''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
            <tr><td bgcolor="#FFCC80"><b>{state_title}</b></td></tr>
            <tr><td align="left">{state_body}</td></tr>
        </table>>''', shape='plain')

        # 逻辑流
        jni.edge('JNI_Process', 'JNI_Detect', 'if detectPlane==true', style='dashed')
        jni.edge('JNI_Process', 'Global_State', t('Update Pose/MapPoints', '更新位姿/地图点'))
        jni.edge('JNI_Detect', 'Global_State', t('Update pPlane', '更新平面'), penwidth='2.0')

    # ==================== ORB-SLAM2 系统层 ====================
    with dot.subgraph(name='cluster_2_ORBSLAM') as system:
        system.attr(label=t('ORB-SLAM2 System Core', 'ORB-SLAM2 系统核心'), 
                    color='#2E7D32', style='rounded', bgcolor='#FAFAFA')

        # 跟踪线程 (Tracking Thread)
        with dot.subgraph(name='cluster_2a_Tracking') as track:
            track.attr(label=t('Thread: Tracking', '线程: 跟踪 (Tracking)'), 
                       color='#43A047', bgcolor=COLOR_SLAM_TRACK)
            
            track.node('System_Track', f'''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
                <tr><td bgcolor="#43A047"><font color="white"><b>TrackMonocular()</b></font></td></tr>
                <tr><td align="left">{t("Orchestrates Tracking", "协调跟踪过程")}</td></tr>
            </table>>''', shape='plain')
            
            track.node('ORB_Extract', 'ORBextractor::operator()', shape='box')
            track.node('Pose_Opt', 'Optimizer::PoseOptimization\n(G2O / BA)', shape='box')
            track.node('Track_Ref', 'TrackReferenceKeyFrame', shape='box')
            track.node('Track_Model', 'TrackWithMotionModel', shape='box')
            track.node('Reloc', t("Relocalization\n(EPnP)", "重定位\n(EPnP)"), shape='hexagon', style='filled', fillcolor='#DCEDC8')

            # 连线
            track.edge('System_Track', 'ORB_Extract', t('Extract Features', '提取特征'))
            track.edge('ORB_Extract', 'Track_Model', t('Try Motion Model', '尝试运动模型'))
            track.edge('Track_Model', 'Track_Ref', t('If Lost', '若丢失'))
            track.edge('Track_Ref', 'Reloc', t('If Lost', '若丢失'))
            track.edge('Reloc', 'Pose_Opt', t('Refine Pose', '优化位姿'))
            track.edge('Track_Model', 'Pose_Opt', t('Refine Pose', '优化位姿'))

        # 局部建图线程 (Local Mapping Thread)
        with dot.subgraph(name='cluster_2b_Mapping') as mapping:
            mapping.attr(label=t('Thread: Local Mapping', '线程: 局部建图 (Local Mapping)'), 
                         color='#43A047', bgcolor=COLOR_SLAM_MAP)
            
            lm_body = t(
                "1. ProcessNewKeyFrame<br/>2. MapPointCulling<br/>3. CreateNewMapPoints<br/>4. LocalBundleAdjustment",
                "1. ProcessNewKeyFrame (处理关键帧)<br/>2. MapPointCulling (剔除地图点)<br/>3. CreateNewMapPoints (创建地图点)<br/>4. LocalBundleAdjustment (局部BA)"
            )
            mapping.node('LocalMap_Loop', f'''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
                <tr><td bgcolor="#66BB6A"><font color="white"><b>LocalMapping::Run()</b></font></td></tr>
                <tr><td align="left">{lm_body}</td></tr>
            </table>>''', shape='plain')

        # 回环检测线程 (Loop Closing Thread)
        with dot.subgraph(name='cluster_2c_Loop') as loop:
            loop.attr(label=t('Thread: Loop Closing', '线程: 回环检测 (Loop Closing)'), 
                         color='#43A047', bgcolor=COLOR_SLAM_LOOP)
            
            lc_body = t(
                "1. DetectLoop (BoW)<br/>2. ComputeSim3<br/>3. CorrectLoop (PoseGraph)",
                "1. DetectLoop (BoW检测)<br/>2. ComputeSim3 (计算Sim3)<br/>3. CorrectLoop (图优化矫正)"
            )
            loop.node('Loop_Run', f'''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
                <tr><td bgcolor="#9CCC65"><font color="white"><b>LoopClosing::Run()</b></font></td></tr>
                <tr><td align="left">{lc_body}</td></tr>
            </table>>''', shape='plain')

        # 数据资产 (Data Assets)
        map_title = t("Map / Atlas", "Map / Atlas (地图/图册)")
        map_body = t(
            "• KeyFrames (Poses)<br/>• MapPoints (3D)<br/>• Spanning Tree",
            "• KeyFrames (关键帧位姿)<br/>• MapPoints (3D地图点)<br/>• Spanning Tree (生成树)"
        )
        system.node('Map_Atlas', f'''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
            <tr><td bgcolor="#AB47BC"><font color="white"><b>{map_title}</b></font></td></tr>
            <tr><td align="left">{map_body}</td></tr>
        </table>>''', shape='plain', fillcolor=COLOR_DATA)

        # 线程间交互
        system.edge('Pose_Opt', 'Map_Atlas', t('Update Current Pose', '更新当前位姿'))
        system.edge('System_Track', 'LocalMap_Loop', label=t('Insert New KF', '插入新关键帧'), style='dashed')
        system.edge('LocalMap_Loop', 'Map_Atlas', t('Add MPs/KFs', '添加点/帧'))
        system.edge('LocalMap_Loop', 'Loop_Run', t('Check Loop', '检查回环'), style='dotted')
        system.edge('Loop_Run', 'Map_Atlas', t('Optimize Graph', '优化全局图'))

        # 持久化模块 (Persistence Module)
        persist_title = t("Persistence (IO)", "持久化 (IO)")
        persist_body = t(
            "• SaveMap(binary)<br/>• LoadMap(binary)<br/>• Save/Load AR Info",
            "• SaveMap(binary) (保存地图)<br/>• LoadMap(binary) (加载地图)<br/>• Save/Load AR Info (保存/加载AR信息)"
        )
        system.node('Persistence', f'''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
            <tr><td bgcolor="#78909C"><font color="white"><b>{persist_title}</b></font></td></tr>
            <tr><td align="left">{persist_body}</td></tr>
        </table>>''', shape='plain', fillcolor=COLOR_IO)

        system.edge('Persistence', 'Map_Atlas', t('Serialize/Deserialize', '序列化/反序列化'), dir='both')

    # ==================== 跨层级连接 ====================
    dot.edge('UI_Activity', 'JNI_Process', t('1. processCameraFrame()', '1. processCameraFrame()'), penwidth='2.0', color='#01579B')
    dot.edge('JNI_Process', 'System_Track', '2. TrackMonocular()', penwidth='2.0', color='#E65100')
    dot.edge('JNI_3DOF', 'Global_State', t('Writes MVP', '写入 MVP'))
    dot.edge('UI_Activity', 'JNI_3DOF', t('3. Get MVP', '3. 获取 MVP'), style='dashed')
    dot.edge('User_Input', 'Persistence', t('Trigger Save/Load', '触发保存/加载'), color='#BF360C')
    dot.edge('Web_Server', 'JNI_Process', t('Query SLAM Data', '查询 SLAM 数据'), style='dotted', color='#0277BD')

    # 保存文件
    try:
        dot.save(filename + '.dot')
        print(f"DOT 源码已保存至 {os.path.abspath(filename + '.dot')}")
    except Exception as e:
        print(f"保存 {lang} 版 DOT 时发生错误: {e}")

    try:
        dot.render(filename, cleanup=False)
        print(f"流程图 ({lang}) 已生成： {os.path.abspath(filename + '.svg')}")
    except Exception as e:
        print(f"渲染 {lang} 版流程图时发生错误: {e}")

if __name__ == "__main__":
    # 生成英文版本
    print("正在生成英文流程图...")
    generate_flowchart('en')
    
    print("-" * 30)
    
    # 生成中文版本
    print("正在生成中文流程图...")
    generate_flowchart('zh')
