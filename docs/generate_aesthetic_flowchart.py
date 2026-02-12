
import graphviz
import os

def generate_aesthetic_flowchart():
    # Create a directed graph with high resolution and "dot" layout
    dot = graphviz.Digraph(
        'ORB_SLAM2s_Architecture', 
        comment='High-Fidelity Visual Causal Flow of Android_ORB-SLAM2s',
        format='png'
    )
    
    # Global Graph Attributes
    dot.attr(rankdir='TB') # Top to Bottom flow
    dot.attr(compound='true') # Allow edges between clusters
    dot.attr(dpi='300') # High Quality for paper
    dot.attr(splines='ortho') # Orthogonal edges for cleaner, more compact routing
    dot.attr(nodesep='0.2', ranksep='0.3') # Significantly reduced spacing
    dot.attr(newrank='true') # Proper rank alignment across clusters
    dot.attr(concentrate='true') # Merge edges to reduce clutter
    dot.attr(fontname='Helvetica', fontsize='12')
    
    # Default Node Styles - Reduce default margins
    dot.attr('node', shape='record', style='filled', fontname='Helvetica', fontsize='10', penwidth='0.5', margin='0.05,0.05')
    dot.attr('edge', fontname='Helvetica', fontsize='9', color='#444444', penwidth='0.8', arrowsize='0.7')

    # Color Palette (Pastel/Professional)
    COLOR_ANDROID = '#E1F5FE' # Light Blue
    COLOR_JNI = '#FFF3E0'     # Light Orange
    COLOR_SLAM_TRACK = '#E8F5E9' # Light Green
    COLOR_SLAM_MAP = '#F1F8E9'   # Lighter Green
    COLOR_SLAM_LOOP = '#F9FBE7'  # Lightest Green
    COLOR_DATA = '#F3E5F5'       # Light Purple
    COLOR_IO = '#ECEFF1'         # Light Grey

    # ==================== Android Java Layer ====================
    with dot.subgraph(name='cluster_0_Android') as android:
        android.attr(label='Android Application Layer (Java)', color='#0277BD', style='rounded', bgcolor=COLOR_ANDROID)
        
        # Nodes using HTML-like labels for detail
        android.node('UI_Activity', '''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
            <tr><td bgcolor="#0288D1"><font color="white"><b>ArCamUIActivity</b></font></td></tr>
            <tr><td align="left">• onResume(): Init System<br/>• onCameraFrame(): Feed Video<br/>• UI Handling</td></tr>
        </table>>''', shape='plain')
        
        android.node('Sensor_IMU', '''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
            <tr><td bgcolor="#0288D1"><font color="white"><b>OrientationSensor</b></font></td></tr>
            <tr><td align="left">• Accel + Gyro + Mag<br/>• Rotation Matrix</td></tr>
        </table>>''', shape='plain')

        android.node('User_Input', '''<<table border="0" cellborder="0" cellspacing="0" cellpadding="4">
            <tr><td><b>User Interactions</b></td></tr>
            <tr><td align="left">1. Toggle SLAM / 3DOF<br/>2. Detect Plane<br/>3. Save/Load Map</td></tr>
        </table>>''', shape='plain', fillcolor='#B3E5FC')

        # Edges internal to Android
        android.edge('User_Input', 'UI_Activity', 'Trigger Event')
        android.edge('Sensor_IMU', 'UI_Activity', 'Sensor Data')

    # ==================== JNI Bridge Layer ====================
    with dot.subgraph(name='cluster_1_JNI') as jni:
        jni.attr(label='JNI Bridge (C++)', color='#EF6C00', style='rounded', bgcolor=COLOR_JNI)
        
        jni.node('JNI_Process', '''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
            <tr><td bgcolor="#F57C00"><font color="white"><b>processImage()</b></font></td></tr>
            <tr><td align="left">• Preprocess Image<br/>• Check SLAM Status<br/>• Manage Thread Locks</td></tr>
        </table>>''', shape='plain')

        jni.node('JNI_Detect', '''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
            <tr><td bgcolor="#F57C00"><font color="white"><b>detectPlane()</b></font></td></tr>
            <tr><td align="left">• RANSAC Plane Fitting<br/>• Update Global Plane</td></tr>
        </table>>''', shape='plain')

        jni.node('JNI_3DOF', '''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
            <tr><td bgcolor="#FF7043"><font color="white"><b>3DOF Module</b></font></td></tr>
            <tr><td align="left">• compute3DofMVP()<br/>• calculateInsertionPoint()</td></tr>
        </table>>''', shape='plain')

        jni.node('Global_State', '''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
            <tr><td bgcolor="#FFCC80"><b>Global State</b></td></tr>
            <tr><td align="left">• gArObjects<br/>• pPlane<br/>• Tcw (Current Pose)</td></tr>
        </table>>''', shape='plain')

        # Logic Flow
        jni.edge('JNI_Process', 'JNI_Detect', 'if detectPlane==true', style='dashed')
        jni.edge('JNI_Process', 'Global_State', 'Update Pose/MapPoints')
        jni.edge('JNI_Detect', 'Global_State', 'Update pPlane', penwidth='2.0')

    # ==================== ORB-SLAM2 System Layer ====================
    with dot.subgraph(name='cluster_2_ORBSLAM') as system:
        system.attr(label='ORB-SLAM2 System Core', color='#2E7D32', style='rounded', bgcolor='#FAFAFA')

        # Tracking Thread
        with dot.subgraph(name='cluster_2a_Tracking') as track:
            track.attr(label='Thread: Tracking', color='#43A047', bgcolor=COLOR_SLAM_TRACK)
            
            track.node('System_Track', '''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
                <tr><td bgcolor="#43A047"><font color="white"><b>TrackMonocular()</b></font></td></tr>
                <tr><td align="left">Orchestrates Tracking</td></tr>
            </table>>''', shape='plain')
            
            track.node('ORB_Extract', 'ORBExtractor::operator()', shape='box')
            track.node('Pose_Opt', 'Optimizer::PoseOptimization\n(G2O / BA)', shape='box')
            track.node('Track_Ref', 'TrackReferenceKeyFrame', shape='box')
            track.node('Track_Model', 'TrackWithMotionModel', shape='box')
            track.node('Reloc', 'Relocalization\n(EPnP)', shape='hexagon', style='filled', fillcolor='#DCEDC8')

            # Edges
            track.edge('System_Track', 'ORB_Extract', 'Extract Features')
            track.edge('ORB_Extract', 'Track_Model', 'Try Motion Model')
            track.edge('Track_Model', 'Track_Ref', 'If Lost')
            track.edge('Track_Ref', 'Reloc', 'If Lost')
            track.edge('Reloc', 'Pose_Opt', 'Refine Pose')
            track.edge('Track_Model', 'Pose_Opt', 'Refine Pose')

        # Local Mapping Thread
        with dot.subgraph(name='cluster_2b_Mapping') as mapping:
            mapping.attr(label='Thread: Local Mapping', color='#43A047', bgcolor=COLOR_SLAM_MAP)
            mapping.node('LocalMap_Loop', '''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
                <tr><td bgcolor="#66BB6A"><font color="white"><b>LocalMapping::Run()</b></font></td></tr>
                <tr><td align="left">1. ProcessNewKeyFrame<br/>2. MapPointCulling<br/>3. CreateNewMapPoints<br/>4. LocalBundleAdjustment</td></tr>
            </table>>''', shape='plain')

        # Loop Closing Thread
        with dot.subgraph(name='cluster_2c_Loop') as loop:
            loop.attr(label='Thread: Loop Closing', color='#43A047', bgcolor=COLOR_SLAM_LOOP)
            loop.node('Loop_Run', '''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
                <tr><td bgcolor="#9CCC65"><font color="white"><b>LoopClosing::Run()</b></font></td></tr>
                <tr><td align="left">1. DetectLoop (BoW)<br/>2. ComputeSim3<br/>3. CorrectLoop (PoseGraph)</td></tr>
            </table>>''', shape='plain')

        # Data Assets
        system.node('Map_Atlas', '''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
            <tr><td bgcolor="#AB47BC"><font color="white"><b>Map / Atlas</b></font></td></tr>
            <tr><td align="left">• KeyFrames (Poses)<br/>• MapPoints (3D)<br/>• Spanning Tree</td></tr>
        </table>>''', shape='plain', fillcolor=COLOR_DATA)

        # Thread Interactions
        system.edge('Pose_Opt', 'Map_Atlas', 'Update Current Pose')
        system.edge('System_Track', 'LocalMap_Loop', label='Insert New KF', style='dashed')
        system.edge('LocalMap_Loop', 'Map_Atlas', 'Add MPs/KFs')
        system.edge('LocalMap_Loop', 'Loop_Run', 'Check Loop', style='dotted')
        system.edge('Loop_Run', 'Map_Atlas', 'Optimize Graph')

        # Persistence Module (Integrated)
        system.node('Persistence', '''<<table border="0" cellborder="1" cellspacing="0" cellpadding="4">
            <tr><td bgcolor="#78909C"><font color="white"><b>Persistence (IO)</b></font></td></tr>
            <tr><td align="left">• SaveMap(binary)<br/>• LoadMap(binary)<br/>• Save/Load AR Info</td></tr>
        </table>>''', shape='plain', fillcolor=COLOR_IO)

        system.edge('Persistence', 'Map_Atlas', 'Serialize/Deserialize', dir='both')

    # ==================== Cross-Layer Connections ====================
    dot.edge('UI_Activity', 'JNI_Process', '1. Mat (Frame)', penwidth='2.0', color='#01579B')
    dot.edge('JNI_Process', 'System_Track', '2. TrackMonocular()', penwidth='2.0', color='#E65100')
    dot.edge('JNI_3DOF', 'Global_State', 'Writes MVP')
    dot.edge('UI_Activity', 'JNI_3DOF', '3. Get MVP', style='dashed')
    dot.edge('User_Input', 'Persistence', 'Trigger Save/Load', color='#BF360C')

    # Save
    output_path = 'aesthetic_visual_causal_flow'
    try:
        dot.save(output_path + '.dot')
        print(f"DOT source saved to {os.path.abspath(output_path + '.dot')}")
    except Exception as e:
        print(f"Error saving DOT: {e}")

    try:
        dot.render(output_path, cleanup=False)
        print(f"Flowchart generated at {os.path.abspath(output_path + '.png')}")
    except Exception as e:
        print(f"Error rendering flowchart: {e}")

if __name__ == "__main__":
    generate_aesthetic_flowchart()
