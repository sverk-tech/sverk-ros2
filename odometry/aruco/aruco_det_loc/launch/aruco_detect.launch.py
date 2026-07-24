from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _make_node(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    is_debug = (mode == 'debug')

    params = {
        # Inputs
        'map_markers_topic': LaunchConfiguration('map_markers_topic'),
        'image_topic': LaunchConfiguration('image_topic'),
        'camera_info_topic': LaunchConfiguration('camera_info_topic'),

        # parent frame for published markers / visualization
        'parent_frame_id': LaunchConfiguration('parent_frame_id'),

        # ArUco
        'marker_size': ParameterValue(LaunchConfiguration('marker_size'), value_type=float),
        'dictionary_id': ParameterValue(LaunchConfiguration('dictionary_id'), value_type=int),

        # ArUco3 fast detection (2-5x faster, may miss markers smaller than ratio*image_side)
        'use_aruco3_detection': ParameterValue(
            LaunchConfiguration('use_aruco3_detection'), value_type=bool
        ),
        'aruco3_min_marker_length_ratio': ParameterValue(
            LaunchConfiguration('aruco3_min_marker_length_ratio'), value_type=float
        ),
        'max_detect_fps': ParameterValue(
            LaunchConfiguration('max_detect_fps'), value_type=float
        ),

        # OpenCV threads (int — works via LaunchConfiguration)
        'opencv_num_threads': ParameterValue(
            LaunchConfiguration('opencv_num_threads'), value_type=int
        ),

        # NOTE: marker_id_whitelist and cpu_affinity_ids are integer-array parameters.
        # They cannot be reliably set via LaunchConfiguration (string → array type mismatch).
        # Use CLI instead:
        #   --ros-args -p marker_id_whitelist:=[0,1,2] -p cpu_affinity_ids:=[0,1]
        # Or pass a YAML file:  --params-file my_config.yaml

        # Single-solvePnP behavior toggle
        'estimate_marker_pose': ParameterValue(
            LaunchConfiguration('estimate_marker_pose'), value_type=bool
        ),

        # solvePnP FPS cap
        'pnp_fps_cap_enabled': ParameterValue(
            LaunchConfiguration('pnp_fps_cap_enabled'), value_type=bool
        ),
        'pnp_fps_cap': ParameterValue(
            LaunchConfiguration('pnp_fps_cap'), value_type=int
        ),

        # Marker debug image + RViz visualization (mode-controlled)
        'debug_image_topic': LaunchConfiguration('debug_image_topic'),
        'publish_debug_image': is_debug,
        'publish_visualization': is_debug,

        # world-axes debug image driven by /aruco/loc/pose_cov (mode-controlled)
        'publish_world_debug_image': is_debug,
        'world_debug_image_topic': LaunchConfiguration('world_debug_image_topic'),
        'pose_cov_topic': LaunchConfiguration('pose_cov_topic'),
        'world_axes_length': ParameterValue(LaunchConfiguration('world_axes_length'), value_type=float),
        'world_axes_thickness': ParameterValue(LaunchConfiguration('world_axes_thickness'), value_type=int),
        'pose_cov_is_world_to_camera': ParameterValue(
            LaunchConfiguration('pose_cov_is_world_to_camera'), value_type=bool
        ),
        'world_pose_max_age_sec': ParameterValue(
            LaunchConfiguration('world_pose_max_age_sec'), value_type=float
        ),
    }

    return [Node(
        package='aruco_det_loc',
        executable='aruco_detect_node',
        name='aruco_detect',
        output='screen',
        parameters=[params],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='normal',
            description="Run mode: 'normal' or 'debug' (debug enables marker debug image + RViz viz + world-axes debug image)",
        ),

        DeclareLaunchArgument('map_markers_topic', default_value='/aruco/map/markers'),

        # Defaults point at the real on-board camera. Override for Gazebo, e.g.
        #   image_topic:=/world/.../imager/image camera_info_topic:=/world/.../imager/camera_info
        DeclareLaunchArgument('image_topic',       default_value='/camera_1/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera_1/camera_info'),

        DeclareLaunchArgument(
            'parent_frame_id',
            default_value='camera_optical_1',
            description="Optical frame of the camera; MUST match the external static "
                        "TF base_link->camera_optical_1. Parent of published markers/viz.",
        ),

        DeclareLaunchArgument('marker_size', default_value='0.5'),
        DeclareLaunchArgument('dictionary_id', default_value='3'),

        # ArUco3 fast detection
        DeclareLaunchArgument(
            'use_aruco3_detection',
            default_value='false',
            description=(
                "Enable ArUco3 multi-scale detection (2-5x faster). "
                "WARNING: markers smaller than aruco3_min_marker_length_ratio * image_side "
                "will not be detected. Keep false if you need small-marker coverage."
            ),
        ),
        DeclareLaunchArgument(
            'aruco3_min_marker_length_ratio',
            default_value='0.10',
            description="Minimum marker side as fraction of image side for ArUco3 (0.0-1.0).",
        ),
        DeclareLaunchArgument(
            'max_detect_fps',
            default_value='0.0',
            description="Max detect/image processing rate in Hz. <=0 disables frame throttling.",
        ),

        # Marker ID whitelist
        DeclareLaunchArgument(
            'marker_id_whitelist',
            default_value='[]',
            description=(
                "JSON-array of marker IDs to accept, e.g. '[0, 1, 2]'. "
                "Empty list (default) means accept all detected markers. "
                "Tip: also set dictionary_id=0 (DICT_4X4_50) when all your "
                "marker IDs are < 50 — smaller dictionary speeds up detection by ~5-10%%."
            ),
        ),

        # CPU affinity
        DeclareLaunchArgument(
            'cpu_affinity_ids',
            default_value='[]',
            description=(
                "Core IDs to pin the detect node to. "
                "Run scripts/check_cpu_topology.sh to identify Cortex-A76 core IDs. "
                "Example for Allwinner A733: '[0, 1]'"
            ),
        ),
        DeclareLaunchArgument(
            'opencv_num_threads',
            default_value='2',
            description=(
                "OpenCV internal thread pool size. "
                "2 = match A76 big-core count (recommended). "
                "0 = fully serial. -1 = OpenCV auto-detect (may pick LITTLE cores)."
            ),
        ),

        # Pose estimation
        DeclareLaunchArgument(
            'estimate_marker_pose',
            default_value='false',
            description="If true, compute per-marker pose (solvePnP) and publish it (and TF if send_tf is enabled in node).",
        ),
        DeclareLaunchArgument(
            'pnp_fps_cap_enabled',
            default_value='true',
            description="If true, limit how often solvePnP runs (global cap).",
        ),
        DeclareLaunchArgument(
            'pnp_fps_cap',
            default_value='10',
            description="Max solvePnP rate in Hz (must be > 0 when pnp_fps_cap_enabled is true).",
        ),

        # Marker debug image
        DeclareLaunchArgument('debug_image_topic', default_value='/aruco/vis/debug_image'),

        # world-axes debug image
        DeclareLaunchArgument('world_debug_image_topic', default_value='/aruco/vis/world_debug_image'),
        DeclareLaunchArgument('pose_cov_topic', default_value='/aruco/loc/pose_cov'),
        DeclareLaunchArgument('world_axes_length', default_value='0.5'),
        DeclareLaunchArgument('world_axes_thickness', default_value='3'),
        DeclareLaunchArgument(
            'pose_cov_is_world_to_camera',
            default_value='false',
            description="false: pose_cov pose is camera pose in world (T_world_camera); true: pose is world->camera (T_camera_world)",
        ),
        DeclareLaunchArgument(
            'world_pose_max_age_sec',
            default_value='0.2',
            description="Max |image_stamp - pose_stamp| to draw world axes; <=0 disables gating",
        ),

        OpaqueFunction(function=_make_node),
    ])
