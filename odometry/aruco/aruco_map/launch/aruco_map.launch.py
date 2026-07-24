from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARUCO_MAP_SRC_DIR = "/home/sverk/sverk_ws/src/sverk_drone/odometry/aruco/aruco_map"

def _make_node(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    is_debug = (mode == 'debug')

    params = {
        # Base map publishing
        'frame_id': LaunchConfiguration('frame_id'),
        'marker_size': LaunchConfiguration('marker_size'),
        'topic_name': LaunchConfiguration('topic_name'),
        'map_file': LaunchConfiguration('map_file'),

        # Debug images
        'publish_debug_images': is_debug,

        # Debug image tuning (still overridable)
        'debug_image_topic_prefix': LaunchConfiguration('debug_image_topic_prefix'),
        'debug_image_publish_rate_hz': LaunchConfiguration('debug_image_publish_rate_hz'),
        'plane_angle_threshold_deg': LaunchConfiguration('plane_angle_threshold_deg'),
        'plane_distance_threshold_m': LaunchConfiguration('plane_distance_threshold_m'),
        'debug_pixels_per_meter': LaunchConfiguration('debug_pixels_per_meter'),
        'debug_image_margin_px': LaunchConfiguration('debug_image_margin_px'),
        'debug_image_max_size_px': LaunchConfiguration('debug_image_max_size_px'),

        # Debug marker rendering options
        'debug_aruco_dictionary': LaunchConfiguration('debug_aruco_dictionary'),
        'debug_aruco_border_bits': LaunchConfiguration('debug_aruco_border_bits'),
        'debug_draw_marker_ids': LaunchConfiguration('debug_draw_marker_ids'),

        # Origin drawing options (only meaningful if publish_debug_images=True)
        'debug_draw_origin': LaunchConfiguration('debug_draw_origin'),
        'debug_origin_axis_length': LaunchConfiguration('debug_origin_axis_length'),
        'debug_origin_axis_width_px': LaunchConfiguration('debug_origin_axis_width_px'),
    }

    return [Node(
        package='aruco_map',
        executable='aruco_map_node',
        name='aruco_map',
        output='screen',
        parameters=[params],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='normal',
            description="Run mode: 'normal' or 'debug' (debug publishes debug plane images)",
        ),

        # Base
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('marker_size', default_value='0.16'),
        DeclareLaunchArgument('topic_name', default_value='/aruco/map/markers'),
        # Comment out the launch argument below and uncomment the one under it to fetch the map file config from /install instead of /src
        DeclareLaunchArgument(
            'map_file',
            default_value=PathJoinSubstitution([ARUCO_MAP_SRC_DIR, 'config', 'markers.txt']),
            description="Path to markers.txt (defaults to source tree markers.txt).",
        ),
#        DeclareLaunchArgument(
#            'map_file',
#            default_value=PathJoinSubstitution([FindPackageShare('aruco_map'), 'config', 'markers.txt']),
#            description=(
#                "Path to markers.txt. If empty, node uses its compiled default "
#                "(ament_index_cpp::get_package_share_directory('aruco_map') + '/config/markers.txt')."
#            ),
#        ),

        # Debug images publishing/timing
        DeclareLaunchArgument('debug_image_topic_prefix', default_value='/aruco/vis/map_debug_image/plane_'),
        DeclareLaunchArgument('debug_image_publish_rate_hz', default_value='1.0'),

        # Plane clustering parameters
        DeclareLaunchArgument('plane_angle_threshold_deg', default_value='10.0'),
        DeclareLaunchArgument('plane_distance_threshold_m', default_value='0.05'),

        # Debug image layout
        DeclareLaunchArgument('debug_pixels_per_meter', default_value='500.0'),
        DeclareLaunchArgument('debug_image_margin_px', default_value='40'),
        DeclareLaunchArgument('debug_image_max_size_px', default_value='4096'),

        # Debug marker rendering
        DeclareLaunchArgument('debug_aruco_dictionary', default_value='DICT_4X4_1000'),
        DeclareLaunchArgument('debug_aruco_border_bits', default_value='1'),
        DeclareLaunchArgument('debug_draw_marker_ids', default_value='false'),

        # Origin drawing
        DeclareLaunchArgument('debug_draw_origin', default_value='true'),
        DeclareLaunchArgument('debug_origin_axis_length', default_value='0.5'),
        DeclareLaunchArgument('debug_origin_axis_width_px', default_value='3'),

        OpaqueFunction(function=_make_node),
    ])
