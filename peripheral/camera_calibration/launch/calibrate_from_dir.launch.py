# ============================================================================
# calibrate_from_dir.launch.py — офлайн-калибровка по ПАПКЕ с готовыми кадрами.
#
# Нода на старте читает все изображения из input_dir, применяет конфигурацию
# доски (задаётся здесь же, БЕЗ сервисных вызовов), калибрует с авто-отсевом
# выбросов, сохраняет YAML и деплоит в config-папки драйвера. Затем завершается
# (batch_shutdown:=true).
#
# Пример (charuco 11x8, fisheye, папка снятых кадров):
#   ros2 launch camera_calibration calibrate_from_dir.launch.py \
#     input_dir:=/home/sverk/camera_calibrations/sessions/20260701_222434/captures \
#     board_type:=charuco camera_model:=fisheye \
#     board_width:=11 board_height:=8 square_size:=0.022 marker_size:=0.016 \
#     aruco_dict:=DICT_4X4_50 sensor_name:=imx219 \
#     deploy_driver_package:=mipi_csi_cam_ros2
# ============================================================================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lc = LaunchConfiguration
    params = {
        'batch_input_dir': lc('input_dir'),
        'batch_shutdown': ParameterValue(lc('batch_shutdown'), value_type=bool),
        # конфигурация доски
        'board_type': lc('board_type'),
        'camera_model': lc('camera_model'),
        'board_width': ParameterValue(lc('board_width'), value_type=int),
        'board_height': ParameterValue(lc('board_height'), value_type=int),
        'square_size': ParameterValue(lc('square_size'), value_type=float),
        'marker_size': ParameterValue(lc('marker_size'), value_type=float),
        'aruco_dict': lc('aruco_dict'),
        'min_frames': ParameterValue(lc('min_frames'), value_type=int),
        'min_charuco_corners': ParameterValue(lc('min_charuco_corners'), value_type=int),
        'max_frame_reproj': ParameterValue(lc('max_frame_reproj'), value_type=float),
        # выход/имя/деплой
        'camera_name': lc('camera_name'),
        'output_dir': lc('output_dir'),
        'sensor_name': lc('sensor_name'),
        'deploy_driver_package': lc('deploy_driver_package'),
        'workspace_src': lc('workspace_src'),
        'deploy_config_dir': lc('deploy_config_dir'),
    }

    return LaunchDescription([
        DeclareLaunchArgument('input_dir', description='Папка с готовыми кадрами (jpg/png)'),
        DeclareLaunchArgument('batch_shutdown', default_value='true'),

        DeclareLaunchArgument('board_type', default_value='charuco'),
        DeclareLaunchArgument('camera_model', default_value='fisheye'),
        DeclareLaunchArgument('board_width', default_value='11'),
        DeclareLaunchArgument('board_height', default_value='8'),
        DeclareLaunchArgument('square_size', default_value='0.022'),
        DeclareLaunchArgument('marker_size', default_value='0.016'),
        DeclareLaunchArgument('aruco_dict', default_value='DICT_4X4_50'),
        DeclareLaunchArgument('min_frames', default_value='15'),
        DeclareLaunchArgument('min_charuco_corners', default_value='15'),
        DeclareLaunchArgument('max_frame_reproj', default_value='2.0'),

        DeclareLaunchArgument('camera_name', default_value='camera_1'),
        DeclareLaunchArgument('output_dir', default_value='~/camera_calibrations'),
        DeclareLaunchArgument('sensor_name', default_value='imx219'),
        DeclareLaunchArgument('deploy_driver_package', default_value='a733_csi_cam_ros2'),
        DeclareLaunchArgument('workspace_src', default_value='/home/sverk/sverk_ws/src'),
        DeclareLaunchArgument('deploy_config_dir', default_value=''),

        Node(
            package='camera_calibration',
            executable='camera_calibration_node.py',
            name='camera_calibration_batch',
            output='screen',
            parameters=[params],
        ),
    ])
