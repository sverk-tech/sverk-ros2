from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = {
        'image_topic': LaunchConfiguration('image_topic'),
        'output_dir': LaunchConfiguration('output_dir'),
        'camera_name': LaunchConfiguration('camera_name'),
        'frame_id': LaunchConfiguration('frame_id'),
        'preview_rate_hz': ParameterValue(LaunchConfiguration('preview_rate_hz'), value_type=float),
        'publish_preview_when_idle': ParameterValue(
            LaunchConfiguration('publish_preview_when_idle'), value_type=bool
        ),
        'min_charuco_corners': ParameterValue(LaunchConfiguration('min_charuco_corners'), value_type=int),
        'max_frame_reproj': ParameterValue(LaunchConfiguration('max_frame_reproj'), value_type=float),
        'auto_deploy': ParameterValue(LaunchConfiguration('auto_deploy'), value_type=bool),
        'auto_deploy_max_reproj': ParameterValue(LaunchConfiguration('auto_deploy_max_reproj'), value_type=float),
        'deploy_driver_package': LaunchConfiguration('deploy_driver_package'),
        'workspace_src': LaunchConfiguration('workspace_src'),
        'deploy_config_dir': LaunchConfiguration('deploy_config_dir'),
        'sensor_name': LaunchConfiguration('sensor_name'),
    }

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera_1/image_raw'),
        DeclareLaunchArgument('output_dir', default_value='~/camera_calibrations'),
        DeclareLaunchArgument('camera_name', default_value='calibrated_camera'),
        DeclareLaunchArgument('frame_id', default_value='camera_optical_1'),
        DeclareLaunchArgument('preview_rate_hz', default_value='3.0'),
        DeclareLaunchArgument('publish_preview_when_idle', default_value='true'),
        DeclareLaunchArgument('min_charuco_corners', default_value='10'),
        # Авто-деплой готовой калибровки в config-папку драйвера камеры.
        # При auto_deploy=true и reproj_error <= auto_deploy_max_reproj YAML
        # сохраняется как <sensor_name>_<w>x<h>.yaml и копируется во ВСЕ найденные
        # config-папки пакета deploy_driver_package: и install/share (эффект сразу),
        # и src (переживает colcon build). Длинный путь указывать не нужно —
        # достаточно имени пакета драйвера.
        # Авто-отсев кадров-выбросов при калибровке: кадры с ошибкой репроекции
        # выше порога (px) итеративно выкидываются. 0 — отключить.
        DeclareLaunchArgument('max_frame_reproj', default_value='2.0'),
        DeclareLaunchArgument('auto_deploy', default_value='false'),
        DeclareLaunchArgument('auto_deploy_max_reproj', default_value='1.0'),
        DeclareLaunchArgument(
            'deploy_driver_package',
            default_value='a733_csi_cam_ros2',
            description='Имя пакета драйвера камеры; его config-папки (install+src) — цели деплоя',
        ),
        DeclareLaunchArgument(
            'workspace_src',
            default_value='/home/sverk/sverk_ws/src',
            description='Корень src workspace для поиска исходной config-папки драйвера',
        ),
        DeclareLaunchArgument(
            'deploy_config_dir',
            default_value='',
            description='Необязательный доп. путь для деплоя (сверх пакета драйвера)',
        ),
        DeclareLaunchArgument('sensor_name', default_value='imx219'),

        Node(
            package='camera_calibration',
            executable='camera_calibration_node.py',
            name='camera_calibration',
            output='screen',
            parameters=[params],
        ),
    ])