#!/usr/bin/env python3
"""
lidar_odom_publisher.launch.py
------------------------------
LPE по лидарной одометрии (3D-лидар Unitree L2 + point_lio).

Нода lidar_odom_publisher — тонкий потребитель: берёт одометрию лидара
(nav_msgs/Odometry от point_lio, поза ЛИДАРА в мире одометрии), пересчитывает
в позу base_link в world ENU (T_post * T_odom * T_mount), конвертирует в
NED/FRD и шлёт в PX4 EKF2 on-new с ограничением частоты. Аналог
pose_subscriber_tf2.launch.py, но вход — лидар, а не ArUco.

Вход:
  /aft_mapped_to_init  (nav_msgs/Odometry) — одометрия point_lio

Дефолты mount/post — проверенные значения тестовой рамы ros2_uni
(лидар 17 см позади base_link, наклон -7° по тангажу, мир одометрии
довёрнут на 180° по yaw). ПОД СВОЁ КРЕПЛЕНИЕ — ПЕРЕМЕРИТЬ.

ВНИМАНИЕ: allow_send_vio по умолчанию false — одометрия в PX4 НЕ уходит,
пока явно не выставишь true (сначала проверь /aft_mapped_to_init и фреймы
на земле: подними/поверни раму руками и сравни знаки осей).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # Топик одометрии (вход от point_lio)
        DeclareLaunchArgument('odom_topic', default_value='/aft_mapped_to_init'),

        # Поза лидара на раме (base_link -> лидар) и доворот мира одометрии к ENU.
        # Значения — тестовая рама ros2_uni; перемерить под своё крепление!
        DeclareLaunchArgument('mount_rpy', default_value='[0.0, -0.122, 0.0]',
            description='RPY (рад) лидара относительно base_link (Rz*Ry*Rx).'),
        DeclareLaunchArgument('mount_xyz', default_value='[-0.170, 0.0, 0.0]',
            description='Смещение (м) лидара относительно base_link.'),
        DeclareLaunchArgument('post_rpy', default_value='[0.0, 0.0, 3.14159]',
            description='Доворот мира одометрии point_lio к world ENU (RPY, рад).'),

        # Поведение
        DeclareLaunchArgument('allow_send_vio', default_value='false',
            description='Разрешить отправку одометрии в PX4 EKF2. true — только после проверки фреймов.'),
        DeclareLaunchArgument('max_rate_hz', default_value='30.0',
            description='Ограничение частоты отправки в EKF2, Гц (0 = без лимита; point_lio может отдавать сотни Гц).'),
        DeclareLaunchArgument('watchdog_rate_hz', default_value='10.0',
            description='Частота watchdog-проверки устаревания данных, Гц (не частота отправки).'),
        DeclareLaunchArgument('data_timeout_sec', default_value='0.5',
            description='Таймаут (с) без свежей одометрии, после которого пишется предупреждение.'),

        Node(
            package='px4_local_pose_publisher',
            executable='lidar_odom_publisher',
            name='lidar_odom_publisher',
            output='screen',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                'mount_rpy':
                    ParameterValue(LaunchConfiguration('mount_rpy'), value_type=None),
                'mount_xyz':
                    ParameterValue(LaunchConfiguration('mount_xyz'), value_type=None),
                'post_rpy':
                    ParameterValue(LaunchConfiguration('post_rpy'), value_type=None),
                'allow_send_vio':
                    ParameterValue(LaunchConfiguration('allow_send_vio'), value_type=bool),
                'max_rate_hz':
                    ParameterValue(LaunchConfiguration('max_rate_hz'), value_type=float),
                'watchdog_rate_hz':
                    ParameterValue(LaunchConfiguration('watchdog_rate_hz'), value_type=float),
                'data_timeout_sec':
                    ParameterValue(LaunchConfiguration('data_timeout_sec'), value_type=float),
            }],
            arguments=['--ros-args', '--log-level', 'info'],
        )
    ])
