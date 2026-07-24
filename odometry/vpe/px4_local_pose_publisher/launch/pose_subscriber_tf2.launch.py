#!/usr/bin/env python3
"""
pose_subscriber_tf2.launch.py
-----------------------------
VPE (Visual Position Estimate) для нового ArUco composed-пайплайна.

Нода pose_subscriber_tf2 — тонкий потребитель: берёт готовую позу base_link
в world-фрейме (ENU, тело FLU) от ArucoTfNode, конвертирует в NED/FRD и шлёт
в PX4 EKF2 on-new (по приходу позы, со штампом времени захвата кадра).
Вся TF/offset-логика — в ArucoTfNode (aruco_composed.launch.py), здесь её нет.

Вход:
  /aruco/world_pose     (geometry_msgs/PoseWithCovarianceStamped, ENU)
  /aruco/reset_counter  (std_msgs/UInt8)

ВНИМАНИЕ: allow_send_vio по умолчанию false — поза в PX4 НЕ уходит, пока явно
не выставишь true (сначала проверь /aruco/world_pose и фреймы на земле).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # Топики (входы от ArucoTfNode)
        DeclareLaunchArgument('world_pose_topic',    default_value='/aruco/world_pose'),
        DeclareLaunchArgument('reset_counter_topic', default_value='/aruco/reset_counter'),

        # Поведение
        DeclareLaunchArgument('allow_send_vio',   default_value='false',
            description='Разрешить отправку позы в PX4 EKF2. true — только после проверки позы/фреймов.'),
        DeclareLaunchArgument('watchdog_rate_hz', default_value='10.0',
            description='Частота watchdog-проверки устаревания данных, Гц (не частота отправки — она on-new).'),
        DeclareLaunchArgument('data_timeout_sec', default_value='0.5',
            description='Таймаут (с) без свежей позы, после которого пишется предупреждение (отправка прекращается).'),
        DeclareLaunchArgument('max_measurement_age_sec', default_value='0.25',
            description='Дропать позу, если она старше этого (с) на момент отправки: после залипа '
                        'моста очередь best_effort сбрасывает пачку старых поз, а такой burst '
                        'ломает EKF2 (reject/reset за горизонтом фьюжена). <=0 отключает.'),

        Node(
            package='px4_local_pose_publisher',
            executable='pose_subscriber_tf2',
            name='pose_subscriber_tf2',
            output='screen',
            parameters=[{
                'world_pose_topic':    LaunchConfiguration('world_pose_topic'),
                'reset_counter_topic': LaunchConfiguration('reset_counter_topic'),
                'allow_send_vio':
                    ParameterValue(LaunchConfiguration('allow_send_vio'), value_type=bool),
                'watchdog_rate_hz':
                    ParameterValue(LaunchConfiguration('watchdog_rate_hz'), value_type=float),
                'data_timeout_sec':
                    ParameterValue(LaunchConfiguration('data_timeout_sec'), value_type=float),
                'max_measurement_age_sec':
                    ParameterValue(LaunchConfiguration('max_measurement_age_sec'), value_type=float),
            }],
            arguments=['--ros-args', '--log-level', 'info'],
        )
    ])
