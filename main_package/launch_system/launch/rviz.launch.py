#!/usr/bin/env python3
"""Отдельный launch для запуска rviz2 в GUI-окне.

rviz2 — Qt/OpenGL-приложение, поэтому в SITL под WSLg рендерится тем же путём,
что и GUI-клиент gz в режиме wslg_gpu_gui full_system_sitl.launch.py:
DISPLAY=:0 + WSLg-сокет /mnt/wslg/.X11-unix/X0 + GALLIUM_DRIVER=d3d12 (GPU NVIDIA
через D3D12). Запускать отдельным вызовом рядом с уже поднятой симуляцией:

    ros2 launch launch_system rviz.launch.py

Аргументы:
    gpu        : true (по умолчанию) — рендер на GPU через d3d12 под WSLg;
                 false — программный llvmpipe (для headless/noVNC/не-WSLg хостов).
    rviz_config: путь к .rviz-конфигу; по умолчанию config/default.rviz из пакета,
                 если он есть, иначе rviz2 стартует с пустой сценой.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def _launch_rviz(context):
    gpu = LaunchConfiguration('gpu').perform(context).strip().lower() == 'true'
    rviz_config = LaunchConfiguration('rviz_config').perform(context).strip()

    # Дефолтный конфиг из пакета, если аргумент не задан и файл существует.
    if not rviz_config:
        default_cfg = os.path.join(
            get_package_share_directory('launch_system'), 'config', 'default.rviz')
        if os.path.isfile(default_cfg):
            rviz_config = default_cfg

    cmd = ['rviz2']
    if rviz_config:
        cmd += ['-d', rviz_config]

    # Общее окружение для WSLg-GUI (см. wslg_gpu_gui в full_system_sitl.launch.py).
    additional_env = {
        'DISPLAY': os.environ.get('DISPLAY', ':0'),
        'WAYLAND_DISPLAY': 'wayland-0',
        'XDG_RUNTIME_DIR': os.environ.get('XDG_RUNTIME_DIR', '/mnt/wslg/runtime-dir'),
        'QT_QPA_PLATFORM': 'xcb',
    }
    if gpu:
        # GPU NVIDIA через Mesa D3D12 — как gz-gui клиент в wslg_gpu_gui.
        additional_env['GALLIUM_DRIVER'] = 'd3d12'
        additional_env['LD_LIBRARY_PATH'] = \
            '/usr/lib/wsl/lib:' + os.environ.get('LD_LIBRARY_PATH', '')
    else:
        # Программный рендер — WSLg/GPU не нужен.
        additional_env['GALLIUM_DRIVER'] = 'llvmpipe'
        additional_env['LIBGL_ALWAYS_SOFTWARE'] = '1'

    # WSLg отдаёт X11-сокет только по /mnt/wslg/.X11-unix; rviz2 (xcb) ищет его
    # в /tmp/.X11-unix. Тот же симлинк, что делает px4_cmd в wslg_gpu_gui-режиме.
    prep = (
        'mkdir -p /tmp/.X11-unix && '
        '{ [ -S /tmp/.X11-unix/X0 ] || ln -sf /mnt/wslg/.X11-unix/X0 /tmp/.X11-unix/X0 ; } && '
        'exec ' + ' '.join(f'"{c}"' for c in cmd)
    )

    return [ExecuteProcess(
        cmd=['bash', '-c', prep],
        additional_env=additional_env,
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gpu', default_value='true',
            description='true: рендер rviz2 на GPU (d3d12/WSLg); false: llvmpipe (софт).'),
        DeclareLaunchArgument(
            'rviz_config', default_value='',
            description='Путь к .rviz-конфигу; пусто — config/default.rviz или пустая сцена.'),
        OpaqueFunction(function=_launch_rviz),
    ])
