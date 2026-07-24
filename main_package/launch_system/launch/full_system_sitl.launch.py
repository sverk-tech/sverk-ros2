#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import getpass

OBRIK_CONFIG_MODELS = {
    'base': 'x500_obrik_base',
    'lidar': 'x500_obrik_lidar',
    'one_rangefinder': 'x500_obrik_one_rangefinder',
    'three_rangefinders_30': 'x500_obrik_three_rangefinders_30',
    'three_rangefinders_45': 'x500_obrik_three_rangefinders_45',
    'graffiti': 'x500_obrik_graffiti',
    'graffiti_old': 'x500_obrik_graffiti_old',
    'gigaobrik': 'x500_obrik_gigaobrik',
}
GRAFFITI_MODELS = ('x500_obrik_graffiti', 'x500_obrik_graffiti_old')

ARUCO_MAP_SRC_DIR = '/home/sverk/sverk_ws/src/sverk_drone/odometry/aruco/aruco_map'
DEFAULT_X500_MODEL = 'x500_mono_cam_down'

# Friendly ROS-facing camera topics. The gz camera sensor publishes on a long
# /world/<world>/model/.../sensor/imager/image path; the bridge remaps it to
# these short names so the camera is easy to find/use in rviz/rqt/rosboard.
# Matches the real drone's /camera_*/image_raw convention and the rangefinder
# bridge, which already remaps to /obrik/rangefinder_down/scan.
CAMERA_ID = 1                     # Camera ID (multi-camera)
IMAGE_TOPIC = f'/camera_{CAMERA_ID}/image_raw'          # Raw image topic
CAMERA_INFO_TOPIC = f'/camera_{CAMERA_ID}/camera_info'  # Camera info topic
CAMERA_FRAME_ID = f'camera_optical_{CAMERA_ID}'         # Camera frame_id for tf


def _llvmpipe_threads():
    # Кап потоков llvmpipe для софтверного рендера сенсоров. Без капа llvmpipe
    # берёт nproc потоков и выедает ВСЕ ядра — GUI-клиент, физика gz, px4
    # (lockstep) и DDS голодают, что снаружи выглядит как фризы/низкий FPS.
    # По умолчанию оставляем 4 ядра остальному стеку; переопределяется
    # переменной окружения LP_NUM_THREADS.
    explicit = os.environ.get('LP_NUM_THREADS', '').strip()
    if explicit:
        return explicit
    return str(max(2, (os.cpu_count() or 8) - 4))

def _is_obrik_model(model):
    return model.startswith('x500_obrik_')

def _resolve_obrik_model(context):
    config = LaunchConfiguration('obrik_config').perform(context).strip()
    if config in ('', 'x500', 'stock', 'legacy'):
        return DEFAULT_X500_MODEL
    if config in OBRIK_CONFIG_MODELS:
        return OBRIK_CONFIG_MODELS[config]
    if config.startswith('x500_'):
        return config
    valid = ', '.join(sorted(OBRIK_CONFIG_MODELS))
    raise ValueError(f"Unknown obrik_config '{config}'. Use one of: x500, stock, legacy, {valid}, or pass a full x500_* model name.")

def _camera_topics(context):
    model = _resolve_obrik_model(context)
    gz_world = LaunchConfiguration('gz_world').perform(context).strip()
    model_instance = f'{model}_0'
    camera_link = 'rpi_camera_link' if _is_obrik_model(model) else 'camera_link'
    image_topic = f'/world/{gz_world}/model/{model_instance}/link/{camera_link}/sensor/imager/image'
    camera_info_topic = f'/world/{gz_world}/model/{model_instance}/link/{camera_link}/sensor/imager/camera_info'
    return image_topic, camera_info_topic

def _make_px4_launch(context, px4_dir):
    headless = LaunchConfiguration('headless').perform(context).strip().lower() == 'true'
    wslg_gui = LaunchConfiguration('wslg_gui').perform(context).strip().lower() == 'true'
    wslg_gpu_gui = LaunchConfiguration('wslg_gpu_gui').perform(context).strip().lower() == 'true'
    host_x11_gui = LaunchConfiguration('host_x11_gui').perform(context).strip().lower() == 'true'
    vnc_gui = LaunchConfiguration('vnc_gui').perform(context).strip().lower() == 'true'
    obrik_model = _resolve_obrik_model(context)
    gz_world = LaunchConfiguration('gz_world').perform(context).strip()
    enable_vio = LaunchConfiguration('enable_vio').perform(context).strip().lower() == 'true'
    allow_send_vio = LaunchConfiguration('allow_send_vio').perform(context).strip().lower() == 'true'

    # PX4 v1.15.4: with PX4_SIM_MODEL set and PX4_GZ_MODEL_NAME UNSET, px4-rc.simulator
    # runs `gz_bridge start -m <model> -i <instance>` which SPAWNS the entity <model>_0.
    # Setting PX4_GZ_MODEL_NAME switches gz_bridge to attach-only (no spawn), which left
    # the drone missing and bridge topics empty. See full_system_sitl porting notes.
    px4_spawn_params = ''
    if (
        _is_obrik_model(obrik_model)
        and obrik_model not in ('x500_obrik_gigaobrik', 'x500_obrik_graffiti_old')
    ):
        # Center the downward camera over the takeoff marker at world origin.
        px4_spawn_params = ' PX4_GZ_MODEL_POSE=0,0.0615,0,0,0,0'

    px4_sensor_params = ''
    if obrik_model == 'x500_obrik_gigaobrik':
        px4_sensor_params = (
            ' PX4_PARAM_EKF2_OF_CTRL=1'
            ' PX4_PARAM_EKF2_OF_POS_X=0.0'
            ' PX4_PARAM_EKF2_OF_POS_Y=0.081'
            ' PX4_PARAM_EKF2_OF_POS_Z=-0.015801'
        )
    elif _is_obrik_model(obrik_model):
        if obrik_model == 'x500_obrik_graffiti_old':
            # Model pose is FLU (0, -0.025, -0.099); PX4 expects body FRD.
            range_pos_y, range_pos_z = 0.025, 0.099
        else:
            # Model pose is FLU (0, 0.0475, -0.009); PX4 expects body FRD.
            range_pos_y, range_pos_z = -0.0475, 0.009
        px4_sensor_params = (
            ' PX4_PARAM_EKF2_HGT_REF=2'
            ' PX4_PARAM_EKF2_RNG_CTRL=2'
            ' PX4_PARAM_EKF2_RNG_POS_X=0.0'
            f' PX4_PARAM_EKF2_RNG_POS_Y={range_pos_y}'
            f' PX4_PARAM_EKF2_RNG_POS_Z={range_pos_z}'
        )
        if enable_vio:
            # Fuse ArUco horizontal position and yaw; range remains the height source.
            px4_sensor_params += ' PX4_PARAM_EKF2_EV_CTRL=9 PX4_PARAM_EKF2_EV_QMIN=0'
            if allow_send_vio:
                # GPS holds the estimator at the origin until the ground ArUco pose is stable.
                px4_sensor_params += ' PX4_PARAM_EKF2_GPS_CTRL=7'
    px4_run_env = (
        f'PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_{obrik_model} '
        f'PX4_GZ_WORLD={gz_world}{px4_spawn_params}{px4_sensor_params}'
    )
    # edit_rcS.bash patches px4-rc.mavlink with a unicast target at the host IP
    # (or host.docker.internal) so MAVLink/QGroundControl reaches out of the
    # container — PX4 relies on broadcast by default, which doesn't cross
    # Docker NAT. The file only appears after px4_sitl_default builds, so we
    # patch it between the build and starting the binary.
    edit_rcs_cmd = 'bash /home/sverk/builder/scripts/edit_rcS.bash'
    px4_cmd = (
        f'{edit_rcs_cmd} && '
        'rm -f parameters.bson parameters_backup.bson && '
        f'{px4_run_env} ./build/px4_sitl_default/bin/px4'
    )
    additional_env = {}

    if headless:
        # px4-rc.simulator при HEADLESS=1 запускает сервер как
        # `gz sim -r -s <world>` БЕЗ --headless-rendering: HEADLESS у PX4
        # выключает только GUI-клиент. Сенсорам (камера обрика) сервер
        # тогда поднимает обычный GL-контекст; на WSLg это Mesa d3d12,
        # который ТЕЧЁТ ~2.5 ГБ/мин (gz-sim#2873) до OOM всей WSL VM —
        # т.е. «headless» вовсе не был безопасным режимом. Поэтому
        # запускаем gz сами с -s --headless-rendering и переводим px4 в
        # PX4_GZ_STANDALONE=1 — та же схема, что в ветках wslg_gui/vnc_gui
        # ниже. ВАЖНО: одного --headless-rendering мало — при проброшенном
        # /dev/dxg EGL-headless тоже выбирает d3d12 (GL_RENDERER = "D3D12
        # (NVIDIA ...)") и течёт с той же скоростью (проверено), поэтому
        # рендер сенсоров дополнительно форсируем в llvmpipe, как в
        # vnc_gui. Вернуть GPU (когда апстрим починят):
        # HEADLESS_GALLIUM_DRIVER=d3d12, либо '' — автовыбор Mesa.
        px4_gz_worlds = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'worlds')
        px4_gz_models = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')
        gz_server_cmd = f'gz sim --verbose=1 -r -s --headless-rendering "{gz_world}.sdf" &'
        px4_cmd = (
            f'{gz_server_cmd} '
            'sleep 3 && '
            f'{edit_rcs_cmd} && '
            f'PX4_GZ_STANDALONE=1 {px4_run_env} ./build/px4_sitl_default/bin/px4'
        )
        additional_env = {
            'GZ_SIM_RESOURCE_PATH': f'{px4_gz_models}:{px4_gz_worlds}',
        }
        headless_gallium = os.environ.get('HEADLESS_GALLIUM_DRIVER', 'llvmpipe')
        if headless_gallium:
            additional_env['GALLIUM_DRIVER'] = headless_gallium
        if headless_gallium == 'llvmpipe':
            additional_env['LIBGL_ALWAYS_SOFTWARE'] = '1'
            additional_env['LP_NUM_THREADS'] = _llvmpipe_threads()
    elif host_x11_gui:
        additional_env = {
            'DISPLAY': os.environ.get('DISPLAY', ':0'),
            'QT_QPA_PLATFORM': 'xcb',
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
        }
    elif wslg_gui:
        # Gazebo (server + GUI, one process — exactly what px4-rc.simulator
        # would run) is started HERE and px4 runs with PX4_GZ_STANDALONE=1 —
        # the same spawn-timeout cure as vnc_gui below: on a cold GUI start
        # Gazebo needs longer than the single 1s attempt PX4's gz_bridge gives
        # the `create` spawn service, and px4 exits with "Service call timed
        # out", leaving the world without a drone. With PX4_GZ_STANDALONE=1
        # gz_bridge retries every 2s until the server answers.
        # GZ_SIM_RESOURCE_PATH must include worlds/ too: px4-rc.simulator no
        # longer launches gz, so nobody else resolves the world .sdf for us.
        px4_gz_worlds = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'worlds')
        px4_gz_models = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')
        gz_cmd = f'gz sim --verbose=1 -r "{gz_world}.sdf" &'
        px4_cmd = (
            'mkdir -p /tmp/.X11-unix && '
            '[ -S /tmp/.X11-unix/X0 ] || ln -sf /mnt/wslg/.X11-unix/X0 /tmp/.X11-unix/X0 && '
            f'{gz_cmd} '
            'sleep 3 && '
            f'{edit_rcs_cmd} && '
            f'PX4_GZ_STANDALONE=1 {px4_run_env} ./build/px4_sitl_default/bin/px4'
        )
        additional_env = {
            'DISPLAY': ':0',
            'WAYLAND_DISPLAY': 'wayland-0',
            'XDG_RUNTIME_DIR': '/mnt/wslg/runtime-dir',
            'PULSE_SERVER': 'unix:/mnt/wslg/PulseServer',
            'QT_QPA_PLATFORM': 'xcb',
            'GZ_SIM_RESOURCE_PATH': f'{px4_gz_models}:{px4_gz_worlds}',
        }
        # Forces the WSLg D3D12 GPU passthrough path. Some GPU/driver combos
        # (seen with Intel Arc as of WSL 2.7.10 / WSLg 1.0.73) crash outright
        # ("failed to create drisw screen") instead of falling back to
        # llvmpipe when this is forced — set WSLG_GALLIUM_DRIVER='' (empty)
        # to let Mesa auto-select instead (software rendering, but no crash).
        #
        # ВНИМАНИЕ: на NVIDIA (RTX 3070, Win10, Mesa 23.2 d3d12) d3d12-путь
        # ТЕЧЁТ ~2-5 ГБ/мин (ogre2 x Mesa d3d12, апстрим gz-sim#2873): текут
        # и GUI, и сервер (рендер камеры), пустой мир тоже. Для сессий
        # дольше пары минут запускать с WSLG_GALLIUM_DRIVER=llvmpipe —
        # софтверный рендер стабилен (проверено: ~0 роста за 7+ мин).
        # mem_limit в docker-compose.sitl.wslg.yml страхует VM от OOM.
        wslg_gallium_driver = os.environ.get('WSLG_GALLIUM_DRIVER', 'd3d12')
        if wslg_gallium_driver:
            additional_env['GALLIUM_DRIVER'] = wslg_gallium_driver
        if wslg_gallium_driver == 'llvmpipe':
            additional_env['LP_NUM_THREADS'] = _llvmpipe_threads()
    elif wslg_gpu_gui:
        # Компромиссный GPU-режим для WSLg, пока жив gz-sim#2873 (ogre2 x
        # Mesa d3d12 течёт ~2-5 ГБ/мин): сервер рендерит сенсоры софтверно
        # (llvmpipe, -s --headless-rendering, standalone — как в vnc_gui
        # ниже), а GUI-клиент идёт ОТДЕЛЬНЫМ процессом на ogre1 через
        # d3d12 (GPU). ogre1 не использует паттерн ogre-next, который
        # триггерит утечку (проверено: GL_RENDERER = D3D12/NVIDIA, RSS
        # без роста). Графика GUI беднее ogre2 (ogre1 deprecated), зато
        # окно живое и память стабильна.
        px4_gz_worlds = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'worlds')
        px4_gz_models = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')
        gz_server_cmd = (
            f'GALLIUM_DRIVER=llvmpipe LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS={_llvmpipe_threads()} '
            f'gz sim --verbose=1 -r -s --headless-rendering "{gz_world}.sdf" &'
        )
        gz_gui_cmd = 'GALLIUM_DRIVER=d3d12 gz sim -g --render-engine-gui ogre &'
        px4_cmd = (
            'mkdir -p /tmp/.X11-unix && '
            '[ -S /tmp/.X11-unix/X0 ] || ln -sf /mnt/wslg/.X11-unix/X0 /tmp/.X11-unix/X0 && '
            f'{gz_server_cmd} '
            'sleep 3 && '
            f'{gz_gui_cmd} '
            f'{edit_rcs_cmd} && '
            f'PX4_GZ_STANDALONE=1 {px4_run_env} ./build/px4_sitl_default/bin/px4'
        )
        additional_env = {
            'DISPLAY': ':0',
            'WAYLAND_DISPLAY': 'wayland-0',
            'XDG_RUNTIME_DIR': '/mnt/wslg/runtime-dir',
            'PULSE_SERVER': 'unix:/mnt/wslg/PulseServer',
            'QT_QPA_PLATFORM': 'xcb',
            'GZ_SIM_RESOURCE_PATH': f'{px4_gz_models}:{px4_gz_worlds}',
        }
    elif vnc_gui:
        # Software rendering inside the container's own Xvfb (:99), streamed
        # out via x11vnc/noVNC — see scripts/sitl/Dockerfile.sitl.novnc and
        # docker-compose.sitl.novnc.yml. No GPU/WSLg/D3D12/host X11 involved,
        # so this works on any host, including ones where GPU passthrough
        # into Docker is broken or unavailable.
        #
        # We start Gazebo (server + GUI) OURSELVES here and run px4 with
        # PX4_GZ_STANDALONE=1, instead of letting px4-rc.simulator launch gz.
        # Why: PX4's gz_bridge gives Gazebo's `create` spawn service exactly
        # one attempt with a 1s timeout (src/modules/simulation/gz_bridge/
        # GZBridge.cpp). On llvmpipe, `gz sim` startup is slower than that, so
        # px4 gives up and exits with "Service call timed out" before Gazebo
        # is ready. With PX4_GZ_STANDALONE=1, gz_bridge instead retries every
        # 2s until the server answers, and px4-rc.simulator skips its own
        # (now redundant) gz launch (`if [ -z "${PX4_GZ_STANDALONE}" ]`).
        #
        # --headless-rendering on the server is REQUIRED: without it, camera/
        # depth sensor topics publish zero messages even though the 3D GUI
        # renders fine — those sensors need their own offscreen render context
        # that server-only mode doesn't reliably provide under llvmpipe.
        px4_gz_worlds = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'worlds')
        px4_gz_models = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')
        additional_env = {
            'DISPLAY': ':99',
            'QT_QPA_PLATFORM': 'xcb',
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'GALLIUM_DRIVER': 'llvmpipe',
            # Let the gz server/GUI we launch below resolve the world .sdf and
            # the drone model that gz_bridge asks the server to spawn.
            'GZ_SIM_RESOURCE_PATH': f'{px4_gz_models}:{px4_gz_worlds}',
        }
        # Cap llvmpipe's worker threads if the operator set LP_NUM_THREADS in
        # the compose env (start_sitl_novnc.sh documents this knob but the
        # launch process doesn't inherit its exports — set it here).
        lp_num_threads = os.environ.get('LP_NUM_THREADS', '').strip()
        if lp_num_threads:
            additional_env['LP_NUM_THREADS'] = lp_num_threads
        # gz server headless (offscreen render for sensors) + separate GUI
        # client, both before px4. `-r` unpauses so sensors publish; the GUI
        # attaches to the running server. gz_bridge (in standalone px4) then
        # retries until the server is up, so no fixed wait is strictly needed —
        # a short settle before the GUI/px4 just trims reconnect noise.
        gz_server_cmd = f'gz sim --verbose=1 -r -s --headless-rendering "{gz_world}.sdf" &'
        gz_gui_cmd = 'gz sim -g &'
        px4_cmd = (
            f'{gz_server_cmd} '
            'sleep 3 && '
            f'{gz_gui_cmd} '
            f'{edit_rcs_cmd} && '
            f'PX4_GZ_STANDALONE=1 {px4_run_env} ./build/px4_sitl_default/bin/px4'
        )

    spray_plugin_path = '/home/sverk/sverk_ws/install/gz_sim_spray_painting_plugin/lib/gz_sim_spray_painting_plugin'
    spray_plugin_library = f'{spray_plugin_path}/libSprayPaintPlugin.so'
    spray_plugin_source = '/home/sverk/sverk_ws/src/gz_sim_spray_painting_plugin'
    spray_plugin_repo = 'https://git.sverk.io/software-ros2/gz_sim_spray_painting_plugin.git'
    spray_plugin_ref = 'runtime-config'

    enable_spray = os.environ.get('ENABLE_SPRAY_PLUGIN', '0') == '1'
    if enable_spray:
        spray_plugin_setup_cmd = (
            f'if [ ! -f "{spray_plugin_library}" ]; then '
            'source /opt/ros/humble/setup.bash && '
            'mkdir -p /home/sverk/sverk_ws/src && '
            f'if [ ! -d "{spray_plugin_source}/.git" ]; then '
            f'rm -rf "{spray_plugin_source}" && '
            f'git clone --depth 1 --branch "{spray_plugin_ref}" "{spray_plugin_repo}" "{spray_plugin_source}"; '
            'fi && '
            'cd /home/sverk/sverk_ws && '
            'colcon build --symlink-install --packages-select gz_sim_spray_painting_plugin; '
            'fi'
        )
        px4_cmd = f'{spray_plugin_setup_cmd} && {px4_cmd}'

    if enable_spray and os.path.exists(spray_plugin_library):
        existing_gz_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        additional_env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = (
            f'{spray_plugin_path}:{existing_gz_plugin_path}' if existing_gz_plugin_path else spray_plugin_path
        )

    return [ExecuteProcess(
        cmd=['bash', '-lc', px4_cmd],
        name='px4_sitl_gazebo',
        output='screen',
        cwd=px4_dir,
        additional_env=additional_env,
    )]


def _make_px4_param_setup(context):
    obrik_model = _resolve_obrik_model(context)
    enable_vio = LaunchConfiguration('enable_vio').perform(context).strip().lower() == 'true'
    allow_send_vio = LaunchConfiguration('allow_send_vio').perform(context).strip().lower() == 'true'
    settings = [('NAV_DLL_ACT', '0')]

    if _is_obrik_model(obrik_model) and obrik_model != 'x500_obrik_gigaobrik':
        if obrik_model == 'x500_obrik_graffiti_old':
            range_pos_y, range_pos_z = '0.025', '0.099'
        else:
            range_pos_y, range_pos_z = '-0.0475', '0.009'
        settings.extend([
            ('EKF2_HGT_REF', '2'),
            ('EKF2_RNG_CTRL', '2'),
            ('EKF2_RNG_POS_X', '0.0'),
            ('EKF2_RNG_POS_Y', range_pos_y),
            ('EKF2_RNG_POS_Z', range_pos_z),
        ])
        if enable_vio:
            settings.extend([
                ('EKF2_EV_CTRL', '9'),
                ('EKF2_EV_QMIN', '0'),
            ])
            if allow_send_vio:
                settings.append(('EKF2_GPS_CTRL', '7'))

    cmd = ['ros2', 'run', 'launch_system', 'set_px4_sitl_params']
    for name, value in settings:
        cmd.extend(['--param', name, '--value', value])
    if (
        enable_vio
        and allow_send_vio
        and _is_obrik_model(obrik_model)
        and obrik_model != 'x500_obrik_gigaobrik'
    ):
        cmd.append('--handoff-gps-after-vio')
    return [ExecuteProcess(cmd=cmd, name='px4_sitl_param_setup', output='screen')]

def _make_gz_bridge_nodes(context):
    obrik_model = _resolve_obrik_model(context)
    gz_world = LaunchConfiguration('gz_world').perform(context).strip()
    model_instance = f'{obrik_model}_0'

    image_topic, camera_info_topic = _camera_topics(context)
    if obrik_model == 'x500_obrik_gigaobrik':
        rangefinder_topic = f'/world/{gz_world}/model/{model_instance}/link/lidar_sensor_link/sensor/lidar/scan'
        rangefinder_ros_topic = '/obrik/mtf01/rangefinder/scan'
    else:
        rangefinder_topic = f'/world/{gz_world}/model/{model_instance}/link/obrik_rangefinder_down_link/sensor/obrik_rangefinder_down/scan'
        rangefinder_ros_topic = '/obrik/rangefinder_down/scan'

    def scan_bridge(name, gz_topic, ros_topic):
        return Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=name,
            arguments=[f'{gz_topic}@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
            remappings=[(gz_topic, ros_topic)],
            output='screen'
        )

    bridge_nodes = []
    if obrik_model != 'x500_obrik_gigaobrik':
        bridge_nodes = [
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='image_bridge',
                arguments=[f'{image_topic}@sensor_msgs/msg/Image@gz.msgs.Image'],
                remappings=[(image_topic, IMAGE_TOPIC)],
                output='screen'
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='camera_info_bridge',
                arguments=[f'{camera_info_topic}@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
                remappings=[(camera_info_topic, CAMERA_INFO_TOPIC)],
                output='screen'
            ),
            Node(
                package='launch_system',
                executable='camera_info_latch',
                name='camera_info_latch',
                parameters=[{'topic': CAMERA_INFO_TOPIC}],
                output='screen'
            )
        ]

    if not _is_obrik_model(obrik_model):
        return bridge_nodes

    bridge_nodes.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rangefinder_down_bridge',
        arguments=[f'{rangefinder_topic}@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        remappings=[(rangefinder_topic, rangefinder_ros_topic)],
        output='screen'
    ))

    if obrik_model in GRAFFITI_MODELS:
        bridge_nodes.extend([
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='graffiti_servo_travel_bridge',
                arguments=['/spray/servo/travel@std_msgs/msg/Float64@gz.msgs.Double'],
                output='screen'
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='graffiti_spray_command_bridge',
                arguments=['/spray@std_msgs/msg/Bool@gz.msgs.Boolean'],
                output='screen'
            ),
        ])

    if obrik_model == 'x500_obrik_lidar':
        lidar_topic = f'/world/{gz_world}/model/{model_instance}/link/obrik_lidar_link/sensor/obrik_lidar/scan'
        bridge_nodes.append(scan_bridge('obrik_lidar_bridge', lidar_topic, '/obrik/lidar/scan'))

    elif obrik_model in ('x500_obrik_one_rangefinder', 'x500_obrik_graffiti'):
        slam_rangefinder_topic = f'/world/{gz_world}/model/{model_instance}/link/obrik_slam_rangefinder_link/sensor/obrik_slam_rangefinder/scan'
        bridge_nodes.append(scan_bridge('obrik_slam_rangefinder_bridge', slam_rangefinder_topic, '/obrik/slam_rangefinder/scan'))

    elif obrik_model in ('x500_obrik_three_rangefinders_30', 'x500_obrik_three_rangefinders_45'):
        rangefinder_topics = {
            'center': f'/world/{gz_world}/model/{model_instance}/link/obrik_slam_rangefinder_center_link/sensor/obrik_slam_rangefinder_center/scan',
            'right': f'/world/{gz_world}/model/{model_instance}/link/obrik_slam_rangefinder_right_link/sensor/obrik_slam_rangefinder_right/scan',
            'left': f'/world/{gz_world}/model/{model_instance}/link/obrik_slam_rangefinder_left_link/sensor/obrik_slam_rangefinder_left/scan',
        }
        for side, topic in rangefinder_topics.items():
            bridge_nodes.append(scan_bridge(
                f'obrik_slam_rangefinder_{side}_bridge',
                topic,
                f'/obrik/slam_rangefinders/{side}/scan'
            ))

    return bridge_nodes


def _make_rangefinder_px4_bridge_nodes(context):
    obrik_model = _resolve_obrik_model(context)
    if not _is_obrik_model(obrik_model) or obrik_model == 'x500_obrik_gigaobrik':
        return []

    return [Node(
        package='rangefinder_px4_bridge',
        executable='rangefinder_px4_bridge',
        name='rangefinder_px4_bridge',
        parameters=[{
            'input_topic': '/obrik/rangefinder_down/scan',
            'output_topic': '/fmu/in/distance_sensor',
            'device_id': 1,
            'min_distance': 0.03,
            'max_distance': 4.0,
            'variance': 0.0004,
        }],
        output='screen',
    )]


def _make_graffiti_servo_sim_nodes(context):
    obrik_model = _resolve_obrik_model(context)
    if obrik_model not in GRAFFITI_MODELS:
        return []

    if obrik_model == 'x500_obrik_graffiti_old':
        servo_parameters = {
            'min_angle_deg': 30.0,
            'max_angle_deg': 180.0,
            'center_angle_deg': 90.0,
            'pressed_angle_deg': 180.0,
            'startup_angle_deg': 90.0,
            'start_enabled': True,
            'servo_speed_deg_per_sec': 360.0,
            'reverse_travel_fraction': 0.6666666667,
            'publish_spray_command': True,
            'spray_active_angle_deg': 170.0,
        }
    else:
        servo_parameters = {
            'min_angle_deg': 0.0,
            'max_angle_deg': 156.52,
            'center_angle_deg': 90.0,
            'pressed_angle_deg': 156.52,
            'startup_angle_deg': 90.0,
            'start_enabled': True,
            'servo_speed_deg_per_sec': 360.0,
            'reverse_travel_fraction': 1.3529765484,
            'publish_spray_command': True,
            'spray_active_angle_deg': 137.104942,
        }

    return [Node(
        package='graffiti_servo_sim',
        executable='graffiti_servo_sim_node',
        name='servo_node',
        parameters=[servo_parameters],
        output='screen'
    )]


def _make_led_strip_sim_nodes(context):
    if not _is_obrik_model(_resolve_obrik_model(context)):
        return []

    config = os.path.join(
        get_package_share_directory('led_strip_sim'),
        'config',
        'led_params.yaml',
    )
    return [Node(
        package='led_strip_sim',
        executable='led_strip_sim_node',
        name='led_strip_sim_node',
        parameters=[config],
        output='screen',
    )]

def _resolve_aruco_map_file(context):
    explicit_map_file = LaunchConfiguration('aruco_map_file').perform(context).strip()
    if explicit_map_file:
        return explicit_map_file

    gz_world = LaunchConfiguration('gz_world').perform(context).strip()
    marker_file = 'markers_large.txt' if gz_world == 'obrik_aruco' else 'markers.txt'
    marker_path = os.path.join(ARUCO_MAP_SRC_DIR, 'config', marker_file)
    if os.path.exists(marker_path):
        return marker_path
    return os.path.join(ARUCO_MAP_SRC_DIR, 'config', 'markers.txt')

def _make_aruco_map_launch(context):
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aruco_map'),
                'launch',
                'aruco_map.launch.py'
            ])
        ]),
        launch_arguments={
            'map_file': _resolve_aruco_map_file(context),
            'marker_size': LaunchConfiguration('aruco_marker_size'),
        }.items()
    )]

def _make_aruco_pipeline_launch(context):
    if _resolve_obrik_model(context) == 'x500_obrik_gigaobrik':
        return []

    # Detect + Loc + Tf in a single composable container — the SAME pipeline as
    # full_system_real.launch.py (aruco.launch.py -> aruco_composed). The bridge
    # publishes on the friendly ROS topics; aruco subscribes there.
    #
    # The old SITL wiring launched aruco_detect.launch.py and aruco_loc.launch.py
    # separately, which (a) dropped ArucoTfNode entirely, so /aruco/world_pose
    # (the VPE input) was never published and the world TFs were missing, and
    # (b) left aruco_loc publishing its own TF (composed sets publish_tf=False and
    # lets aruco_tf own the world frames). aruco_composed fixes both.
    #
    # parent_frame_id / tf_parent_frame MUST equal the camera optical frame that
    # _make_camera_tf_node publishes (CAMERA_FRAME_ID). The old code relied on the
    # launch defaults (camera_optical_1) while the static TF published
    # main_camera_optical — a frame mismatch that broke the base_link<->camera
    # chain. Both now come from CAMERA_FRAME_ID.
    image_topic, camera_info_topic = IMAGE_TOPIC, CAMERA_INFO_TOPIC
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aruco_det_loc'),
                'launch',
                'aruco_composed.launch.py'
            ])
        ]),
        launch_arguments={
            'image_topic': image_topic,
            'camera_info_topic': camera_info_topic,
            'parent_frame_id': CAMERA_FRAME_ID,
            'tf_parent_frame': CAMERA_FRAME_ID,
            'marker_size': LaunchConfiguration('aruco_marker_size'),
            'dictionary_id': LaunchConfiguration('aruco_dictionary_id'),
            'initialize_at_origin': 'true',
        }.items()
    )]

def _make_sensor_tf_nodes(context):
    obrik_model = _resolve_obrik_model(context)
    if not _is_obrik_model(obrik_model):
        return []

    model_instance = f'{obrik_model}_0'

    def static_tf(name, x, y, z, yaw, pitch, roll, child_frame):
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=name,
            arguments=[
                str(x), str(y), str(z),
                str(yaw), str(pitch), str(roll),
                'base_link',
                child_frame,
            ],
            output='screen'
        )

    if obrik_model == 'x500_obrik_gigaobrik':
        rangefinder_down_pose = (0, 0.081000, -0.015801)
        rangefinder_down_rotation = (0, 1.5707963, 3.1415926)
        rangefinder_down_frame = f'{model_instance}/lidar_sensor_link/lidar'
    elif obrik_model == 'x500_obrik_graffiti_old':
        rangefinder_down_pose = (0, -0.025000, -0.099000)
        rangefinder_down_rotation = (0, 1.5707963, 0)
        rangefinder_down_frame = f'{model_instance}/obrik_rangefinder_down_link/obrik_rangefinder_down'
    else:
        rangefinder_down_pose = (0, 0.047500, -0.009000)
        rangefinder_down_rotation = (0, 1.5707963, 0)
        rangefinder_down_frame = f'{model_instance}/obrik_rangefinder_down_link/obrik_rangefinder_down'

    tf_nodes = [
        static_tf(
            'rangefinder_down_sensor_tf_publisher',
            rangefinder_down_pose[0], rangefinder_down_pose[1], rangefinder_down_pose[2],
            rangefinder_down_rotation[0], rangefinder_down_rotation[1], rangefinder_down_rotation[2],
            rangefinder_down_frame
        )
    ]

    if obrik_model == 'x500_obrik_gigaobrik':
        tf_nodes.extend([
            static_tf(
                'mtf01_link_tf_publisher',
                0, 0.081000, -0.015801,
                0, 0, 0,
                f'{model_instance}/mtf01_link'
            ),
            static_tf(
                'mtf01_optical_tf_publisher',
                0, 0.081000, -0.015801,
                0, 1.5707963, 0,
                f'{model_instance}/mtf01_optical'
            ),
        ])

    if obrik_model == 'x500_obrik_lidar':
        tf_nodes.append(static_tf(
            'obrik_lidar_sensor_tf_publisher',
            0, -0.051500, 0.026600,
            0, 0, 0,
            f'{model_instance}/obrik_lidar_link/obrik_lidar'
        ))

    elif obrik_model in ('x500_obrik_one_rangefinder', 'x500_obrik_graffiti'):
        if obrik_model == 'x500_obrik_graffiti':
            rangefinder_pose = (0, 0.057490, 0.080600)
            rangefinder_yaw = 1.5707963
        else:
            rangefinder_pose = (0, -0.050510, 0.028500)
            rangefinder_yaw = -1.5707963
        tf_nodes.append(static_tf(
            'obrik_slam_rangefinder_sensor_tf_publisher',
            rangefinder_pose[0], rangefinder_pose[1], rangefinder_pose[2],
            rangefinder_yaw, 0, 0,
            f'{model_instance}/obrik_slam_rangefinder_link/obrik_slam_rangefinder'
        ))

    elif obrik_model in ('x500_obrik_three_rangefinders_30', 'x500_obrik_three_rangefinders_45'):
        tf_nodes.append(static_tf(
            'obrik_slam_rangefinder_center_sensor_tf_publisher',
            0, -0.085100, 0.025600,
            -1.5707963, 0, 0,
            f'{model_instance}/obrik_slam_rangefinder_center_link/obrik_slam_rangefinder_center'
        ))

        if obrik_model == 'x500_obrik_three_rangefinders_30':
            side_specs = [
                ('right', 0.025263, -0.078741, -1.0471976),
                ('left', -0.025263, -0.078741, -2.0943951),
            ]
        else:
            side_specs = [
                ('right', 0.023614, -0.075729, -0.7853982),
                ('left', -0.023614, -0.075729, -2.3561945),
            ]

        for side, x, y, yaw in side_specs:
            tf_nodes.append(static_tf(
                f'obrik_slam_rangefinder_{side}_sensor_tf_publisher',
                x, y, 0.025600,
                yaw, 0, 0,
                f'{model_instance}/obrik_slam_rangefinder_{side}_link/obrik_slam_rangefinder_{side}'
            ))

    return tf_nodes

def _make_camera_tf_node(context):
    model = _resolve_obrik_model(context)
    if model == 'x500_obrik_gigaobrik':
        return []
    if model == 'x500_obrik_graffiti_old':
        arguments = ['0', '-0.044845', '-0.099000', '0', '0', '3.1415926', 'base_link', CAMERA_FRAME_ID]
    elif _is_obrik_model(model):
        # Gazebo's camera looks along link +X; ROS optical axes are +Z forward,
        # +X right, +Y down. The Obrik SDF mounting therefore needs roll=pi only.
        arguments = ['0', '-0.061500', '-0.003100', '0', '0', '3.1415926', 'base_link', CAMERA_FRAME_ID]
    else:
        arguments = ['0', '0', '-0.10', '-1.5707963', '0', '3.1415926', 'base_link', CAMERA_FRAME_ID]

    return [Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=arguments,
        output='screen'
    )]

def _make_vio_launch(context):
    # VPE — thin consumer of /aruco/world_pose from ArucoTfNode: converts base_link
    # pose (ENU/FLU) to NED/FRD and feeds it to PX4 EKF2 on-new. Same wiring as
    # full_system_real.launch.py (ENABLE_VIO). Gated by enable_vio (default true).
    # allow_send_vio gates whether the pose actually reaches PX4 — set it false to
    # observe /aruco/world_pose without feeding EKF2 (check frames on the ground).
    if _resolve_obrik_model(context) == 'x500_obrik_gigaobrik':
        return []
    if LaunchConfiguration('enable_vio').perform(context).strip().lower() != 'true':
        return []
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('px4_local_pose_publisher'),
                'launch',
                'pose_subscriber_tf2.launch.py'
            ])
        ]),
        launch_arguments={
            'allow_send_vio': LaunchConfiguration('allow_send_vio'),
        }.items()
    )]

def _make_pid_tuning_nodes(context):
    # Read-only PID tuning assistant (web UI :8086). Same node as
    # full_system_real.launch.py (ENABLE_PID_TUNING). Off by default; enable for
    # PID tests against SITL with enable_pid_tuning:=true. Needs the rate-loop DDS
    # topics from the PX4 fork (dds_topics.yaml PID block) to see the inner loop.
    if LaunchConfiguration('enable_pid_tuning').perform(context).strip().lower() != 'true':
        return []
    return [Node(
        package='pid_tuning_assistant',
        executable='pid_tuning_assistant',
        name='pid_tuning_assistant',
        arguments=['--port', '8086'],
        output='screen',
    )]

def _make_self_check_nodes(context):
    # Self-check diagnostics node (see docs/dev/system_check.md). Same node as
    # full_system_real.launch.py (ENABLE_SELF_CHECK). No board-specific SITL
    # profile exists, so it runs against the RPI profile (self_check's own
    # fallback for an unrecognized platform). Never arms, never writes PX4
    # params (00_contract.md §8).
    if LaunchConfiguration('enable_self_check').perform(context).strip().lower() != 'true':
        return []
    lang = LaunchConfiguration('self_check_language').perform(context).strip().lower()
    if lang not in ('en', 'ru'):
        lang = 'en'
    return [Node(
        package='self_check',
        executable='self_check_node',  # см. self_check setup.py entry_points
        name='self_check',
        parameters=[{'platform': 'RPI', 'language': lang}],
        output='screen',
    )]

def generate_launch_description():
    # Resolve the container user's home directory.
    home_dir = os.path.expanduser(f'~{getpass.getuser()}')
    px4_dir = os.path.join(home_dir, 'PX4-Autopilot')

    # Check that the PX4 directory exists.
    if not os.path.exists(px4_dir):
        print(f"WARNING: PX4 directory not found at {px4_dir}. Make sure it exists.")

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo without GUI.',
        ),
        DeclareLaunchArgument(
            'wslg_gui',
            default_value='false',
            description='Render Gazebo through WSLg. Start Docker with docker-compose.wslg.yml.',
        ),
        DeclareLaunchArgument(
            'wslg_gpu_gui',
            default_value='false',
            description='WSLg GUI on the GPU via ogre1 (leak-safe while gz-sim#2873 is open): '
                        'server renders sensors on llvmpipe, GUI client runs ogre1 through d3d12. '
                        'Start Docker with docker-compose.sitl.wslg.yml.',
        ),
        DeclareLaunchArgument(
            'host_x11_gui',
            default_value='false',
            description='Render Gazebo on a native Linux host X11/XWayland display. Start Docker with docker-compose.linux-host.yml.',
        ),
        DeclareLaunchArgument(
            'vnc_gui',
            default_value='false',
            description='Render Gazebo in software (llvmpipe) inside the container\'s own Xvfb, viewable via noVNC. '
                        'For hosts where GPU passthrough is broken/unavailable. Start with docker-compose.sitl.novnc.yml.',
        ),
        DeclareLaunchArgument(
            'obrik_config',
            default_value='x500',
            description='Vehicle model: x500/stock/legacy for the default PX4 x500 camera model, base/lidar/one_rangefinder/three_rangefinders_30/three_rangefinders_45/graffiti/graffiti_old/gigaobrik for Obrik, or a full x500_* model name.',
        ),
        DeclareLaunchArgument(
            'gz_world',
            default_value='aruco',
            description='Gazebo world suffix used in the PX4 make target, for example aruco in gz_<model>_aruco.',
        ),
        DeclareLaunchArgument(
            'aruco_map_file',
            default_value='',
            description='Optional markers.txt path for aruco_map. If empty, obrik_aruco uses markers_large.txt and other worlds use markers.txt.',
        ),
        DeclareLaunchArgument(
            'aruco_marker_size',
            default_value='0.4',
            description='Default marker size for ArUco detection when a per-marker size is not available.',
        ),
        DeclareLaunchArgument(
            'aruco_dictionary_id',
            default_value='3',
            description='OpenCV ArUco dictionary id. 3 is DICT_4X4_1000.',
        ),
        DeclareLaunchArgument(
            'enable_pid_tuning',
            default_value='false',
            description='Start the read-only PID tuning assistant (web UI :8086) for PID tests '
                        'against SITL. Off by default; needs the PX4-fork rate-loop DDS topics.',
        ),
        DeclareLaunchArgument(
            'enable_self_check',
            default_value='true',
            description='Start the self_check diagnostics node (action /self_check/run + '
                        'services). Same as ENABLE_SELF_CHECK in full_system_real.',
        ),
        DeclareLaunchArgument(
            'self_check_language',
            default_value='en',
            description="self_check diagnostics language: 'en' (default) or 'ru'.",
        ),
        DeclareLaunchArgument(
            'enable_vio',
            default_value='true',
            description='Start the VPE node (px4_local_pose_publisher): consume /aruco/world_pose '
                        'and send it to PX4 EKF2. Same as ENABLE_VIO in full_system_real.',
        ),
        DeclareLaunchArgument(
            'allow_send_vio',
            default_value='true',
            description='Actually push the visual pose into PX4 EKF2. Set false to observe '
                        '/aruco/world_pose without feeding PX4 (verify frames on the ground first).',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Run every ROS node on Gazebo simulation time via a /clock bridge '
                        '(added below when true). OFF by default to preserve the verified '
                        'wall-time wiring; enable for honest sim-time stamps across the whole '
                        'TF/EV pipeline (fixes garbage markers_age_ms and mixed-clock TF '
                        'timestamps). Needs a sim smoke-test after enabling.',
        ),

        # Sim time: when use_sim_time:=true, apply it to EVERY node in this launch
        # (including included descriptions). Placed before all nodes so they inherit
        # it; the matching /clock bridge is started with the gz bridges below.
        SetParameter(name='use_sim_time', value=True,
                     condition=IfCondition(LaunchConfiguration('use_sim_time'))),

        # 1. PX4 SITL with Gazebo, launched as a separate process from the PX4 directory
        OpaqueFunction(function=lambda context: _make_px4_launch(context, px4_dir)),

        # 1c. /clock bridge (only when use_sim_time:=true): republish Gazebo sim
        # time as ROS /clock so every node above shares the simulator's clock.
        # Without this, enabling use_sim_time would freeze all nodes at time 0.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_sim_time')),
        ),

        # 1b. Disable the SITL-only requirement for an active ground control station.
        # QGroundControl can still connect, but PX4 will not block arming when it is absent.
        OpaqueFunction(function=_make_px4_param_setup),
        
        # 2. MicroXRCEAgent, launched as a separate process
        ExecuteProcess(
            cmd=[
                'bash',
                '-lc',
                (
                    'agent_bin="${MICRO_XRCE_AGENT:-/home/sverk/sverk_ws/install/microxrcedds_agent/bin/MicroXRCEAgent}"; '
                    'if [ ! -x "$agent_bin" ]; then agent_bin="$(command -v MicroXRCEAgent)"; fi; '
                    'exec env LD_LIBRARY_PATH="/home/sverk/sverk_ws/install/microxrcedds_agent/lib:/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}" '
                    '"$agent_bin" udp4 -p 8888'
                ),
            ],
            name='micro_xrce_agent',
            output='screen',
        ),

        # 3-4b. Gazebo bridges for the selected model configuration
        OpaqueFunction(function=lambda context: _make_gz_bridge_nodes(context)),
        OpaqueFunction(function=lambda context: _make_rangefinder_px4_bridge_nodes(context)),

        # Simulation adapters used by the override models.
        OpaqueFunction(function=lambda context: _make_graffiti_servo_sim_nodes(context)),
        OpaqueFunction(function=lambda context: _make_led_strip_sim_nodes(context)),

        # 4c. Static TFs for Obrik Gazebo sensor frames used by LaserScan messages
        OpaqueFunction(function=lambda context: _make_sensor_tf_nodes(context)),
        
        # 5. Static transform for the camera
        OpaqueFunction(function=lambda context: _make_camera_tf_node(context)),
       
        # 6. Aruco map node
        OpaqueFunction(function=lambda context: _make_aruco_map_launch(context)),


        # 7. Aruco detect + loc + tf pipeline (composable container).
        # Same node set as full_system_real (aruco_composed): ArucoDetectNode,
        # ArucoLocNode, ArucoTfNode. Publishes /aruco/det/markers, /aruco/loc/pose_cov
        # and /aruco/world_pose (the VPE input). Replaces the old separate detect/loc
        # includes, which never started ArucoTfNode.
        OpaqueFunction(function=lambda context: _make_aruco_pipeline_launch(context)),

        # 9. VPE — /aruco/world_pose -> PX4 EKF2 (px4_local_pose_publisher).
        # Same as full_system_real ENABLE_VIO; gated by enable_vio (default true),
        # with allow_send_vio controlling whether the pose actually reaches PX4.
        OpaqueFunction(function=lambda context: _make_vio_launch(context)),

        # 10. Offboard control node
        Node(
            package='offboard_control',
            executable='offboard_control',
            name='offboard_control',
            parameters=[
                {'simulator': True}
            ],
            output='screen'
        ),

        # 11. FMU control services (arm/reboot/ESC/motor test/calibration).
        # Same as full_system_real.launch.py; provides the /fmu_control/* services
        # that qgc-web's DDS backend and ros_services_bridge call. Not simulated by
        # Gazebo — pure logic over the uXRCE-DDS bridge.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('fmu_control'),
                    'launch',
                    'calibration_control.launch.py'
                ])
            ]),
            launch_arguments={}.items()
        ),

        # 12. Web bridges — same set as full_system_real.launch.py ENABLE_WEB.
        # start_sitl.sh only starts SSH/butterfly/code-server, so without these the
        # SITL web containers (dotmdviz on :8888/:9090, qgc-web, browser MJPEG) have
        # nothing to connect to. Software bridges, not simulated by Gazebo.
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video',
            parameters=[{'port': 8080}],
            output='screen'
        ),
        Node(
            package='rosboard',
            executable='rosboard_node',
            name='rosboard',
            parameters=[{'port': 8888}],
            output='screen'
        ),
        Node(
            package='ros_services_bridge',
            executable='ros_services_bridge_node',
            name='ros_services_bridge',
            parameters=[],
            output='screen'
        ),

        # 13. PID tuning assistant (read-only, web UI :8086). Gated by
        # enable_pid_tuning (default false). qgc-web proxies it via PID_ASSIST_URL.
        OpaqueFunction(function=lambda context: _make_pid_tuning_nodes(context)),

        # 14. Self-check diagnostics node. Gated by enable_self_check (default true).
        OpaqueFunction(function=lambda context: _make_self_check_nodes(context)),
    ])

if __name__ == '__main__':
    generate_launch_description()
