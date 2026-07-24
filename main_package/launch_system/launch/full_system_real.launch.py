#!/usr/bin/env python3
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch.substitutions import PathJoinSubstitution

def _env_bool(name, default):
    """ENABLE_* flags: читаем из окружения контейнера (пишет web-builder через
    runtime_env активных типов, image_builder/stages/70_firstboot.sh → device
    .env → docker compose пробрасывает в контейнер). Не задано — берём
    дефолт ниже (ручной docker run/старые образы без web-builder)."""
    v = os.environ.get(name)
    if v is None:
        return default
    return v.strip() not in ("", "0", "false", "False")


def generate_launch_description():
    ENABLE_ARUCO = _env_bool('ENABLE_ARUCO', True)
    ENABLE_CAMERA = _env_bool('ENABLE_CAMERA', True)
    ENABLE_VIO = _env_bool('ENABLE_VIO', True)            # Send ArUco to PX4
    ENABLE_MAVLINKROUTER = _env_bool('ENABLE_MAVLINKROUTER', False)  # Link to QC (mavlink-router)
    ENABLE_WEB = _env_bool('ENABLE_WEB', True)
    ENABLE_LED = _env_bool('ENABLE_LED', True)
    ENABLE_AI = _env_bool('ENABLE_AI', False)
    ENABLE_LIDAR = _env_bool('ENABLE_LIDAR', False)
    ENABLE_PID_TUNING = _env_bool('ENABLE_PID_TUNING', False)  # Read-only PID tuning assistant (web UI :8086). Diagnostic tool, off by default.
    ENABLE_SELF_CHECK = _env_bool('ENABLE_SELF_CHECK', False)   # self_check diagnostics node (action /self_check/run + services)
    ENABLE_SWARM = _env_bool('ENABLE_SWARM', False)        # модуль роевого взаимодействия (тип swarm)
    ENABLE_GRAFFITI = _env_bool('ENABLE_GRAFFITI', False)  # модуль «граффити»: серво PWM + VL53L1X (тип graffiti)
    ENABLE_LIDAR_3D = _env_bool('ENABLE_LIDAR_3D', False)  # модуль 3d lidar с отправкой данных одометрии в fmu
    ENABLE_LIDAR_3D_SLAM = _env_bool('ENABLE_LIDAR_3D_SLAM', False)  # point_lio SLAM -> /aft_mapped_to_init
    ENABLE_LIDAR_3D_ODOM = _env_bool('ENABLE_LIDAR_3D_ODOM', False)  # отправка данных одометрии в fmu с 3d lidar
    ENABLE_UWB = _env_bool('ENABLE_UWB', False)            # модуль UWB-позиционирования (тип uwb, ros_uwb)

    SELF_CHECK_LANGUAGE = os.environ.get('SELF_CHECK_LANGUAGE', 'en')  # язык ответов self_check (en|ru)

    # Платформа: из env контейнера (runtime_env base-типа, image_builder/
    # types.yaml → device .env). Дефолт ниже — для ручного docker run и
    # старых образов, собранных до перехода на MK_NAME-из-env.
    MK_NAME = (os.environ.get('MK_NAME') or 'ORANGE_3W').strip().upper()
                                      # RPI | RADXA | RDX | ORANGE | ORANGE_3W

    # ===== FRAME (формфактор дрона: obrick | gigaobrick) =====
    # SVERK_FRAME пишет frame-тип (frame_obrick | frame_gigaobrick,
    # image_builder/types.yaml). Геометрия рамы — config/frames/<frame>.yaml
    # этого пакета: статики камеры уходят драйверу аргументами tf_*
    # (mipi_csi_cam_ros2 / a733_csi_cam_ros2 публикуют static TF сами);
    # для RDK (вендорный mipi_cam без TF) — static_transform_publisher ниже
    # из тех же значений.
    SVERK_FRAME = (os.environ.get('SVERK_FRAME') or 'obrick').strip()
    # Дефолт = обрик (позиция линзы по CAD от центра платы FC, камера вниз);
    # источник и конвертация осей — config/frames/obrick.yaml.
    CAM_TF = {'x': 0.0735, 'y': 0.0, 'z': -0.0212,
              'roll': 3.1415926, 'pitch': 0.0, 'yaw': -1.5707963}
    try:
        _frame_yaml = os.path.join(
            get_package_share_directory('launch_system'),
            'config', 'frames', f'{SVERK_FRAME}.yaml')
        with open(_frame_yaml, encoding='utf-8') as _f:
            _frame_cfg = yaml.safe_load(_f) or {}
        CAM_TF.update((_frame_cfg.get('cameras') or [{}])[0].get('tf') or {})
    except Exception as e:  # нет файла/битый yaml — летим на дефолтах обрика
        print(f'[launch_system] frames/{SVERK_FRAME}.yaml не прочитан ({e}) — '
              f'статики камеры по умолчанию (обрик)')
    CAM_TF_LAUNCH_ARGS = {f'tf_{k}': str(v) for k, v in CAM_TF.items()}

    # ===== CAMERA PARAMETERS =====
    CAMERA_SENSOR = os.environ.get('CAMERA_SENSOR', 'imx219')
                                      # RPI: ov5647 | imx219 | imx708
                                      # RADXA: imx219 | ov5647
                                      # RDX: IMX219 | F37 | GC4663
                                      # ORANGE_3W: imx219 (calib: config/<sensor>_<w>x<h>.yaml)
    CAMERA_WIDTH = 1280                # Image width
    CAMERA_HEIGHT = 960               # Image height
    CAMERA_FPS = 30                   # Desired FPS
    CAMERA_ID = 1                     # Camera ID (multi-camera)
    IMAGE_TOPIC = f'/camera_{CAMERA_ID}/image_raw'          # Raw image topic
    CAMERA_INFO_TOPIC = f'/camera_{CAMERA_ID}/camera_info'  # Camera info topic
    CAMERA_FRAME_ID = f'camera_optical_{CAMERA_ID}'         # Camera frame_id for tf

    # ===== RDK X3 SPECIFIC =====
    RDX_MIPI_TOPIC = '/hbmem_img'           # Internal NV12 topic from mipi_cam
    RDX_CODEC_OUT_TOPIC = IMAGE_TOPIC       # bgr8 output for aruco / vio
    RDX_CODEC_JPEG_TOPIC = f'{IMAGE_TOPIC}/compressed'  # JPEG for web_video_server

    actions = []

    # =========================================================================
    # 1. MicroXRCEAgent
    # =========================================================================
    # --- Маппинг Serial-порта и параметров клиента по платформе ---
    XRCE_SERIAL_DEV = {
        "RPI":      "/dev/ttyAMA0",
        "RADXA":    "/dev/ttyS2",
        "RDX":      "/dev/ttyS3",
        "ORANGE":   "/dev/ttyS0",
        "ORANGE_3W": "/dev/ttyS2",
    }
    XRCE_BAUD = {
        "RPI":      "921600",
        "RADXA":    "921600",
        "RDX":      "921600",
        "ORANGE":   "1500000",
        "ORANGE_3W": "1500000",
    }

    # Client params on the FC (uxrce_dds_client) must match these.
    _xrce_dev  = XRCE_SERIAL_DEV.get(MK_NAME, "/dev/ttyAMA0")
    _xrce_baud = XRCE_BAUD.get(MK_NAME, "921600")

    # SCHED_FIFO for the agent: at 100% CPU a normal-priority serial thread
    # starves, the tty buffer (64K, ~0.7s at 921600) overflows and PX4 drops
    # the XRCE session. RT needs ulimits rtprio in docker-compose.yml; when
    # absent (old container, manual docker run) — fall back to a normal start,
    # otherwise the respawn loop would kill the agent entirely.
    _agent_cmd = f'MicroXRCEAgent serial --dev {_xrce_dev} -b {_xrce_baud}'
    actions.append(
        ExecuteProcess(
            cmd=['bash', '-c',
                 f'if chrt -f 30 true 2>/dev/null; then exec chrt -f 30 {_agent_cmd}; '
                 f'else echo "[micro_xrce_agent] rtprio unavailable, running SCHED_OTHER"; '
                 f'exec {_agent_cmd}; fi'],
            name='micro_xrce_agent',
            output='screen',
            shell=False,
            respawn=True,
            respawn_delay=8.0,
        )
    )

    # =========================================================================
    # 2. mavlink-routerd
    # =========================================================================
    # Link to QGC: UDP (default) or TCP — selected via the MAVLINK_LINK_MODE
    # env var (udp|tcp), set in docker-compose.yml (os service -> environment).
    # Port is fixed at 14550.
    MAVLINK_LINK_MODE = os.environ.get('MAVLINK_LINK_MODE', 'udp').strip().lower()
    MAVLINK_LINK_PORT = '14550'

    if ENABLE_MAVLINKROUTER:
        mavlink_cmd = ['mavlink-routerd', '/dev/ttyACM0:57600']
        if MAVLINK_LINK_MODE == 'tcp':
            # -p — port mavlink-routerd listens on for incoming TCP connections
            mavlink_cmd += ['-p', MAVLINK_LINK_PORT]
        else:
            # -e — UDP endpoint (receives/sends packets)
            mavlink_cmd += ['-e', f'0.0.0.0:{MAVLINK_LINK_PORT}']

        actions.append(
            ExecuteProcess(
                cmd=mavlink_cmd,
                name='mavlink-routerd',
                output='screen',
                shell=False,
                respawn=True,
                respawn_delay=1.0,
            )
        )

    # =========================================================================
    # 3. Camera driver
    # =========================================================================
    if ENABLE_CAMERA:
        if MK_NAME == "RPI":
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('mipi_csi_cam_ros2'),
                            'launch',
                            'camera.launch.py'
                        ])
                    ),
                    launch_arguments={
                        'sensor': CAMERA_SENSOR,
                        'width': str(CAMERA_WIDTH),
                        'height': str(CAMERA_HEIGHT),
                        'fps': str(CAMERA_FPS),
                        'camera_id': str(CAMERA_ID),
                        'source_type': 'libcamera',
                        'qos_reliable': 'true',
                        # Статики камеры рамы (frames/<SVERK_FRAME>.yaml) —
                        # драйвер публикует static TF; пока его camera.launch.py
                        # не объявляет tf_* — аргументы безвредно игнорируются.
                        **CAM_TF_LAUNCH_ARGS,
                    }.items()
                )
            )

        elif MK_NAME == "RADXA" or MK_NAME == "ORANGE":
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('mipi_csi_cam_ros2'),
                            'launch',
                            'camera.launch.py'
                        ])
                    ),
                    launch_arguments={
                        'sensor': CAMERA_SENSOR,
                        'width': str(CAMERA_WIDTH),
                        'height': str(CAMERA_HEIGHT),
                        'fps': str(CAMERA_FPS),
                        'camera_id': str(CAMERA_ID),
                        'source_type': 'v4l2',
                        'video_device': '/dev/video11',
                        'format': 'NV12',
                        'io_mode': 'dmabuf',
                        'qos_reliable': 'true',
                        **CAM_TF_LAUNCH_ARGS,   # статики рамы (см. ветку RPI)
                    }.items()
                )
            )

        elif MK_NAME == "ORANGE_3W":
            # a733-нода захватывает BGR24 через mmap; topic/frame_id выводятся
            # из camera_id внутри launch (camera_id=1 -> /camera_1/image_raw,
            # camera_optical_1), что совпадает с IMAGE_TOPIC / CAMERA_FRAME_ID.
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('a733_csi_cam_ros2'),
                            'launch',
                            'camera.launch.py'
                        ])
                    ),
                    launch_arguments={
                        'sensor': str(CAMERA_SENSOR),
                        'width': str(CAMERA_WIDTH),
                        'height': str(CAMERA_HEIGHT),
                        'fps': str(CAMERA_FPS),
                        'camera_id': str(CAMERA_ID),
                        'source_type': 'v4l2',
                        'video_device': '/dev/video8',
                        'format': 'BGR24',
                        'io_mode': 'mmap',
                        'qos_reliable': 'true',
                        'enable_isp': 'true',
                        **CAM_TF_LAUNCH_ARGS,   # статики рамы (см. ветку RPI)
                    }.items()
                )
            )

        elif MK_NAME == "RDX":
            actions.append(
                Node(
                    package='mipi_cam',
                    executable='mipi_cam',
                    name='mipi_cam',
                    output='log',
                    parameters=[
                        {'video_device': CAMERA_SENSOR},
                        {'image_width': CAMERA_WIDTH},
                        {'image_height': CAMERA_HEIGHT},
                        {'out_format': 'bgr8'},
                        # Update file calibration
                        # {'camera_calibration_file_path': '/home/sverk/sverk_ws/src/sverk_drone/.../config/home/sverk/imx219_640x480.yaml'},
                    ],
                    remappings=[
                        ('/image_raw', IMAGE_TOPIC),
                        ('/camera_info', CAMERA_INFO_TOPIC),
                    ],
                    arguments=['--ros-args', '--log-level', 'warn'],
                )
            )

            # =========================================================================
            # 3.2. Camera static TF: base_link -> $CAMERA_FRAME_ID
            #      Только RDK: вендорный mipi_cam TF не публикует. Значения —
            #      из frames/<SVERK_FRAME>.yaml (CAM_TF), не хардкод.
            # =========================================================================
            actions.append(
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='base_to_main_camera_optical_tf',
                    arguments=[
                        '--x', str(CAM_TF['x']), '--y', str(CAM_TF['y']), '--z', str(CAM_TF['z']),
                        '--yaw', str(CAM_TF['yaw']), '--pitch', str(CAM_TF['pitch']), '--roll', str(CAM_TF['roll']),
                        '--frame-id', 'base_link',
                        '--child-frame-id', CAMERA_FRAME_ID,
                    ],
                    output='screen',
                )
            )

    # =========================================================================
    # 4. Aruco
    # =========================================================================
    if ENABLE_ARUCO:
        # Весь ArUco-блок (карта + детект + локализация) вынесен в aruco.launch.py
        # Per-platform CPU-профиль (affinity/coarse/потоки) для aruco-контейнера:
        # без него composable-ноды не получают cpu_affinity_ids и на big.LITTLE
        # (A733) CV-потоки садятся на медленные A55.
        aruco_cpu_yaml = {
            "ORANGE_3W": "cpu_a733.yaml",
            "RPI":       "cpu_rpi5.yaml",
        }.get(MK_NAME)
        aruco_cpu_params = PathJoinSubstitution([
            FindPackageShare('aruco_det_loc'), 'config', aruco_cpu_yaml
        ]) if aruco_cpu_yaml else ''
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('launch_system'),
                        'launch',
                        'aruco.launch.py'
                    ])
                ]),
                launch_arguments={
                    'mode': 'debug',
                    'image_topic': IMAGE_TOPIC,
                    'camera_info_topic': CAMERA_INFO_TOPIC,
                    'tf_parent_frame': CAMERA_FRAME_ID,
                    'cpu_params_file': aruco_cpu_params,
                }.items()
            )
        )

    # =========================================================================
    # 5. PX4 local pose publisher
    # =========================================================================
    if ENABLE_VIO:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('px4_local_pose_publisher'),
                        'launch',
                        'pose_subscriber_tf2.launch.py'
                    ])
                ]),
                launch_arguments={
                    # Новый VPE — тонкий потребитель /aruco/world_pose от ArucoTfNode.
                    # Фреймы/offset больше не нужны (их считает ArucoTfNode).
                    # allow_send_vio: 'false' — поза в PX4 НЕ уходит, пока не проверишь её.
                    'allow_send_vio': 'true',
                    # Дропать позу старше этого (с) при отправке: гасит burst старых
                    # поз после залипа моста (иначе EKF2 reject/reset -> рывки).
                    'max_measurement_age_sec': '0.25',
                }.items()
            )
        )

    # =========================================================================
    # 6. Offboard control node
    # =========================================================================
    actions.append(
        Node(
            package='offboard_control',
            executable='offboard_control',
            name='offboard_control',
            parameters=[{
                'simulator': False,
                # Node stick-abort off (sticks disabled on PX4). Set 0.30-0.40 to test on RC.
                'rc_stick_abort_thresh': 0.0,
                'excite_stick_abort_thresh': 0.0,
                # RC loss handled by PX4 (NAV_RCL_ACT), not duplicated here.
                'manual_control_timeout': 0.0,
                # Odometry watchdog off: FMU owns position (optical flow + rangefinder).
                # Set enforce_freshness=True to re-enable the node-side net.
                'enforce_freshness': False,
                'onboard_failsafe_action': 'position',  # on trip: release stream -> PX4 Position
            }],
            output='screen'
        )
    )

    # =========================================================================
    # 7. Web
    # =========================================================================
    if ENABLE_WEB:
        # 7.1. Web video server (port 8080)
        # On RDX need set 7777 port
        # nice 10 on web/AI nodes: under peak CPU load the cores go to the
        # flight path (XRCE agent, offboard_control, vpe, camera->aruco), not
        # to JPEG encoding for the browser. With no CPU contention nice
        # doesn't limit anything.
        actions.append(
            Node(
                package='web_video_server',
                executable='web_video_server',
                name='web_video',
                parameters=[
                    {'port': 8080}, # On RPI and RADXA
                    # {'port': 7777}, # On RDK
                ],
                output='screen',
                prefix='nice -n 10',
            )
        )

        # 7.2. Rosboard (port 8888, explicit to avoid clashing with web_video_server)
        actions.append(
            Node(
                package='rosboard',
                executable='rosboard_node',
                name='rosboard',
                parameters=[
                    {'port': 8888},
                ],
                output='screen',
                prefix='nice -n 10',
            )
        )

        # 7.3. ROS Services Bridge (HTTP API, port 9090)
        actions.append(
            Node(
                package='ros_services_bridge',
                executable='ros_services_bridge_node',
                name='ros_services_bridge',
                parameters=[],
                output='screen'
            )
        )

        # 7.4. Send command to FMU
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('fmu_control'),
                        'launch',
                        'calibration_control.launch.py'
                    ])
                ]),
                launch_arguments={}.items()
            )
        )

    # =========================================================================
    # 7.5. PID Tuning Assistant (read-only telemetry recorder + PID advisor)
    #      Веб-UI на порту 8086 (8080 занят web_video_server). READ-ONLY к FMU:
    #      не армит, не пишет параметры. Диагностическая фича — по умолчанию
    #      выключена (ENABLE_PID_TUNING). Для полного набора rate-loop топиков
    #      нужен PID-блок в dds_topics.yaml (прошивка matek_h743-slim_pid_on.px4).
    # =========================================================================
    if ENABLE_PID_TUNING:
        actions.append(
            Node(
                package='pid_tuning_assistant',
                executable='pid_tuning_assistant',
                name='pid_tuning_assistant',
                arguments=['--port', '8086'],
                output='screen',
            )
        )

    # =========================================================================
    # 7.6. Self-check diagnostics (see docs/dev/system_check.md)
    #      Passive + interactive drone diagnostics, ROS action API. Never arms,
    #      never writes PX4 params (00_contract.md §8).
    # =========================================================================
    if ENABLE_SELF_CHECK:
        actions.append(
            Node(
                package='self_check',
                executable='self_check_node',  # см. self_check setup.py entry_points
                name='self_check',
                parameters=[{'platform': MK_NAME, 'language': SELF_CHECK_LANGUAGE}],
                output='screen',
            )
        )

    # =========================================================================
    # 8. Led control
    # =========================================================================
    if ENABLE_LED:
        # /dev/spidevX.Y differs per board (overlay/DT wiring) — empty string
        # means "no override, use led_params.yaml default" (see led.launch.py).
        LED_SPI = {
            "RADXA":     ("1", "0"),   # matches led_params.yaml default
            "RPI":     ("1", "0"),   # matches led_params.yaml default
            "ORANGE_3W": ("3", "0"),   # spi3-cs0-cs1-spidev overlay -> /dev/spidev3.0
        }
        _led_spi_bus, _led_spi_device = LED_SPI.get(MK_NAME, ("", ""))

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('led_control'),
                        'launch',
                        'led.launch.py'
                    ])
                ]),
                launch_arguments={
                    'spi_bus': _led_spi_bus,
                    'spi_device': _led_spi_device,
                }.items()
            )
        )

    # =========================================================================
    # 9. Demo AI
    # =========================================================================
    if ENABLE_AI:
        if MK_NAME == "RADXA" or MK_NAME == "ORANGE":
            # 9.1 Yolo 8 Pose + Seg detection on NPU
            actions.append(
                Node(
                    package='rknn_yolov8_pose_demo',
                    executable='rknn_yolov8_pose_demo',
                    name='rknn_yolov8_pose_demo',
                    parameters=[],
                    output='screen',
                    prefix='nice -n 10',
                )
            )
        elif MK_NAME == "RDX":
            # 9.2 Yolo Pose + Seg detection on BPU
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('yolo_bpu'),
                            'launch',
                            'yolo_bpu.launch.py',
                        ])
                    ),
                    launch_arguments={
                        'image_topic': IMAGE_TOPIC,
                    }.items(),
                )
            )
        # elif MK_NAME == "RPI":
        #     # 9.2 Yolo Pose + Seg detection on Hailo
        #     ...

    # =========================================================================
    # 10. 2D Lidar COIN-D4A DTOF (UART/USB) — драйвер ldlidar_stl_ros2
    #     Включается типом lidar_2d (image_builder/types.yaml runtime_env
    #     ENABLE_LIDAR=1 + LIDAR_PORT/LIDAR_BAUD/LIDAR_MODEL в device .env).
    #     Публикует: /scan (sensor_msgs/LaserScan)
    # =========================================================================
    if ENABLE_LIDAR:
        # executable/parameter names — стандартный публичный интерфейс
        # ldrobotSensorTeam/ldlidar_stl_ros2 на момент написания; пакет
        # клонируется в момент docker-сборки (Dockerfile.2d_lidar), поэтому
        # НЕ проверено против точной версии в этом чекауте — сверь при первом
        # запуске на реальном лидаре (`ros2 pkg xml ldlidar_stl_ros2`,
        # `ros2 run ldlidar_stl_ros2 <tab>` для точных имён).
        actions.append(
            Node(
                package='ldlidar_stl_ros2',
                executable='ldlidar_stl_ros2_node',
                name='ldlidar_publisher_ld',
                output='screen',
                parameters=[
                    {'product_name': os.environ.get('LIDAR_MODEL', 'LDLiDAR_LD06')},
                    {'topic_name': 'scan'},
                    {'frame_id': 'base_laser'},
                    {'port_name': os.environ.get('LIDAR_PORT', '/dev/ttyUSB0')},
                    {'port_baudrate': int(os.environ.get('LIDAR_BAUD', '230400'))},
                    {'laser_scan_dir': True},
                    {'enable_angle_crop_func': False},
                    {'angle_crop_min': 135.0},
                    {'angle_crop_max': 225.0},
                ],
            )
        )

    # (№11 зарезервирован: 3D-лидар Unitree L2 приходит с мерджем gigaobrick_3d)

    # =========================================================================
    # 12. Роевое взаимодействие (тип swarm, ENABLE_SWARM в device .env).
    #     Контракт модулей: пакет несёт свой launch/<пакет>.launch.py, здесь —
    #     только включение по флагу. Пакет появится в
    #     sverk_ws/src/sverk_drone/modules/swarm; до этого ENABLE_SWARM=1 даёт
    #     предупреждение, а не падение всей системы на старте.
    # =========================================================================
    if ENABLE_SWARM:
        try:
            get_package_share_directory('swarm')
        except PackageNotFoundError:
            print('[launch_system] ENABLE_SWARM=1, но пакета swarm нет в образе — блок пропущен')
        else:
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('swarm'),
                            'launch',
                            'swarm.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'swarm_id': os.environ.get('SWARM_ID', '1'),
                    }.items()
                )
            )

    # =========================================================================
    # 13. Модуль «граффити»: серво (PWM) + лазерный дальномер VL53L1X (I2C).
    #     Тип graffiti (только рама обрик). Контракт тот же: пакет
    #     modules/graffiti несёт launch/graffiti.launch.py — внутри серво-нода
    #     и существующая VL53L1X-нода из peripheral/ (драйвер не дублируем).
    # =========================================================================
    if ENABLE_GRAFFITI:
        try:
            get_package_share_directory('graffiti')
        except PackageNotFoundError:
            print('[launch_system] ENABLE_GRAFFITI=1, но пакета graffiti нет в образе — блок пропущен')
        else:
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('graffiti'),
                            'launch',
                            'graffiti.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'tof_i2c_bus': os.environ.get('GRAFFITI_TOF_I2C_BUS', '1'),
                    }.items()
                )
            )

    # =========================================================================
    # 14. 3D Lidar Unitree L2 (unilidar_sdk2: Ethernet/UDP или Serial)
    #     Включается через ENABLE_LIDAR_3D=1 в docker-compose.3d_lidar.yml
    #     (драйвер запекается аддоном Dockerfile.3d_lidar; без него секция спит)
    #     Публикует: /unilidar/cloud (sensor_msgs/PointCloud2),
    #                /unilidar/imu   (sensor_msgs/Imu)
    # =========================================================================
    if ENABLE_LIDAR_3D:
        actions.append(
            Node(
                package='unitree_lidar_ros2',
                executable='unitree_lidar_ros2_node',
                name='unitree_lidar_l2',
                output='screen',
                parameters=[{
                    # 1 = Serial (USB), 2 = UDP/Ethernet
                    'initialize_type': int(os.environ.get('LIDAR_3D_INIT_TYPE', '2')),
                    'work_mode': int(os.environ.get('LIDAR_3D_WORK_MODE', '0')),
                    'use_system_timestamp': True,
                    'range_min': 0.0,
                    'range_max': 100.0,
                    'cloud_scan_num': 18,
                    # Serial
                    'serial_port': os.environ.get('LIDAR_3D_SERIAL_PORT', '/dev/ttyACM0'),
                    'baudrate': int(os.environ.get('LIDAR_3D_BAUD', '4000000')),
                    # UDP: лидар шлёт поток на <IP хоста>:local_port; в bridge-сети
                    # контейнера биндимся на 0.0.0.0, порт проброшен compose-оверлеем
                    'lidar_ip': os.environ.get('LIDAR_3D_LIDAR_IP', '192.168.1.62'),
                    'lidar_port': int(os.environ.get('LIDAR_3D_LIDAR_PORT', '6101')),
                    'local_ip': os.environ.get('LIDAR_3D_LOCAL_IP', '0.0.0.0'),
                    'local_port': int(os.environ.get('LIDAR_3D_LOCAL_PORT', '6201')),
                    'cloud_frame': 'unilidar_lidar',
                    'cloud_topic': 'unilidar/cloud',
                    'imu_frame': 'unilidar_imu',
                    'imu_topic': 'unilidar/imu',
                }],
            )
        )
        # Поза лидара относительно рамы — подстрой смещение под своё крепление
        actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='lidar_3d_tf',
                arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'unilidar_lidar'],
            )
        )

    # ── 14.1 Лидарная одометрия -> PX4 EKF2 (lidar_odom_publisher) ──────────
    # Включается отдельно от драйвера: нужен источник одометрии point_lio
    # (топик /aft_mapped_to_init), см. docs/dev/quick_start_orangepi5pro_3d_lidar.md.
    # Отправка в PX4 дополнительно закрыта предохранителем LIDAR_3D_ODOM_SEND_VIO.
    if ENABLE_LIDAR_3D_ODOM:
        def _env_floats(name, default):
            return [float(v) for v in os.environ.get(name, default).split(',')]
        actions.append(
            Node(
                package='px4_local_pose_publisher',
                executable='lidar_odom_publisher',
                name='lidar_odom_publisher',
                output='screen',
                parameters=[{
                    'odom_topic': os.environ.get('LIDAR_3D_ODOM_TOPIC', '/aft_mapped_to_init'),
                    # Проверенные значения тестовой рамы ros2_uni — перемерить под своё крепление!
                    'mount_rpy': _env_floats('LIDAR_3D_MOUNT_RPY', '0.0,-0.122,0.0'),
                    'mount_xyz': _env_floats('LIDAR_3D_MOUNT_XYZ', '-0.170,0.0,0.0'),
                    'post_rpy':  _env_floats('LIDAR_3D_POST_RPY',  '0.0,0.0,3.14159'),
                    # Предохранитель: по умолчанию одометрия в PX4 НЕ уходит
                    'allow_send_vio': os.environ.get('LIDAR_3D_ODOM_SEND_VIO', '0') == '1',
                    'max_rate_hz': float(os.environ.get('LIDAR_3D_ODOM_MAX_RATE', '30.0')),
                }],
            )
        )

    # ── 14.2 Point-LIO SLAM (источник одометрии для 14.1) ──────────────────
    # Включается через ENABLE_LIDAR_3D_SLAM=1 (device .env / compose-оверлей).
    # Запускает mapping_unilidar_l2.launch.py из пакета point_lio (аддон
    # Dockerfile.3d_lidar). Подписывается на /unilidar/cloud + /unilidar/imu
    # (от драйвера, секция 14), публикует /aft_mapped_to_init (nav_msgs/Odometry),
    # который читает секция 14.1 (lidar_odom_publisher -> PX4 EKF2).
    # rviz:=false — на плате нет дисплея, rviz падает в headless.
    if ENABLE_LIDAR_3D_SLAM:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('point_lio'),
                        'launch', 'mapping_unilidar_l2.launch.py'
                    ])
                ),
                launch_arguments={'rviz': 'false'}.items()
            )
        )

    # =========================================================================
    # 15. UWB-позиционирование (тип uwb, ENABLE_UWB в device .env). Гигаобрик.
    #     Пакет ros_uwb (submodule airframes/ros-uwb) собран в base-образе.
    #     Нода uwb_node читает UWB-модуль по serial (на гигаобрике — /dev/ttyS6,
    #     overlay uart6-m1) и публикует /uwb/result + /uwb/peer_info.
    #     У пакета нет своего launch — поднимаем ноду напрямую. Нет пакета в
    #     образе (ENABLE_UWB=1 без submodule) → предупреждение, не падение.
    # =========================================================================
    if ENABLE_UWB:
        try:
            get_package_share_directory('ros_uwb')
        except PackageNotFoundError:
            print('[launch_system] ENABLE_UWB=1, но пакета ros_uwb нет в образе — блок пропущен')
        else:
            actions.append(
                Node(
                    package='ros_uwb',
                    executable='uwb_node',
                    name='uwb_node',
                    output='screen',
                    parameters=[{
                        'port': os.environ.get('UWB_PORT', '/dev/ttyS6'),
                        'baudrate': int(os.environ.get('UWB_BAUD', '500000')),
                    }],
                )
            )

    return LaunchDescription(actions)


if __name__ == '__main__':
    generate_launch_description()

# colcon build --packages-select launch_system