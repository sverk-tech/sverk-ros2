#!/usr/bin/env python3
"""
aruco.launch.py
---------------
Единая точка запуска и настройки всего ArUco-блока (новый composable-пайплайн).
full_system_real.launch.py подключает этот файл одной ссылкой.

Запускает:
  1. aruco_map  (отдельный процесс)
     — публикует карту маркеров /aruco/map/markers (transient_local, latched).
       В composable-контейнер не кладётся: transient_local несовместим с IPC.

  2. aruco_composed (ОДИН процесс, ComposableNodeContainer "aruco_container"):
       • ArucoDetectNode — детекция маркеров  -> /aruco/det/markers
       • ArucoLocNode    — глобальная поза камеры -> /aruco/loc/pose_cov
       • ArucoTfNode     — TF + поза base_link -> /aruco/world_pose, /aruco/reset_counter

Поток данных:
  камера -> detect -> loc -> tf -> /aruco/world_pose -> px4_local_pose_publisher (VPE)

Все настройки пайплайна собраны здесь (раздел args), поэтому
full_system_real.launch.py остаётся компактным.

Пример:
  ros2 launch launch_system aruco.launch.py \\
      image_topic:=/camera_1/image_raw \\
      camera_info_topic:=/camera_1/camera_info \\
      tf_parent_frame:=camera_optical_1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ================================================================== #
    # Аргументы — единственное место настройки ArUco
    # ================================================================== #
    args = [
        # ---- Интерфейс (прокидывается из full_system_real) -------------
        DeclareLaunchArgument('image_topic',       default_value='/camera_1/image_raw',
            description='Топик сырой картинки для детекции'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera_1/camera_info',
            description='Топик camera_info'),
        DeclareLaunchArgument('tf_parent_frame',   default_value='camera_optical_1',
            description='frame_id камеры (optical). Идёт в parent_frame_id и tf_parent_frame.'),
        DeclareLaunchArgument('world_frame_id',    default_value='map'),
        DeclareLaunchArgument('base_link_frame_id', default_value='base_link'),
        DeclareLaunchArgument('mode',              default_value='debug',
            description="Режим карты: 'debug' (отладочные изображения плоскостей) или 'normal'"),

        # ---- Карта маркеров (aruco_map) --------------------------------
        # 0.25 m is the field kit marker size (0.601 m grid pitch). This is
        # only a fallback: markers.txt sets the size per marker in its second
        # column.
        DeclareLaunchArgument('map_marker_size',   default_value='0.25',
            description='Physical size of MAP markers, m (fallback, see markers.txt)'),

        # ---- Детекция (ArucoDetectNode) --------------------------------
        DeclareLaunchArgument('marker_size',       default_value='0.5',
            description='Размер одиночных (вне карты) маркеров для PnP, м'),
        DeclareLaunchArgument('dictionary_id',     default_value='3'),
        DeclareLaunchArgument('coarse_to_fine_enabled', default_value='true',
            description='Coarse-to-fine: detect on a downscaled frame + subpixel refine. '
                        'On the fisheye camera (equidistant, 1280x960) @0.5 detection is '
                        'BETTER than full resolution: 59 vs 38 frames with markers on the '
                        '2026-07-18 field bag, and ~3x cheaper on CPU (5.5 vs 17.7 ms/frame).'),
        DeclareLaunchArgument('coarse_scale',      default_value='0.5'),
        DeclareLaunchArgument('corner_refine_pnp_only', default_value='true'),
        DeclareLaunchArgument('subpix_win_size',   default_value='5'),
        DeclareLaunchArgument('max_detect_fps',    default_value='20.0',
            description='Max detect/image processing rate in Hz. <=0 disables frame throttling.'),
        DeclareLaunchArgument('estimate_marker_pose',   default_value='false'),
        DeclareLaunchArgument('pnp_non_map_markers',    default_value='false'),

        # ---- Локализация (ArucoLocNode, hybrid PnP) ---------------------
        # auto: 1 marker -> IPPE_SQUARE, >=2 markers -> RANSAC.
        # The camera was recalibrated as fisheye (equidistant, fx≈1118,
        # 1280x960 — a center crop of the 160° lens, effective HFoV≈65°), so
        # the 100/12.0 band-aid is gone. 32/6.0 were picked by a sweep over
        # the 2026-07-18 field bag: 49 poses out of 59 frames with markers at
        # 2.06 px inlier RMSE (strict 16/4.0 gives 46 poses at the same RMSE,
        # 100/12.0 gives 52 but with a worse tail). The final gating
        # max_inlier_rmse_px=6.0 (composed default) stays strict.
        # 6.0 -> 9.0 after the 2026-07-19 flight-bag replay: at ~1 m altitude
        # every cm of map error is ~10 px of reprojection, and with a
        # cm-level-imperfect markers.txt the 6 px threshold caused multi-second
        # streaks of dropped frames (yield 27-47%); 9 px roughly doubles the
        # yield on an imperfect map and costs nothing on a surveyed one
        # (inlier RMSE stays ~2.5 px). Survey the map with
        # aruco_map/scripts/survey_map_from_bag.py to remove the root cause.
        DeclareLaunchArgument('pnp_iterations',            default_value='32'),
        DeclareLaunchArgument('pnp_reprojection_error_px', default_value='9.0'),
        DeclareLaunchArgument('pnp_min_used_markers',      default_value='0'),
        DeclareLaunchArgument('pnp_min_inlier_points',     default_value='0'),

        # Одномаркерный режим с attitude-prior (P1′/P2′/P3′, см.
        # aruco_det_loc/SINGLE_MARKER_JITTER_ANALYSIS.md ч.2): TF на stamp
        # кадра, гейты PnP↔prior, трансляция при известной ориентации,
        # честная ковариация. Убирает rolling-shutter → tilt → ×h механизм
        # (30°/с корпуса → медиана 14 см ошибки у свободного PnP).
        DeclareLaunchArgument('single_prior_enabled', default_value='true'),
        DeclareLaunchArgument('single_gate_tilt_deg', default_value='20.0'),
        DeclareLaunchArgument('single_gate_yaw_deg',  default_value='0.0'),

        # Radial corner-noise weights (IRLS refine + weighted covariance).
        # Refit after each camera recalibration; see aruco_composed.launch.py.
        DeclareLaunchArgument('weights_radial_enabled', default_value='true'),
        DeclareLaunchArgument('weights_sigma0_px',      default_value='1.50'),
        DeclareLaunchArgument('weights_beta',           default_value='2.40'),
        DeclareLaunchArgument('weights_q',              default_value='3.0'),

        # ---- TF / выход для VPE (ArucoTfNode) --------------------------
        DeclareLaunchArgument('world_pose_topic',    default_value='/aruco/world_pose'),
        DeclareLaunchArgument('reset_counter_topic', default_value='/aruco/reset_counter'),
        DeclareLaunchArgument('allow_offset_timeout_reset', default_value='false'),
        DeclareLaunchArgument('offset_timeout',      default_value='3.0'),
        # Бамп reset_counter при возврате позы после дыры >1 c (оффсет мира не
        # трогается) — EKF2 принимает разрыв сбросом, а не рывком скорости.
        DeclareLaunchArgument('reacquisition_reset_gap', default_value='3.0'),
        # Периодическая переотправка оффсета T_world_aruco (Гц); <=0 — по изменению.
        DeclareLaunchArgument('map_offset_publish_rate_hz', default_value='10.0'),

        # ---- Отладка composed-пайплайна --------------------------------
        DeclareLaunchArgument('publish_debug_image',       default_value='true'),
        DeclareLaunchArgument('publish_world_debug_image', default_value='true'),
        DeclareLaunchArgument('publish_visualization',     default_value='false'),

        # ---- Производительность ----------------------------------------
        # 2 потока OpenCV: при coarse 0.5 детекта хватает, а 2 ядра из 4
        # остаются под сторонние процессы (цель — общая загрузка <70%).
        DeclareLaunchArgument('opencv_num_threads', default_value='2'),
        # CPU-аффинити/потоки/coarse — через per-platform YAML (config/cpu_rpi5.yaml
        # или cpu_a733.yaml). Массив cpu_affinity_ids нельзя передать launch-строкой
        # в composable-ноды, поэтому используется cpu_params_file (пусто = дефолты launch).
        DeclareLaunchArgument('cpu_params_file', default_value='',
            description='Per-platform YAML для aruco_composed; пусто = дефолты launch.'),
    ]

    lc = LaunchConfiguration

    # ================================================================== #
    # 1. Aruco map — отдельный процесс (transient_local карта)
    # ================================================================== #
    aruco_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aruco_map'), 'launch', 'aruco_map.launch.py'
            ])
        ]),
        launch_arguments={
            'mode':        lc('mode'),
            'frame_id':    lc('world_frame_id'),   # карта в world-фрейме
            'marker_size': lc('map_marker_size'),
            'topic_name':  '/aruco/map/markers',
        }.items()
    )

    # ================================================================== #
    # 2. Aruco composed — detect + loc + tf в одном контейнере
    # ================================================================== #
    aruco_composed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aruco_det_loc'), 'launch', 'aruco_composed.launch.py'
            ])
        ]),
        launch_arguments={
            # Топики
            'image_topic':         lc('image_topic'),
            'camera_info_topic':   lc('camera_info_topic'),
            'map_markers_topic':   '/aruco/map/markers',
            'markers_topic':       '/aruco/det/markers',
            'pose_cov_topic':      '/aruco/loc/pose_cov',
            'world_pose_topic':    lc('world_pose_topic'),
            'reset_counter_topic': lc('reset_counter_topic'),

            # Фреймы (камера-optical идёт и как parent_frame_id, и как tf_parent_frame)
            'parent_frame_id':     lc('tf_parent_frame'),
            'tf_parent_frame':     lc('tf_parent_frame'),
            'world_frame_id':      lc('world_frame_id'),
            'base_link_frame_id':  lc('base_link_frame_id'),

            # Детекция
            'marker_size':            lc('marker_size'),
            'dictionary_id':          lc('dictionary_id'),
            'coarse_to_fine_enabled': lc('coarse_to_fine_enabled'),
            'coarse_scale':           lc('coarse_scale'),
            'corner_refine_pnp_only': lc('corner_refine_pnp_only'),
            'subpix_win_size':        lc('subpix_win_size'),
            'max_detect_fps':         lc('max_detect_fps'),
            'estimate_marker_pose':   lc('estimate_marker_pose'),
            'pnp_non_map_markers':    lc('pnp_non_map_markers'),

            # Локализация
            'pnp_iterations':            lc('pnp_iterations'),
            'pnp_reprojection_error_px': lc('pnp_reprojection_error_px'),
            'pnp_min_used_markers':      lc('pnp_min_used_markers'),
            'pnp_min_inlier_points':     lc('pnp_min_inlier_points'),
            'single_prior_enabled':      lc('single_prior_enabled'),
            'single_gate_tilt_deg':      lc('single_gate_tilt_deg'),
            'single_gate_yaw_deg':       lc('single_gate_yaw_deg'),
            'weights_radial_enabled':    lc('weights_radial_enabled'),
            'weights_sigma0_px':         lc('weights_sigma0_px'),
            'weights_beta':              lc('weights_beta'),
            'weights_q':                 lc('weights_q'),

            # TF offset
            'allow_offset_timeout_reset': lc('allow_offset_timeout_reset'),
            'offset_timeout':             lc('offset_timeout'),
            'reacquisition_reset_gap':    lc('reacquisition_reset_gap'),
            'map_offset_publish_rate_hz': lc('map_offset_publish_rate_hz'),

            # Отладка
            'publish_debug_image':       lc('publish_debug_image'),
            'publish_world_debug_image': lc('publish_world_debug_image'),
            'publish_visualization':     lc('publish_visualization'),

            # Производительность
            'opencv_num_threads': lc('opencv_num_threads'),
            'cpu_params_file':    lc('cpu_params_file'),
        }.items()
    )

    return LaunchDescription(args + [aruco_map, aruco_composed])
