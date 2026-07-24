"""
aruco_composed.launch.py
------------------------
Runs ArucoDetectNode, ArucoLocNode, and ArucoTfNode in a single OS process via
rclcpp_components ComposableNodeContainer.

IPC (use_intra_process_comms) is intentionally DISABLED for ArucoDetectNode and
ArucoLocNode because both nodes subscribe the map-markers topic with
transient_local durability, which is incompatible with IPC zero-copy
(ROS2 requirement: IPC requires volatile durability on every pub/sub in the node).

ArucoTfNode ALSO runs with IPC disabled: it publishes /aruco/map_offset with
transient_local (latched) durability, which is incompatible with IPC zero-copy.
It had no intra-process partner anyway (aruco_loc has IPC off), so this is free.

All three nodes share the same OS process → reduced scheduling overhead and
shared L2/L3 cache even without zero-copy.

Limitations:
  - aruco_map_node uses transient_local QoS and must remain a separate process.
  - Camera image transport subscriptions use a separate DDS thread; IPC does
    not apply to the image topic coming from outside this container.

CPU affinity / threading / coarse-to-fine are platform-specific. They are array
and startup-only parameters that cannot be passed reliably as launch-arg
strings, so pass them through a per-platform YAML via `cpu_params_file:`:

  # Raspberry Pi 5 (4x A76, homogeneous — no affinity needed)
  ros2 launch aruco_det_loc aruco_composed.launch.py \\
    cpu_params_file:=$(ros2 pkg prefix aruco_det_loc)/share/aruco_det_loc/config/cpu_rpi5.yaml

  # Orange Pi Zero 3W / Allwinner A733 (2x A76 + 6x A55 — pin to A76)
  ros2 launch aruco_det_loc aruco_composed.launch.py \\
    cpu_params_file:=$(ros2 pkg prefix aruco_det_loc)/share/aruco_det_loc/config/cpu_a733.yaml

Override any other argument, e.g.:
  ros2 launch aruco_det_loc aruco_composed.launch.py \\
    image_topic:=/camera_1/image_raw world_frame_id:=map max_detect_fps:=20.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


# ---------------------------------------------------------------------------
# Launch arguments
# ---------------------------------------------------------------------------
ARGS = [
    # Topics — defaults point at the real on-board camera (override for sim).
    DeclareLaunchArgument('image_topic',         default_value='/camera_1/image_raw'),
    DeclareLaunchArgument('camera_info_topic',   default_value='/camera_1/camera_info'),
    DeclareLaunchArgument('map_markers_topic',   default_value='/aruco/map/markers'),
    DeclareLaunchArgument('markers_topic',       default_value='/aruco/det/markers'),
    DeclareLaunchArgument('pose_cov_topic',      default_value='/aruco/loc/pose_cov'),
    DeclareLaunchArgument('world_pose_topic',    default_value='/aruco/world_pose'),
    DeclareLaunchArgument('reset_counter_topic', default_value='/aruco/reset_counter'),
    DeclareLaunchArgument('map_offset_topic',    default_value='/aruco/map_offset'),

    # Frames — MUST match the external static TF base_link->camera_optical_1.
    DeclareLaunchArgument('parent_frame_id',         default_value='camera_optical_1'),
    DeclareLaunchArgument('world_frame_id',          default_value='map'),
    DeclareLaunchArgument('base_link_frame_id',      default_value='base_link'),
    DeclareLaunchArgument('camera_frame_id',         default_value=''),
    DeclareLaunchArgument('tf_parent_frame',         default_value='camera_optical_1'),
    DeclareLaunchArgument('tf_child_frame',          default_value='aruco_map_detected'),
    DeclareLaunchArgument('tf_map_frame',            default_value='aruco_map'),
    DeclareLaunchArgument('aruco_map_frame_id',      default_value='aruco_map'),
    DeclareLaunchArgument('aruco_detected_frame_id', default_value='aruco_map_detected'),
    DeclareLaunchArgument('base_aruco_frame_id',     default_value='base_link_aruco'),

    # Detection
    DeclareLaunchArgument('marker_size',          default_value='0.5'),
    DeclareLaunchArgument('dictionary_id',        default_value='3'),
    DeclareLaunchArgument('use_aruco3_detection', default_value='false'),
    DeclareLaunchArgument('aruco3_min_marker_length_ratio', default_value='0.10'),
    DeclareLaunchArgument('marker_id_whitelist',  default_value='[]',
        description='JSON array of allowed marker IDs, e.g. [0,1,2]. Empty = accept all.'),
    DeclareLaunchArgument('max_detect_fps',       default_value='0.0',
        description='Max detect/image processing rate in Hz. <=0 disables frame throttling.'),

    # Per-marker solvePnP (individual markers outside map)
    DeclareLaunchArgument('estimate_marker_pose', default_value='false'),
    DeclareLaunchArgument('pnp_non_map_markers',  default_value='false'),
    DeclareLaunchArgument('pnp_fps_cap_enabled',  default_value='true'),
    DeclareLaunchArgument('pnp_fps_cap',          default_value='10'),

    # Localisation PnP (aruco_loc): >=2 markers use RANSAC; one marker uses
    # IPPE_SQUARE without RANSAC. Keep known_vertical_frame/auto_flip enabled
    # for reliable single-marker operation on a planar map.
    # 4.0 -> 9.0: cm-level map errors project to ~10 px at 1 m altitude; a
    # tight threshold turns them into multi-second dropped-frame streaks
    # (see 2026-07-19 bag replay). 9 px is safe: with a surveyed map the
    # inlier RMSE stays ~2.5 px and gating_max_inlier_rmse_px still applies.
    DeclareLaunchArgument('pnp_iterations',             default_value='16'),
    DeclareLaunchArgument('pnp_reprojection_error_px',  default_value='9.0'),
    DeclareLaunchArgument('pnp_flags',                  default_value='0',
        description='solvePnP algorithm: 0=ITERATIVE, 8=SQPNP. SQPNP is often '
                    'more robust on a planar map — profile both.'),
    DeclareLaunchArgument('pnp_min_used_markers',       default_value='0',
        description='<=0: auto hybrid PnP; >0: legacy hard minimum marker count.'),
    DeclareLaunchArgument('pnp_min_inlier_points',      default_value='0',
        description='<=0: auto inlier minimum; >0: legacy hard minimum inlier count.'),
    DeclareLaunchArgument('gating_max_inlier_rmse_px',  default_value='6.0',
        description='Reject pose if inlier reprojection RMSE exceeds this (px).'),

    # Radial corner-noise model σ(r) = sigma0·(1+beta·r^q) — IRLS refine +
    # weighted covariance instead of dropping edge markers (see
    # aruco_loc_utils.hpp). Defaults fitted to the 2026-07-19 field-bag
    # replay; refit against aruco_map/scripts/survey_map_from_bag.py
    # residuals after each camera recalibration and override here.
    DeclareLaunchArgument('weights_radial_enabled', default_value='true'),
    DeclareLaunchArgument('weights_sigma0_px',      default_value='1.50'),
    DeclareLaunchArgument('weights_beta',           default_value='2.40'),
    DeclareLaunchArgument('weights_q',              default_value='3.0'),

    # Vertical / flip — recommended ON for a planar (z=0) map to suppress the
    # PnP flip ambiguity. Requires a valid world->base_link TF (IMU/EKF attitude).
    DeclareLaunchArgument('known_vertical_frame', default_value='base_link'),
    DeclareLaunchArgument('auto_flip',            default_value='true'),

    # Одномаркерный режим с attitude-prior (P1′/P2′/P3′, см.
    # SINGLE_MARKER_JITTER_ANALYSIS.md ч.2): TF на stamp кадра, гейты
    # PnP↔prior, трансляция при известной ориентации, честная ковариация.
    DeclareLaunchArgument('single_prior_enabled', default_value='true',
        description='Single-marker: t-only solve with stamp-aligned EKF attitude prior.'),
    DeclareLaunchArgument('single_gate_tilt_deg', default_value='20.0',
        description='Reject single-marker pose when PnP tilt deviates from prior by more (deg); <=0 off.'),
    DeclareLaunchArgument('single_gate_yaw_deg',  default_value='0.0',
        description='Reject single-marker pose when ArUco yaw deviates from prior by more (deg); <=0 off.'),

    # TF offset
    DeclareLaunchArgument('allow_offset_timeout_reset', default_value='false'),
    DeclareLaunchArgument('initialize_at_origin',       default_value='false',
        description='Use the first qualified ArUco pose as the local-frame origin.'),
    DeclareLaunchArgument('offset_timeout',             default_value='3.0'),
    DeclareLaunchArgument('offset_init_samples',        default_value='15'),
    DeclareLaunchArgument('offset_candidate_translation_tolerance', default_value='0.10'),
    DeclareLaunchArgument('offset_candidate_rotation_tolerance',    default_value='0.10'),
    DeclareLaunchArgument('covariance_xy_floor',        default_value='0.0004'),
    DeclareLaunchArgument('covariance_z_floor',         default_value='0.0009'),
    DeclareLaunchArgument('covariance_angle_floor',     default_value='0.0027415568'),
    DeclareLaunchArgument('reacquisition_reset_gap',    default_value='3.0',
        description='Bump reset_counter when the pose stream resumes after a '
                    'gap longer than this (s); EKF2 then accepts the position '
                    'discontinuity as a reset instead of a velocity spike. '
                    'The world offset is NOT recomputed. <=0 disables.'),
    DeclareLaunchArgument('map_offset_publish_rate_hz', default_value='10.0',
        description='Re-publish the T_world_aruco offset at this rate (Hz) on '
                    'top of the latched on-change publication. <=0 = on-change '
                    'only.'),

    # Swarm mode: publish base_link directly in the shared aruco_map frame
    # (offset = identity for every drone) → absolute, fleet-comparable coords.
    # false = legacy per-drone world anchoring (pose in `world_frame_id`).
    DeclareLaunchArgument('swarm_mode', default_value='false',
        description='true: /aruco/world_pose is base_link in the common aruco_map '
                    'frame, identical across all drones. Set at launch, not in flight.'),

    # Coarse-to-fine corner refinement (aruco_detect).
    # coarse_to_fine_enabled / coarse_scale / subpix_win_size / opencv_num_threads
    # are best set per-platform in cpu_params_file (see config/cpu_*.yaml).
    DeclareLaunchArgument('coarse_to_fine_enabled', default_value='false'),
    DeclareLaunchArgument('coarse_scale',           default_value='0.25'),
    DeclareLaunchArgument('corner_refine_pnp_only', default_value='true'),
    DeclareLaunchArgument('subpix_win_size',        default_value='6'),
    DeclareLaunchArgument('opencv_num_threads',     default_value='2'),

    # Debug output
    DeclareLaunchArgument('publish_debug_image',       default_value='false',
        description='Publish image with detected markers drawn (topic: /aruco/vis/debug_image)'),
    DeclareLaunchArgument('publish_world_debug_image', default_value='false',
        description='Publish image with aruco_map axes projected (topic: /aruco/vis/world_debug_image)'),
    DeclareLaunchArgument('publish_visualization',     default_value='false',
        description='Publish RViz MarkerArray visualization'),

    # Per-platform startup params (CPU affinity, threads, coarse-to-fine).
    DeclareLaunchArgument('cpu_params_file', default_value='',
        description='Path to a per-platform YAML (config/cpu_rpi5.yaml or '
                    'config/cpu_a733.yaml). Empty = use launch-arg defaults.'),
]


def _make_container(context, *_args, **_kwargs):
    cpu_params_file = LaunchConfiguration('cpu_params_file').perform(context).strip()

    # Per-key parameters wired from launch arguments.
    detect_params = {
        'image_topic':       LaunchConfiguration('image_topic'),
        'camera_info_topic': LaunchConfiguration('camera_info_topic'),
        'map_markers_topic': LaunchConfiguration('map_markers_topic'),
        'parent_frame_id':   LaunchConfiguration('parent_frame_id'),
        'marker_size':       ParameterValue(LaunchConfiguration('marker_size'), value_type=float),
        'dictionary_id':     ParameterValue(LaunchConfiguration('dictionary_id'), value_type=int),
        'use_aruco3_detection':
            ParameterValue(LaunchConfiguration('use_aruco3_detection'), value_type=bool),
        'aruco3_min_marker_length_ratio':
            ParameterValue(LaunchConfiguration('aruco3_min_marker_length_ratio'), value_type=float),
        'max_detect_fps':
            ParameterValue(LaunchConfiguration('max_detect_fps'), value_type=float),
        'estimate_marker_pose':
            ParameterValue(LaunchConfiguration('estimate_marker_pose'), value_type=bool),
        'pnp_non_map_markers':
            ParameterValue(LaunchConfiguration('pnp_non_map_markers'), value_type=bool),
        'pnp_fps_cap_enabled':
            ParameterValue(LaunchConfiguration('pnp_fps_cap_enabled'), value_type=bool),
        'pnp_fps_cap':
            ParameterValue(LaunchConfiguration('pnp_fps_cap'), value_type=int),
        'use_map_markers':           True,
        'detect_individual_markers': True,
        'publish_debug_image':
            ParameterValue(LaunchConfiguration('publish_debug_image'), value_type=bool),
        'publish_visualization':
            ParameterValue(LaunchConfiguration('publish_visualization'), value_type=bool),
        'publish_world_debug_image':
            ParameterValue(LaunchConfiguration('publish_world_debug_image'), value_type=bool),
        'coarse_to_fine_enabled':
            ParameterValue(LaunchConfiguration('coarse_to_fine_enabled'), value_type=bool),
        'coarse_scale':
            ParameterValue(LaunchConfiguration('coarse_scale'), value_type=float),
        'corner_refine_pnp_only':
            ParameterValue(LaunchConfiguration('corner_refine_pnp_only'), value_type=bool),
        'subpix_win_size':
            ParameterValue(LaunchConfiguration('subpix_win_size'), value_type=int),
        'opencv_num_threads':
            ParameterValue(LaunchConfiguration('opencv_num_threads'), value_type=int),
    }

    loc_params = {
        'markers_topic':      LaunchConfiguration('markers_topic'),
        'map_markers_topic':  LaunchConfiguration('map_markers_topic'),
        'camera_info_topic':  LaunchConfiguration('camera_info_topic'),
        'pose_topic':         LaunchConfiguration('pose_cov_topic'),
        'world_frame_id':     LaunchConfiguration('world_frame_id'),
        'base_link_frame_id': LaunchConfiguration('base_link_frame_id'),
        'camera_frame_id':    LaunchConfiguration('camera_frame_id'),
        'tf_parent_frame':    LaunchConfiguration('tf_parent_frame'),
        'tf_child_frame':     LaunchConfiguration('tf_child_frame'),
        'tf_map_frame':       LaunchConfiguration('tf_map_frame'),
        # aruco_tf owns the world-frame TFs (incl. aruco_map_detected). Disable
        # loc's dynamic camera->aruco_map_detected broadcast to avoid a two-parent
        # TF collision on aruco_map_detected. Static aruco_<id> frames stay on
        # (publish_static_markers_tf is separate).
        'publish_tf':         False,
        'known_vertical_frame': LaunchConfiguration('known_vertical_frame'),
        'auto_flip':
            ParameterValue(LaunchConfiguration('auto_flip'), value_type=bool),
        'single.prior_enabled':
            ParameterValue(LaunchConfiguration('single_prior_enabled'), value_type=bool),
        'single.gate_tilt_deg':
            ParameterValue(LaunchConfiguration('single_gate_tilt_deg'), value_type=float),
        'single.gate_yaw_deg':
            ParameterValue(LaunchConfiguration('single_gate_yaw_deg'), value_type=float),
        'pnp.iterations':
            ParameterValue(LaunchConfiguration('pnp_iterations'), value_type=int),
        'pnp.reprojection_error_px':
            ParameterValue(LaunchConfiguration('pnp_reprojection_error_px'), value_type=float),
        'pnp.flags':
            ParameterValue(LaunchConfiguration('pnp_flags'), value_type=int),
        'pnp.min_used_markers':
            ParameterValue(LaunchConfiguration('pnp_min_used_markers'), value_type=int),
        'pnp.min_inlier_points':
            ParameterValue(LaunchConfiguration('pnp_min_inlier_points'), value_type=int),
        'gating.max_inlier_rmse_px':
            ParameterValue(LaunchConfiguration('gating_max_inlier_rmse_px'), value_type=float),
        'weights.radial_enabled':
            ParameterValue(LaunchConfiguration('weights_radial_enabled'), value_type=bool),
        'weights.sigma0_px':
            ParameterValue(LaunchConfiguration('weights_sigma0_px'), value_type=float),
        'weights.beta':
            ParameterValue(LaunchConfiguration('weights_beta'), value_type=float),
        'weights.q':
            ParameterValue(LaunchConfiguration('weights_q'), value_type=float),
    }

    # Append the per-platform YAML last so it overrides the per-key values above
    # (this is the only reliable way to feed array/startup params such as
    # cpu_affinity_ids to a *composable* node).
    detect_param_list = [detect_params]
    loc_param_list = [loc_params]
    if cpu_params_file:
        detect_param_list.append(cpu_params_file)
        loc_param_list.append(cpu_params_file)

    container = ComposableNodeContainer(
        name='aruco_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='aruco_det_loc',
                plugin='aruco_det_loc::ArucoDetectNode',
                name='aruco_detect',
                parameters=detect_param_list,
                extra_arguments=[{'use_intra_process_comms': False}],
            ),
            ComposableNode(
                package='aruco_det_loc',
                plugin='aruco_det_loc::ArucoLocNode',
                name='aruco_loc',
                parameters=loc_param_list,
                extra_arguments=[{'use_intra_process_comms': False}],
            ),
            ComposableNode(
                package='aruco_det_loc',
                plugin='aruco_det_loc::ArucoTfNode',
                name='aruco_tf',
                parameters=[{
                    'pose_cov_topic':          LaunchConfiguration('pose_cov_topic'),
                    'world_pose_topic':        LaunchConfiguration('world_pose_topic'),
                    'reset_counter_topic':     LaunchConfiguration('reset_counter_topic'),
                    'map_offset_topic':        LaunchConfiguration('map_offset_topic'),
                    'camera_frame_id':         LaunchConfiguration('parent_frame_id'),
                    'base_link_frame_id':      LaunchConfiguration('base_link_frame_id'),
                    'world_frame_id':          LaunchConfiguration('world_frame_id'),
                    'aruco_map_frame_id':      LaunchConfiguration('aruco_map_frame_id'),
                    'aruco_detected_frame_id': LaunchConfiguration('aruco_detected_frame_id'),
                    'base_aruco_frame_id':     LaunchConfiguration('base_aruco_frame_id'),
                    'allow_offset_timeout_reset':
                        ParameterValue(LaunchConfiguration('allow_offset_timeout_reset'), value_type=bool),
                    'initialize_at_origin':
                        ParameterValue(LaunchConfiguration('initialize_at_origin'), value_type=bool),
                    'offset_timeout':
                        ParameterValue(LaunchConfiguration('offset_timeout'), value_type=float),
                    'offset_init_samples':
                        ParameterValue(LaunchConfiguration('offset_init_samples'), value_type=int),
                    'offset_candidate_translation_tolerance':
                        ParameterValue(LaunchConfiguration('offset_candidate_translation_tolerance'), value_type=float),
                    'offset_candidate_rotation_tolerance':
                        ParameterValue(LaunchConfiguration('offset_candidate_rotation_tolerance'), value_type=float),
                    'covariance_xy_floor':
                        ParameterValue(LaunchConfiguration('covariance_xy_floor'), value_type=float),
                    'covariance_z_floor':
                        ParameterValue(LaunchConfiguration('covariance_z_floor'), value_type=float),
                    'covariance_angle_floor':
                        ParameterValue(LaunchConfiguration('covariance_angle_floor'), value_type=float),
                    'reacquisition_reset_gap':
                        ParameterValue(LaunchConfiguration('reacquisition_reset_gap'), value_type=float),
                    'map_offset_publish_rate_hz':
                        ParameterValue(LaunchConfiguration('map_offset_publish_rate_hz'), value_type=float),
                    'swarm_mode':
                        ParameterValue(LaunchConfiguration('swarm_mode'), value_type=bool),
                }],
                # IPC must stay OFF: /aruco/map_offset is transient_local (latched),
                # which rclcpp forbids under intra-process comms. No IPC partner
                # exists anyway (aruco_loc runs with IPC disabled).
                extra_arguments=[{'use_intra_process_comms': False}],
            ),
        ],
        output='screen',
        # nice 5: at 100% CPU detection yields cores to the flight path (XRCE
        # agent, offboard_control) — detection FPS degrades, not the PX4 link.
        # Not 10: this container also publishes /aruco/world_pose for VPE, so
        # full starvation would drop the vision position in the EKF.
        prefix='nice -n 5',
    )

    return [container]


def generate_launch_description():
    return LaunchDescription(ARGS + [OpaqueFunction(function=_make_container)])
