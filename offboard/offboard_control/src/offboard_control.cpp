#include "offboard_control/offboard_control.hpp"

namespace {
// TRAJECTORY group validation/emission helpers. A group (position, velocity or
// acceleration) is "on" only when all three components are finite; "off" only
// when all three are NaN. any_finite_3 is used to decide which OffboardControlMode
// flags to raise so PX4 does not reject a mixed setpoint.
bool all_finite_3(const std::array<double, 3>& values) {
    return std::all_of(values.begin(), values.end(),
                       [](double value) { return std::isfinite(value); });
}

bool all_nan_3(const std::array<double, 3>& values) {
    return std::all_of(values.begin(), values.end(),
                       [](double value) { return std::isnan(value); });
}

bool any_finite_3(const std::array<double, 3>& values) {
    return std::any_of(values.begin(), values.end(),
                       [](double value) { return std::isfinite(value); });
}

// ENU vector -> NED array (x=E,y=N,z=U  ->  x=N,y=E,z=-U), same convention as the
// VELOCITY branch.
void set_ned_from_enu_vector(const geometry_msgs::msg::Vector3& enu, std::array<float, 3>& ned) {
    ned[0] = static_cast<float>(enu.y);
    ned[1] = static_cast<float>(enu.x);
    ned[2] = static_cast<float>(-enu.z);
}
}  // namespace


OffboardControlNode::OffboardControlNode(std::string px4_namespace)
: Node("offboard_control"),
  tf_buffer_(this->get_clock()),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_)),
  tf_broadcaster_(this),
  static_tf_broadcaster_(this)
{
    // --- Parameters ---
    this->declare_parameter<std::string>("body_frame_id", "base_link");
    this->declare_parameter<std::string>("map_frame_id", "map");
    this->declare_parameter<std::string>("aruco_map_frame_id", "aruco_map");
    this->declare_parameter<double>("transform_timeout", 0.5);
    this->declare_parameter<double>("offboard_timeout", 10.0);
    this->declare_parameter<double>("arming_timeout", 5.0);
    this->declare_parameter<double>("default_speed", 0.5);
    this->declare_parameter<bool>("land_only_in_offboard", true);
    this->declare_parameter<bool>("check_kill_switch", true);
    this->declare_parameter<double>("local_position_timeout", 2.0);
    this->declare_parameter<double>("global_position_timeout", 10.0);
    this->declare_parameter<double>("battery_timeout", 2.0);
    this->declare_parameter<double>("manual_control_timeout", 0.0);
    this->declare_parameter<bool>("simulator", false);
    this->declare_parameter<bool>("enforce_rc_in_sim", false);
    this->declare_parameter<double>("rc_stick_abort_thresh", 0.05);
    // 0 = topic off (default keeps prior behavior); adjustable at runtime via
    // `ros2 param set ... telemetry_rate_hz 50.0`.
    this->declare_parameter<double>("telemetry_rate_hz", 0.0);
    this->declare_parameter<bool>("enforce_freshness", true);
    // Max age of a ~/trajectory_correction message still applied as feed-forward;
    // older stream is dropped and the drone holds the base TRAJECTORY target.
    // <=0 disables the correction stream entirely.
    this->declare_parameter<double>("correction_timeout", 0.5);
    // Geofence / limits (prompt 99) — off by default, behavior unchanged until enabled.
    this->declare_parameter<bool>("geofence_enable", false);
    this->declare_parameter<double>("geofence_radius", 10.0);
    this->declare_parameter<double>("geofence_center_x", 0.0);
    this->declare_parameter<double>("geofence_center_y", 0.0);
    this->declare_parameter<double>("alt_floor", 0.0);
    this->declare_parameter<double>("alt_ceil", 50.0);
    this->declare_parameter<double>("max_speed", 0.0);
    // Failsafe policy (prompt 104): hold | land | rtl for node-side critical
    // rollbacks; land_from_hold additively relaxes the land gate.
    this->declare_parameter<std::string>("onboard_failsafe_action", "hold");
    this->declare_parameter<bool>("land_from_hold", false);

    // --- Excitation (system-ID) parameters — conservative safety defaults ---
    this->declare_parameter<double>("excite_stream_hz", 200.0);
    this->declare_parameter<double>("excite_settle_in_s", 1.5);
    this->declare_parameter<double>("excite_settle_out_s", 1.5);
    this->declare_parameter<double>("excite_max_inject_s", 20.0);
    this->declare_parameter<double>("excite_max_session_s", 90.0);
    this->declare_parameter<bool>("allow_rate_loop_excitation", false);
    this->declare_parameter<double>("excite_geofence_radius", 2.0);
    this->declare_parameter<double>("excite_agl_floor", 1.0);
    this->declare_parameter<double>("excite_agl_ceil", 4.0);
    this->declare_parameter<double>("excite_kp_pos", 0.04);
    this->declare_parameter<double>("excite_kp_alt", 0.05);
    this->declare_parameter<double>("excite_kd_vel", 0.02);
    this->declare_parameter<double>("excite_max_baseline_tilt", 0.087);
    this->declare_parameter<double>("excite_hover_thrust", 0.5);
    this->declare_parameter<double>("excite_stick_abort_thresh", 0.05);
    this->declare_parameter<double>("excite_settle_rate_thresh", 0.30);
    this->declare_parameter<double>("excite_settle_vel_thresh", 0.30);

    this->get_parameter("body_frame_id", body_frame_id_);
    this->get_parameter("map_frame_id", map_frame_id_);
    this->get_parameter("aruco_map_frame_id", aruco_map_frame_id_);
    this->get_parameter("transform_timeout", transform_timeout_);
    this->get_parameter("offboard_timeout", offboard_timeout_);
    this->get_parameter("arming_timeout", arming_timeout_);
    this->get_parameter("default_speed", default_speed_);
    this->get_parameter("land_only_in_offboard", land_only_in_offboard_);
    this->get_parameter("check_kill_switch", check_kill_switch_);
    this->get_parameter("local_position_timeout", local_position_timeout_);
    this->get_parameter("global_position_timeout", global_position_timeout_);
    this->get_parameter("battery_timeout", battery_timeout_);
    this->get_parameter("manual_control_timeout", manual_control_timeout_);
    this->get_parameter("simulator", is_simulator_);
    this->get_parameter("enforce_rc_in_sim", enforce_rc_in_sim_);
    this->get_parameter("rc_stick_abort_thresh", rc_stick_abort_thresh_);
    this->get_parameter("enforce_freshness", enforce_freshness_);
    this->get_parameter("correction_timeout", correction_timeout_);
    // Declared here (not in the block above) because the default depends on the
    // already-read simulator flag: sim zeroes the displayed yaw at the first
    // odometry sample, the real drone does not — the historical behavior,
    // now explicit and overridable on both (prompt 103).
    this->declare_parameter<bool>("zero_heading_at_start", is_simulator_);
    this->get_parameter("zero_heading_at_start", zero_heading_at_start_);
    this->get_parameter("geofence_enable", geofence_enable_);
    this->get_parameter("geofence_radius", geofence_radius_);
    this->get_parameter("geofence_center_x", geofence_center_x_);
    this->get_parameter("geofence_center_y", geofence_center_y_);
    this->get_parameter("alt_floor", alt_floor_);
    this->get_parameter("alt_ceil", alt_ceil_);
    this->get_parameter("max_speed", max_speed_);
    this->get_parameter("onboard_failsafe_action", onboard_failsafe_action_);
    this->get_parameter("land_from_hold", land_from_hold_);
    if (onboard_failsafe_action_ != "hold" && onboard_failsafe_action_ != "land" &&
        onboard_failsafe_action_ != "rtl"  && onboard_failsafe_action_ != "position") {
        RCLCPP_WARN(this->get_logger(),
                    "onboard_failsafe_action '%s' is not hold|land|rtl|position — falling back to 'hold'",
                    onboard_failsafe_action_.c_str());
        onboard_failsafe_action_ = "hold";
    }

    this->get_parameter("excite_stream_hz", excite_stream_hz_);
    this->get_parameter("excite_settle_in_s", excite_settle_in_s_);
    this->get_parameter("excite_settle_out_s", excite_settle_out_s_);
    this->get_parameter("excite_max_inject_s", excite_max_inject_s_);
    this->get_parameter("excite_max_session_s", excite_max_session_s_);
    this->get_parameter("allow_rate_loop_excitation", allow_rate_loop_excitation_);
    this->get_parameter("excite_geofence_radius", excite_geofence_radius_default_);
    this->get_parameter("excite_agl_floor", excite_agl_floor_);
    this->get_parameter("excite_agl_ceil", excite_agl_ceil_);
    this->get_parameter("excite_kp_pos", excite_kp_pos_);
    this->get_parameter("excite_kp_alt", excite_kp_alt_);
    this->get_parameter("excite_kd_vel", excite_kd_vel_);
    this->get_parameter("excite_max_baseline_tilt", excite_max_baseline_tilt_);
    this->get_parameter("excite_hover_thrust", excite_hover_thrust_);
    this->get_parameter("excite_stick_abort_thresh", excite_stick_abort_thresh_);
    this->get_parameter("excite_settle_rate_thresh", excite_settle_rate_thresh_);
    this->get_parameter("excite_settle_vel_thresh", excite_settle_vel_thresh_);

    // Load and parse reference_frames
    create_reference_frames();
    
    // --- State initialization ---
    current_local_position_.heading = 0.0;
    current_local_position_.x = 0.0;
    current_local_position_.y = 0.0;
    current_local_position_.z = 0.0;
    current_local_position_.vx = 0.0;
    current_local_position_.vy = 0.0;
    current_local_position_.vz = 0.0;
    current_local_position_.timestamp = 0.0;

    last_sended_pose = px4_msgs::msg::TrajectorySetpoint();
    service_done_ = false;
    control_rate_ = 30.0; // Hz
    lat_ = 0.0;
    lon_ = 0.0;
    alt_ = 0.0;
    battery_voltage = 0.0;
    battery_percentage  = 0.0;
    current_manual_control_timestamp_ = 0;
    current_battery_timestamp_ = 0;
    current_global_position_timestamp_ = 0;
    current_vehicle_status_timestamp_ = 0;
    
    // Create callback group for service to run in separate thread
    service_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    
    // Initialize publishers, subscribers, services, and timer
    Init_publishers(px4_namespace);
    Init_subscribers(px4_namespace);
    Init_services();
    Init_actions();
    Init_timer();

    // Telemetry streaming (prompt 96): own group so the stream never delays the
    // setpoint timer; rate is 0 (off) by default and changeable at runtime.
    telemetry_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    telemetry_publisher_ = create_publisher<offboard_interfaces::msg::Telemetry>(
        "~/telemetry", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    double telemetry_rate = 0.0;
    this->get_parameter("telemetry_rate_hz", telemetry_rate);
    apply_telemetry_rate(telemetry_rate);
    param_cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&OffboardControlNode::on_parameters_set, this, std::placeholders::_1));

    // Diagnostics topic (prompt 101): low-rate, own group, no control impact.
    status_publisher_ = create_publisher<offboard_interfaces::msg::OffboardStatus>(
        "~/status", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort());
    status_timer_ = create_wall_timer(
        500ms, std::bind(&OffboardControlNode::publish_status, this),
        telemetry_callback_group_);
    
    // Initialize TF frames
    init_tf_frames();
    
    // Publish static TF transforms
    publish_static_transforms();
    
    RCLCPP_INFO(this->get_logger(), "Offboard Control Node initialized with frame_id: %s", map_frame_id_.c_str());
}

OffboardControlNode::~OffboardControlNode()
{
    // Ask any running excitation to stop and join its worker thread.
    request_excite_abort("shutdown", true);
    excite_finished_ = true;
    if (excite_thread_.joinable()) {
        excite_thread_.join();
    }
    telemetry_stream_stop_ = true;
    if (telemetry_stream_thread_.joinable()) {
        telemetry_stream_thread_.join();
    }
    nav_action_stop_ = true;
    if (nav_action_thread_.joinable()) {
        nav_action_thread_.join();
    }
    stopOffboardHeartbeat();
}

// --- Helpers ---
void OffboardControlNode::Init_SpeedController(
    const Pose& start_pose_ned, 
    const Pose& target_pose_any_frame,
    const std::string& frame_id,
    double speed) 
{
    // RAII buffer
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        global_speed_controller_.reset();
    }
    rclcpp::Logger logger = this->get_logger();
    try {
        // Resolve reference frame for target frame
        std::string reference_frame = get_reference_frame(frame_id);
        rclcpp::Time stamp = get_transform_lookup_time(frame_id, reference_frame);
        if (frame_id == aruco_map_frame_id_) {
            RCLCPP_INFO(logger, "Navigate in aruco_map frame (tf_static): using TF time 0 for lookup");
        }
        auto start_timestamp_micros = getTimestamp();

        // Check transform availability
        if (frame_id == aruco_map_frame_id_) {
            RCLCPP_INFO(logger, "Navigate in aruco_map frame: checking TF %s -> %s (need map->aruco_map from px4_local_pose_publisher)",
                        frame_id.c_str(), reference_frame.c_str());
        }
        if (!wait_for_transform(reference_frame, frame_id, stamp, 
                              rclcpp::Duration::from_seconds(transform_timeout_)) ||
            !wait_for_transform(map_frame_id_, reference_frame, stamp, 
                              rclcpp::Duration::from_seconds(transform_timeout_))) {
            if (frame_id == aruco_map_frame_id_) {
                RCLCPP_ERROR(logger, "Transform map <-> aruco_map not available. Check: 1) ArUco markers visible to camera, 2) px4_local_pose_publisher running and offset computed (see its logs for 'Offset computed').");
            }
            throw std::runtime_error("Transform not available between frames");
        }
        
        std::optional<builtin_interfaces::msg::Time> tf_lookup_override;
        if (frame_id == aruco_map_frame_id_) {
            builtin_interfaces::msg::Time t;
            t.sec = 0;
            t.nanosec = 0;
            tf_lookup_override = t;
        }
        // Built locally (constructor does TF work) and swapped in under cmd_mutex_
        // only on success — the timer never sees a half-initialized controller.
        auto controller = std::make_shared<SpeedController>(
            logger, start_pose_ned, target_pose_any_frame, frame_id, map_frame_id_, speed,
            start_timestamp_micros, tf_buffer_, transform_timeout_, tf_lookup_override);

        if (controller->hasTransformationError()) {
            RCLCPP_ERROR(logger, "Initialization failed due to TF error.");
            // Publish target frame on error too (debug)
            publish_target_frame(target_pose_any_frame, frame_id, stamp);
        } else {
            {
                std::lock_guard<std::mutex> lk(cmd_mutex_);
                global_speed_controller_ = controller;
            }
            RCLCPP_INFO(logger, "Global SpeedController initialized with frame_id: %s (reference: %s)",
                      frame_id.c_str(), reference_frame.c_str());
            // Publish target frame
            publish_target_frame(target_pose_any_frame, frame_id, stamp);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "SpeedController initialization failed: %s", e.what());
    }
}

// !
bool OffboardControlNode::rc_should_abort(std::string & reason, double stick_thresh)
{
    if (is_simulator_ && !enforce_rc_in_sim_) {
        return false;  // no RC link in SITL unless explicitly enforced
    }
    px4_msgs::msg::ManualControlSetpoint manual_control;
    uint64_t manual_control_ts;
    {
        std::lock_guard<std::mutex> lk(telemetry_mutex_);
        manual_control = current_manual_control_;
        manual_control_ts = current_manual_control_timestamp_;
    }
    if (check_kill_switch_) {
        const uint8_t SWITCH_POS_ON = 1;
        const int KILL_SWITCH_BIT = 12;
        const uint8_t kill_switch = (manual_control.buttons >> KILL_SWITCH_BIT) & 0b11;
        if (kill_switch == SWITCH_POS_ON) {
            reason = "kill_switch";
            return true;
        }
    }
    // manual_control_timeout = 0 keeps the RC-loss check off (documented default);
    // set e.g. 1.0 s to park the drone when the RC stream goes stale.
    if (manual_control_timeout_ > 0.0) {
        const double age_s = (getTimestamp() - manual_control_ts) / 1000000.0;
        if (age_s > manual_control_timeout_) {
            reason = "rc_timeout";
            return true;
        }
    }
    // Pilot stick override. Needs COM_RC_OVERRIDE=3 on the PX4 side for PX4
    // itself to hand control back — see the RC section in Readme.md.
    if (stick_thresh > 0.0 &&
        (std::abs(manual_control.roll) > stick_thresh ||
         std::abs(manual_control.pitch) > stick_thresh ||
         std::abs(manual_control.yaw) > stick_thresh)) {
        reason = "rc_stick";
        return true;
    }
    return false;
}

void OffboardControlNode::checkManualControl() {
    // Real RC gate (previously hard-disabled by `|| true`). Stick override is
    // handled per-tick in the control loop, not here.
    std::string reason;
    if (!rc_should_abort(reason, 0.0)) {
        return;
    }
    if (reason == "kill_switch") {
        stopOffboardHeartbeat();
        throw std::runtime_error("Kill switch is on");
    }
    throw std::runtime_error("Manual control timeout, RC is switched off?");
}
// !

bool OffboardControlNode::wait_for_vehicle_state(
    const std::function<bool(const px4_msgs::msg::VehicleStatus &)> & predicate,
    double timeout_s)
{
    std::unique_lock<std::mutex> lk(arm_wait_mutex_);
    return arm_wait_cv_.wait_for(lk, std::chrono::duration<double>(timeout_s), [&] {
        return !rclcpp::ok() || command_abort_.load() || predicate(snapshot_vehicle_status());
    }) && rclcpp::ok() && !command_abort_.load();
}

bool OffboardControlNode::offboardAndArm() {
    checkManualControl();
    // The setpoint timer is the ONLY setpoint publisher: warm PX4 up by letting
    // it stream offboard mode + idle/active setpoints. This thread never
    // publishes; it just parks on the CV. The timer keeps running during the
    // whole wait, so offboard does not drop out by COM_OF_LOSS_T.
    startOffboardHeartbeat();
    {
        // Warmup window before the mode switch (was 10 x 100 ms of service-thread
        // publishing; now the timer streams while we park here).
        std::unique_lock<std::mutex> lk(arm_wait_mutex_);
        arm_wait_cv_.wait_for(lk, std::chrono::milliseconds(1000),
                              [this] { return !rclcpp::ok() || command_abort_.load(); });
    }
    if (!rclcpp::ok() || command_abort_.load()) {
        return false;
    }
    switch_to_offboard_mode();
    if (!wait_for_vehicle_state([](const px4_msgs::msg::VehicleStatus & s) {
            return s.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
        }, offboard_timeout_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to switch to OFFBOARD mode: Timeout.");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Successfully switched to OFFBOARD mode.");
    arm();
    if (!wait_for_vehicle_state([](const px4_msgs::msg::VehicleStatus & s) {
            return s.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        }, arming_timeout_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to arm: Timeout.");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Successfully armed.");
    return true;
}

namespace {
// Shared geofence math (general fence + excitation fence use the same check):
// returns "" when inside, otherwise the violated constraint. NaN coordinates
// (partial navigate targets = "keep current") compare false and stay inside —
// the runtime fence catches an actual escape.
std::string check_cylinder(double dx, double dy, double radius,
                           double alt, double alt_floor, double alt_ceil) {
    if (std::sqrt(dx * dx + dy * dy) > radius) {
        return "geofence";
    }
    if (alt < alt_floor || alt > alt_ceil) {
        return "altitude";
    }
    return "";
}
}  // namespace

CurrentPose OffboardControlNode::snapshot_local_position() {
    std::lock_guard<std::mutex> lk(telemetry_mutex_);
    return current_local_position_;
}

px4_msgs::msg::VehicleStatus OffboardControlNode::snapshot_vehicle_status(uint64_t * timestamp) {
    std::lock_guard<std::mutex> lk(telemetry_mutex_);
    if (timestamp) {
        *timestamp = current_vehicle_status_timestamp_;
    }
    return current_vehicle_status_;
}

OffboardControlNode::CmdSnapshot OffboardControlNode::snapshot_cmd_state() {
    CmdSnapshot cmd;
    // Feed-forward correction copied under its OWN lock, fully released before
    // cmd_mutex_ is taken below — the two are never held together (lock order).
    {
        int64_t recv_ns;
        {
            std::lock_guard<std::mutex> lk(corr_mutex_);
            cmd.correction_velocity = correction_velocity_;
            cmd.correction_acceleration = correction_acceleration_;
            cmd.correction_yaw_rate = correction_yaw_rate_;
            cmd.correction_frame_id = correction_frame_id_;
            recv_ns = correction_recv_ns_;
        }
        if (recv_ns != 0 && correction_timeout_ > 0.0) {
            const double age_s = (this->now().nanoseconds() - recv_ns) / 1e9;
            cmd.correction_fresh = (age_s <= correction_timeout_);
        }
    }
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    cmd.type = setpoint_type_;
    cmd.nav_controller = global_speed_controller_;
    cmd.last_pose = last_sended_pose;
    cmd.altitude_override_set = altitude_override_set_;
    cmd.altitude_override_z = altitude_override_z_;
    cmd.altitude_override_frame_id = altitude_override_frame_id_;
    cmd.yaw_override_set = yaw_override_set_;
    cmd.yaw_override = yaw_override_;
    cmd.yaw_override_frame_id = yaw_override_frame_id_;
    cmd.yaw_rate_override_set = yaw_rate_override_set_;
    cmd.yaw_rate_override = yaw_rate_override_;
    cmd.position_x = position_target_x_;
    cmd.position_y = position_target_y_;
    cmd.position_z = position_target_z_;
    cmd.position_yaw = position_target_yaw_;
    cmd.position_frame_id = position_target_frame_id_;
    cmd.velocity_vx = velocity_target_vx_;
    cmd.velocity_vy = velocity_target_vy_;
    cmd.velocity_vz = velocity_target_vz_;
    cmd.velocity_yaw = velocity_target_yaw_;
    cmd.velocity_frame_id = velocity_target_frame_id_;
    cmd.trajectory_position = trajectory_position_;
    cmd.trajectory_velocity = trajectory_velocity_;
    cmd.trajectory_acceleration = trajectory_acceleration_;
    cmd.trajectory_yaw = trajectory_yaw_;
    cmd.trajectory_yaw_rate = trajectory_yaw_rate_;
    cmd.trajectory_frame_id = trajectory_frame_id_;
    cmd.attitude_roll = attitude_roll_;
    cmd.attitude_pitch = attitude_pitch_;
    cmd.attitude_yaw = attitude_yaw_;
    cmd.attitude_thrust = attitude_thrust_;
    cmd.attitude_frame_id = attitude_frame_id_;
    cmd.rates_roll = rates_roll_;
    cmd.rates_pitch = rates_pitch_;
    cmd.rates_yaw = rates_yaw_;
    cmd.rates_thrust = rates_thrust_;
    return cmd;
}

void OffboardControlNode::publish_setpoint() {
    publish_offboard_control_mode();
    // Commanded state is copied under cmd_mutex_ and published from the copy —
    // TF lookups and publishing below run without the lock.
    const CmdSnapshot cmd = snapshot_cmd_state();

    // RC/kill-switch interception for EVERY active maneuver. EXCITE keeps its
    // own abort path (excite_rc_abort → graceful settle-out/revert) — do not
    // double-handle it here.
    if (cmd.type != NONE && cmd.type != EXCITE) {
        std::string rc_reason;
        if (rc_should_abort(rc_reason, rc_stick_abort_thresh_)) {
            if (rc_reason == "rc_timeout") {
                // RC LOST — no pilot to take over: apply the configured
                // onboard failsafe action (hold/land/rtl).
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "RC lost: stopping maneuver type %s -> onboard failsafe",
                    setpoint_type_to_string(cmd.type));
                failsafe_fallback(rc_reason);
            } else {
                // Pilot takeover (kill-switch / stick): release the stream and
                // let PX4 hand control back (COM_RC_OVERRIDE/COM_OBL_RC_ACT).
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "RC abort (%s): stopping maneuver type %s, releasing offboard stream to PX4 failsafe",
                    rc_reason.c_str(), setpoint_type_to_string(cmd.type));
                stopOffboardHeartbeat();
                {
                    std::lock_guard<std::mutex> lk(cmd_mutex_);
                    last_abort_reason_ = rc_reason;
                }
                busy_ = false;
            }
            return;
        }
    }

    // Telemetry-freshness watchdog (prompt 98): an offboard maneuver flying on a
    // stale pose is parked. EXCITE checks freshness in excite_safety_ok().
    if (enforce_freshness_ && cmd.type != NONE && cmd.type != EXCITE) {
        uint64_t pos_ts, status_ts;
        uint8_t nav_state;
        {
            std::lock_guard<std::mutex> lk(telemetry_mutex_);
            pos_ts = current_local_position_.timestamp;
            status_ts = current_vehicle_status_timestamp_;
            nav_state = current_vehicle_status_.nav_state;
        }
        const bool in_offboard =
            (nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
        const bool position_dependent =
            (cmd.type == NAVIGATE || cmd.type == NAVIGATE_GLOBAL ||
             cmd.type == POSITION || cmd.type == VELOCITY || cmd.type == TRAJECTORY);
        const auto now_us = getTimestamp();
        if (in_offboard && position_dependent && local_position_timeout_ > 0.0) {
            if (pos_ts == 0) {
                // Never received odometry — nothing valid to hold: release the
                // stream and let the PX4 failsafe take over.
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Freshness watchdog: no odometry ever received in offboard -> stop stream");
                stopOffboardHeartbeat();
                {
                    std::lock_guard<std::mutex> lk(cmd_mutex_);
                    last_abort_reason_ = "no_odometry";
                }
                busy_ = false;
                return;
            }
            const double pos_age_s = (now_us - pos_ts) / 1000000.0;
            if (pos_age_s > local_position_timeout_) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Freshness watchdog: local position stale (%.2f s > %.2f s) -> onboard failsafe",
                    pos_age_s, local_position_timeout_);
                failsafe_fallback("stale_local_position");
                return;
            }
        }
        // Vehicle status staleness: warn only (no dedicated timeout parameter);
        // 3x the position timeout tolerates the low native status rate.
        if (in_offboard && status_ts > 0 && local_position_timeout_ > 0.0) {
            const double status_age_s = (now_us - status_ts) / 1000000.0;
            if (status_age_s > 3.0 * local_position_timeout_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Freshness watchdog: vehicle status stale (%.2f s)", status_age_s);
            }
        }
    }

    // Runtime geofence (prompt 99): a drone escaping the fence mid-maneuver is
    // parked. POSITION is exempt — it is the fallback hold itself and a valid
    // in-fence POSITION target is what pulls the drone back inside; EXCITE has
    // its own start-relative fence in excite_safety_ok().
    if (geofence_enable_ && cmd.type != NONE && cmd.type != EXCITE && cmd.type != POSITION) {
        uint8_t nav_state;
        CurrentPose pos;
        {
            std::lock_guard<std::mutex> lk(telemetry_mutex_);
            nav_state = current_vehicle_status_.nav_state;
            pos = current_local_position_;
        }
        if (nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
            pos.timestamp != 0) {
            const Pose enu = TransformPX4::nedToEnuPose(Pose(pos));
            const std::string violated = check_cylinder(
                enu.x - geofence_center_x_, enu.y - geofence_center_y_,
                geofence_radius_, enu.z, alt_floor_, alt_ceil_);
            if (!violated.empty()) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Geofence watchdog: %s limit violated at ENU (%.2f, %.2f, %.2f) -> onboard failsafe",
                    violated.c_str(), enu.x, enu.y, enu.z);
                failsafe_fallback("geofence_" + violated);
                return;
            }
        }
    }

    Pose current_fcu_pose_ned(snapshot_local_position());
    const float nan_val = std::numeric_limits<float>::quiet_NaN();

    if (cmd.type == NAVIGATE && cmd.nav_controller) {
        if (cmd.nav_controller->isEnd(current_fcu_pose_ned)) {
            {
                std::lock_guard<std::mutex> lk(cmd_mutex_);
                // Re-check under the lock: a service may have replaced the maneuver
                // while this tick was evaluating the (stale) snapshot.
                if (setpoint_type_ == NAVIGATE && global_speed_controller_ == cmd.nav_controller) {
                    global_speed_controller_.reset();
                    setpoint_type_ = NONE;
                }
            }
            RCLCPP_INFO(this->get_logger(), "Navigation complite");
            return;
        }

        auto sp = cmd.nav_controller->getCurrentSetpointMSG_NED(getTimestamp());
        sp.timestamp = getTimestamp();

        if (cmd.altitude_override_set) {
            double ned_z;
            if (get_altitude_ned_from_frame(cmd.altitude_override_z, cmd.altitude_override_frame_id, ned_z)) {
                sp.position[2] = static_cast<float>(ned_z);
            }
        }
        if (cmd.yaw_rate_override_set) {
            sp.yawspeed = static_cast<float>(cmd.yaw_rate_override);
            sp.yaw = nan_val;
        } else if (cmd.yaw_override_set) {
            double ned_yaw;
            if (transform_yaw_to_ned(cmd.yaw_override, cmd.yaw_override_frame_id, ned_yaw)) {
                sp.yaw = static_cast<float>(ned_yaw);
            }
            sp.yawspeed = nan_val;
        }

        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            last_sended_pose = sp;
        }
        trajectory_setpoint_publisher_->publish(sp);
        Pose setpoint_pose_ned(sp.position[0], sp.position[1], sp.position[2], sp.yaw);
        publish_setpoint_frame(setpoint_pose_ned, this->now());
    } else if (cmd.type == POSITION) {
        geometry_msgs::msg::Point pt;
        pt.x = cmd.position_x;
        pt.y = cmd.position_y;
        pt.z = cmd.position_z;
        geometry_msgs::msg::Point map_pt;
        if (!transform_point(pt, cmd.position_frame_id, map_frame_id_, get_transform_lookup_time(cmd.position_frame_id, map_frame_id_), map_pt)) {
            publish_idle_setpoint();
            return;
        }
        Pose target_enu(map_pt.x, map_pt.y, map_pt.z, cmd.position_yaw);
        Pose target_ned = TransformPX4::enuToNedPose(target_enu);
        if (cmd.altitude_override_set) {
            double ned_z;
            if (get_altitude_ned_from_frame(cmd.altitude_override_z, cmd.altitude_override_frame_id, ned_z)) {
                target_ned.z = ned_z;
            }
        }
        auto sp = cmd.last_pose;
        if (cmd.yaw_rate_override_set) {
            sp.position[0] = static_cast<float>(target_ned.x);
            sp.position[1] = static_cast<float>(target_ned.y);
            sp.position[2] = static_cast<float>(target_ned.z);
            sp.yaw = nan_val;
            sp.yawspeed = static_cast<float>(cmd.yaw_rate_override);
        } else if (cmd.yaw_override_set) {
            double ned_yaw;
            if (transform_yaw_to_ned(cmd.yaw_override, cmd.yaw_override_frame_id, ned_yaw)) {
                target_ned.yaw = ned_yaw;
            }
            sp.position[0] = static_cast<float>(target_ned.x);
            sp.position[1] = static_cast<float>(target_ned.y);
            sp.position[2] = static_cast<float>(target_ned.z);
            sp.yaw = static_cast<float>(target_ned.yaw);
            sp.yawspeed = nan_val;
        } else {
            sp.position[0] = static_cast<float>(target_ned.x);
            sp.position[1] = static_cast<float>(target_ned.y);
            sp.position[2] = static_cast<float>(target_ned.z);
            sp.yaw = static_cast<float>(target_ned.yaw);
            sp.yawspeed = nan_val;
        }
        sp.timestamp = getTimestamp();
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            last_sended_pose = sp;
        }
        trajectory_setpoint_publisher_->publish(sp);
        publish_setpoint_frame(target_ned, this->now());
    } else if (cmd.type == VELOCITY) {
        geometry_msgs::msg::Vector3 vel;
        vel.x = cmd.velocity_vx;
        vel.y = cmd.velocity_vy;
        vel.z = cmd.velocity_vz;
        geometry_msgs::msg::Vector3 vel_enu;
        if (!transform_velocity(vel, cmd.velocity_frame_id, map_frame_id_, get_transform_lookup_time(cmd.velocity_frame_id, map_frame_id_), vel_enu)) {
            publish_idle_setpoint();
            return;
        }
        double ned_yaw;
        if (!transform_yaw_to_ned(cmd.velocity_yaw, cmd.velocity_frame_id, ned_yaw)) {
            ned_yaw = current_fcu_pose_ned.yaw;
        }
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = getTimestamp();
        msg.position[0] = msg.position[1] = msg.position[2] = nan_val;
        msg.velocity[0] = static_cast<float>(vel_enu.y);
        msg.velocity[1] = static_cast<float>(vel_enu.x);
        msg.velocity[2] = static_cast<float>(-vel_enu.z);
        msg.yaw = static_cast<float>(ned_yaw);
        msg.yawspeed = nan_val;
        trajectory_setpoint_publisher_->publish(msg);
    } else if (cmd.type == TRAJECTORY) {
        // Single trajectory setpoint: base target (position/yaw) from the service,
        // velocity/acceleration/yaw_rate feed-forward from the base OR a fresh
        // ~/trajectory_correction (merged per group below). Unset groups stay NaN so
        // PX4 does not treat an unset axis as a zero command. The matching
        // OffboardControlMode flags are raised group-wise in publish_offboard_control_mode().
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = getTimestamp();
        msg.position[0] = msg.position[1] = msg.position[2] = nan_val;
        msg.velocity[0] = msg.velocity[1] = msg.velocity[2] = nan_val;
        msg.acceleration[0] = msg.acceleration[1] = msg.acceleration[2] = nan_val;
        msg.jerk[0] = msg.jerk[1] = msg.jerk[2] = nan_val;
        msg.yaw = nan_val;
        msg.yawspeed = nan_val;

        const auto lookup = get_transform_lookup_time(cmd.trajectory_frame_id, map_frame_id_);

        if (all_finite_3(cmd.trajectory_position)) {
            geometry_msgs::msg::Point pt;
            pt.x = cmd.trajectory_position[0];
            pt.y = cmd.trajectory_position[1];
            pt.z = cmd.trajectory_position[2];
            geometry_msgs::msg::Point map_pt;
            if (!transform_point(pt, cmd.trajectory_frame_id, map_frame_id_, lookup, map_pt)) {
                publish_idle_setpoint();
                return;
            }
            Pose target_ned = TransformPX4::enuToNedPose(Pose(map_pt.x, map_pt.y, map_pt.z, 0.0));
            msg.position[0] = static_cast<float>(target_ned.x);
            msg.position[1] = static_cast<float>(target_ned.y);
            msg.position[2] = static_cast<float>(target_ned.z);
        }

        // Velocity / acceleration / yaw_rate: prefer a FRESH ~/trajectory_correction
        // value (each group independently), else fall back to the base service group.
        // Each group carries its own frame (correction frame, empty = base frame).
        const std::string base_frame = cmd.trajectory_frame_id;
        const std::string corr_frame =
            cmd.correction_frame_id.empty() ? base_frame : cmd.correction_frame_id;

        std::array<double, 3> eff_vel = cmd.trajectory_velocity;
        std::string eff_vel_frame = base_frame;
        if (cmd.correction_fresh && all_finite_3(cmd.correction_velocity)) {
            eff_vel = cmd.correction_velocity;
            eff_vel_frame = corr_frame;
        }
        if (all_finite_3(eff_vel)) {
            geometry_msgs::msg::Vector3 vel;
            vel.x = eff_vel[0];
            vel.y = eff_vel[1];
            vel.z = eff_vel[2];
            geometry_msgs::msg::Vector3 vel_enu;
            if (!transform_velocity(vel, eff_vel_frame, map_frame_id_,
                                    get_transform_lookup_time(eff_vel_frame, map_frame_id_), vel_enu)) {
                publish_idle_setpoint();
                return;
            }
            set_ned_from_enu_vector(vel_enu, msg.velocity);
        }

        std::array<double, 3> eff_acc = cmd.trajectory_acceleration;
        std::string eff_acc_frame = base_frame;
        if (cmd.correction_fresh && all_finite_3(cmd.correction_acceleration)) {
            eff_acc = cmd.correction_acceleration;
            eff_acc_frame = corr_frame;
        }
        if (all_finite_3(eff_acc)) {
            geometry_msgs::msg::Vector3 accel;
            accel.x = eff_acc[0];
            accel.y = eff_acc[1];
            accel.z = eff_acc[2];
            geometry_msgs::msg::Vector3 accel_enu;
            if (!transform_velocity(accel, eff_acc_frame, map_frame_id_,
                                    get_transform_lookup_time(eff_acc_frame, map_frame_id_), accel_enu)) {
                publish_idle_setpoint();
                return;
            }
            set_ned_from_enu_vector(accel_enu, msg.acceleration);
        }

        if (std::isfinite(cmd.trajectory_yaw)) {
            double ned_yaw;
            if (!transform_yaw_to_ned(cmd.trajectory_yaw, base_frame, ned_yaw)) {
                ned_yaw = current_fcu_pose_ned.yaw;
            }
            msg.yaw = static_cast<float>(ned_yaw);
        }
        double eff_yaw_rate = cmd.trajectory_yaw_rate;
        if (cmd.correction_fresh && std::isfinite(cmd.correction_yaw_rate)) {
            eff_yaw_rate = cmd.correction_yaw_rate;
        }
        if (std::isfinite(eff_yaw_rate)) {
            msg.yawspeed = static_cast<float>(eff_yaw_rate);
        }

        trajectory_setpoint_publisher_->publish(msg);
    } else if (cmd.type == ATTITUDE) {
        double ned_yaw;
        if (!transform_yaw_to_ned(static_cast<double>(cmd.attitude_yaw), cmd.attitude_frame_id, ned_yaw)) {
            ned_yaw = current_fcu_pose_ned.yaw;
        }
        Eigen::Quaterniond q_ned = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(cmd.attitude_roll, cmd.attitude_pitch, ned_yaw);
        px4_msgs::msg::VehicleAttitudeSetpoint msg{};
        msg.timestamp = getTimestamp();
        msg.q_d[0] = static_cast<float>(q_ned.w());
        msg.q_d[1] = static_cast<float>(q_ned.x());
        msg.q_d[2] = static_cast<float>(q_ned.y());
        msg.q_d[3] = static_cast<float>(q_ned.z());
        msg.thrust_body[0] = 0.0f;
        msg.thrust_body[1] = 0.0f;
        msg.thrust_body[2] = -std::max(0.0f, std::min(1.0f, cmd.attitude_thrust));
        msg.yaw_sp_move_rate = std::numeric_limits<float>::quiet_NaN();
        vehicle_attitude_setpoint_publisher_->publish(msg);
    } else if (cmd.type == RATES) {
        px4_msgs::msg::VehicleRatesSetpoint msg{};
        msg.timestamp = getTimestamp();
        msg.roll = cmd.rates_roll;
        msg.pitch = cmd.rates_pitch;
        msg.yaw = cmd.rates_yaw;
        msg.thrust_body[0] = 0.0f;
        msg.thrust_body[1] = 0.0f;
        msg.thrust_body[2] = -std::max(0.0f, std::min(1.0f, cmd.rates_thrust));
        vehicle_rates_setpoint_publisher_->publish(msg);
    } else if (cmd.type == FLIP) {
        // flip_* runtime fields are only mutated on this timer thread after the
        // service handoff; phase transitions still go under a short cmd_mutex_
        // lock so publish_offboard_control_mode() reads a consistent phase.
        auto now = this->now();

        if (flip_phase_ == FlipPhase::CLIMB) {
            double elapsed = (now - flip_climb_start_time_).seconds();
            if (elapsed >= flip_climb_duration_) {
                std::lock_guard<std::mutex> lk(cmd_mutex_);
                flip_phase_ = FlipPhase::ROTATE;
                flip_rotate_start_time_ = now;
                flip_angle_done_ = 0.0;
            } else {
                px4_msgs::msg::TrajectorySetpoint msg{};
                msg.timestamp = getTimestamp();
                msg.position[0] = msg.position[1] = msg.position[2] = nan_val;
                msg.velocity[0] = 0.0f;
                msg.velocity[1] = 0.0f;
                msg.velocity[2] = static_cast<float>(-flip_vz_);
                msg.yaw = static_cast<float>(flip_initial_pose_ned_.yaw);
                msg.yawspeed = nan_val;
                trajectory_setpoint_publisher_->publish(msg);
            }
        }
        if (flip_phase_ == FlipPhase::ROTATE) {
            double dt = 1.0 / control_rate_;
            flip_angle_done_ += std::abs(flip_rate_) * dt;
            if (flip_angle_done_ >= flip_target_angle_) {
                std::lock_guard<std::mutex> lk(cmd_mutex_);
                flip_phase_ = FlipPhase::POSITION_HOLD;
            } else {
                px4_msgs::msg::VehicleRatesSetpoint msg{};
                msg.timestamp = getTimestamp();
                float rate = static_cast<float>(flip_rate_);
                msg.roll = (flip_axis_ == FlipAxis::ROLL) ? rate : 0.0f;
                msg.pitch = (flip_axis_ == FlipAxis::PITCH) ? rate : 0.0f;
                msg.yaw = (flip_axis_ == FlipAxis::YAW) ? rate : 0.0f;
                msg.thrust_body[0] = 0.0f;
                msg.thrust_body[1] = 0.0f;
                msg.thrust_body[2] = -std::max(0.0f, std::min(1.0f, flip_thrust_));
                vehicle_rates_setpoint_publisher_->publish(msg);
            }
        }
        if (flip_phase_ == FlipPhase::POSITION_HOLD) {
            auto sp = cmd.last_pose;
            sp.position[0] = static_cast<float>(flip_initial_pose_ned_.x);
            sp.position[1] = static_cast<float>(flip_initial_pose_ned_.y);
            sp.position[2] = static_cast<float>(flip_initial_pose_ned_.z);
            sp.yaw = static_cast<float>(flip_initial_pose_ned_.yaw);
            sp.yawspeed = nan_val;
            sp.timestamp = getTimestamp();
            {
                std::lock_guard<std::mutex> lk(cmd_mutex_);
                last_sended_pose = sp;
                if (setpoint_type_ == FLIP) {
                    setpoint_type_ = NONE;
                }
            }
            trajectory_setpoint_publisher_->publish(sp);
            RCLCPP_INFO(this->get_logger(), "Flip complete, position hold set to initial pose");
        }
    } else if (cmd.type == EXCITE) {
        publish_excite_setpoint();
    } else if (cmd.type == NONE) {
        publish_idle_setpoint();
    }
}

// !
void OffboardControlNode::startOffboardHeartbeat()
{
    // Resume the persistent setpoint timer if it was stopped (e.g. after land /
    // kill-switch). The timer object is created ONCE in the constructor
    // (Init_timer); services no longer re-create it on every setpoint change —
    // behavior is selected inside publish_setpoint() by setpoint_type_. Only the
    // excitation path still calls Init_timer() to change the stream rate.
    if (setpoint_timer_ && setpoint_timer_->is_canceled()) {
        setpoint_timer_->reset();
    }
}
bool OffboardControlNode::validate_setpoint(SetpointType kind, double x, double y, double z,
                                            double speed, std::string & reason)
{
    // Numeric core is a free header function (unit-testable, prompt 102).
    return validate_setpoint_numeric(kind, x, y, z, speed, max_speed_, reason);
}

bool OffboardControlNode::validate_target_in_geofence(double x, double y, double z,
                                                      const std::string & frame_id, std::string & reason)
{
    if (!geofence_enable_) {
        return true;
    }
    geometry_msgs::msg::Point pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    geometry_msgs::msg::Point map_pt = pt;
    if (!(frame_id.empty() || frame_id == map_frame_id_ || frame_id == "map")) {
        if (!transform_point(pt, frame_id, map_frame_id_,
                             get_transform_lookup_time(frame_id, map_frame_id_), map_pt)) {
            reason = "geofence: cannot transform target to map frame for validation";
            return false;
        }
    }
    const std::string violated = check_cylinder(
        map_pt.x - geofence_center_x_, map_pt.y - geofence_center_y_,
        geofence_radius_, map_pt.z, alt_floor_, alt_ceil_);
    if (!violated.empty()) {
        reason = "target rejected: outside " + violated + " limits "
                 "(radius " + std::to_string(geofence_radius_) +
                 " m, alt [" + std::to_string(alt_floor_) + ", " + std::to_string(alt_ceil_) + "] m)";
        return false;
    }
    return true;
}

void OffboardControlNode::park_to_hold(const std::string & reason)
{
    // Shared safe-fallback: switch whatever maneuver is active into a plain
    // position hold at the CURRENT pose (keeps the offboard stream alive, unlike
    // stopOffboardHeartbeat which releases the drone to the PX4 failsafe).
    const CurrentPose pos = snapshot_local_position();
    const Pose cur_enu = TransformPX4::nedToEnuPose(Pose(pos));
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        global_speed_controller_.reset();
        position_target_x_ = cur_enu.x;
        position_target_y_ = cur_enu.y;
        position_target_z_ = cur_enu.z;
        position_target_yaw_ = cur_enu.yaw;
        position_target_frame_id_ = map_frame_id_;
        setpoint_type_ = POSITION;
        last_abort_reason_ = reason;
    }
    startOffboardHeartbeat();
    busy_ = false;
    RCLCPP_WARN(this->get_logger(), "Park to hold (%s): holding current pose (ENU %.2f, %.2f, %.2f)",
                reason.c_str(), cur_enu.x, cur_enu.y, cur_enu.z);
}

void OffboardControlNode::stopOffboardHeartbeat()
{
    // Stop send offboard
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        setpoint_type_ = SetpointType::NONE;
        global_speed_controller_.reset();
    }
    if (setpoint_timer_) {
        setpoint_timer_->cancel();
    }
}

void OffboardControlNode::failsafe_fallback(const std::string & reason)
{
    // Node-side critical rollback (freshness/geofence watchdogs, RC loss).
    // Configurable via onboard_failsafe_action; the PX4-side failsafe
    // (COM_OBL_RC_ACT etc.) is NOT duplicated here — see Readme.md.
    if (onboard_failsafe_action_ == "land") {
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            last_abort_reason_ = reason;
        }
        RCLCPP_ERROR(this->get_logger(), "Failsafe (%s): action=land", reason.c_str());
        land();  // stops the offboard stream and sends VEHICLE_CMD_NAV_LAND
        busy_ = false;
    } else if (onboard_failsafe_action_ == "rtl") {
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            last_abort_reason_ = reason;
        }
        RCLCPP_ERROR(this->get_logger(), "Failsafe (%s): action=rtl", reason.c_str());
        stopOffboardHeartbeat();
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
        busy_ = false;
    } else if (onboard_failsafe_action_ == "position") {
        // Release stream -> PX4 falls back to COM_OBL_RC_ACT (Position). Preferred
        // when the FMU owns position (optical flow + rangefinder); avoids holding
        // on a stale pose like "hold" does.
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            last_abort_reason_ = reason;
        }
        RCLCPP_ERROR(this->get_logger(),
            "Failsafe (%s): action=position (release stream -> PX4 COM_OBL_RC_ACT)", reason.c_str());
        stopOffboardHeartbeat();
        busy_ = false;
    } else {
        park_to_hold(reason);
    }
}
// !

#pragma region TF2
void OffboardControlNode::clear_navigation_targets() {
    if (!target_transform_.child_frame_id.empty()) {
        target_transform_.header.stamp = this->now();
        target_transform_.header.frame_id = map_frame_id_;
        target_transform_.transform.translation.x = 0.0;
        target_transform_.transform.translation.y = 0.0;
        target_transform_.transform.translation.z = 0.0;
        tf2::Quaternion q_reset;
        q_reset.setRPY(0, 0, 0);
        target_transform_.transform.rotation = tf2::toMsg(q_reset);
        static_tf_broadcaster_.sendTransform(target_transform_);
    }
    
    if (!setpoint_transform_.child_frame_id.empty()) {
        setpoint_transform_.header.stamp = this->now();
        setpoint_transform_.header.frame_id = map_frame_id_;
        setpoint_transform_.transform.translation.x = 0.0;
        setpoint_transform_.transform.translation.y = 0.0;
        setpoint_transform_.transform.translation.z = 0.0;
        tf2::Quaternion q_reset2;
        q_reset2.setRPY(0, 0, 0);
        setpoint_transform_.transform.rotation = tf2::toMsg(q_reset2);
        tf_broadcaster_.sendTransform(setpoint_transform_);
    }
    
    RCLCPP_INFO(this->get_logger(), "Navigation targets cleared after landing");
}

void OffboardControlNode::create_reference_frames() {
    reference_frames_.clear();
    
    reference_frames_[body_frame_id_] = map_frame_id_;
    reference_frames_["body"] = map_frame_id_;
    reference_frames_["setpoint"] = map_frame_id_;
    reference_frames_["navigate_target"] = map_frame_id_;
    reference_frames_["aruco_map"] = map_frame_id_;
    reference_frames_["terrain"] = map_frame_id_;
    reference_frames_["setpoint"] = map_frame_id_;
    reference_frames_[map_frame_id_] = map_frame_id_;
}

std::string OffboardControlNode::get_reference_frame(const std::string& frame_id) {
    if (frame_id.empty()) return map_frame_id_;
    
    auto it = reference_frames_.find(frame_id);
    if (it != reference_frames_.end()) {
        return it->second;
    }
    return frame_id;
}

bool OffboardControlNode::validate_frame_id(const std::string& frame_id) {
    static std::unordered_set<std::string> valid_frames_cache;
   
    // Add frames from reference_frames
    for (const auto& pair : reference_frames_) {
        valid_frames_cache.insert(pair.first);
        valid_frames_cache.insert(pair.second);
    }
    
    // Check frame_id against allowed-frame cache
    if (valid_frames_cache.find(frame_id) != valid_frames_cache.end()) {
        return true;
    }
    
    // If not in cache, match pattern "aruco_*"
    const std::string aruco_prefix = "aruco_";
    if (frame_id.compare(0, aruco_prefix.length(), aruco_prefix) == 0) {
        std::string number_str = frame_id.substr(aruco_prefix.length());
        
        if (number_str.empty()) {
            return false;
        }
        
        // Fast check: all chars are digits
        for (char c : number_str) {
            if (!std::isdigit(static_cast<unsigned char>(c))) {
                return false;
            }
        }
        
        // Check numeric range
        // For values <1000, string length is enough
        if (number_str.length() > 4) {  // 1000 has 4 digits
            return false;
        }
        
        // Parse to int for stricter check
        try {
            int number = std::stoi(number_str);
            return (number >= 0 && number <= 1000);
        } catch (...) {
            return false;
        }
    }
    
    return false;
}

rclcpp::Time OffboardControlNode::get_transform_lookup_time() const {
    return is_simulator_ ? rclcpp::Time(0) : this->now();
}

rclcpp::Time OffboardControlNode::get_transform_lookup_time(const std::string& source_frame, const std::string& target_frame) const {
    if (source_frame == aruco_map_frame_id_ || target_frame == aruco_map_frame_id_) {
        return rclcpp::Time(0);
    }
    return get_transform_lookup_time();
}

bool OffboardControlNode::get_altitude_ned_from_frame(double z, const std::string& frame_id, double& out_ned_z) {
    geometry_msgs::msg::Point pt;
    pt.x = 0.0;
    pt.y = 0.0;
    pt.z = z;
    geometry_msgs::msg::Point map_pt;
    if (!transform_point(pt, frame_id, map_frame_id_, get_transform_lookup_time(frame_id, map_frame_id_), map_pt)) {
        return false;
    }
    Eigen::Vector3d enu_vec(map_pt.x, map_pt.y, map_pt.z);
    Eigen::Vector3d ned_vec = px4_ros_com::frame_transforms::enu_to_ned_local_frame(enu_vec);
    out_ned_z = ned_vec.z();
    return true;
}

bool OffboardControlNode::transform_yaw_to_ned(double yaw, const std::string& frame_id, double& out_ned_yaw) {
    double map_yaw_enu;
    if (frame_id == map_frame_id_ || frame_id == "map") {
        map_yaw_enu = yaw;
    } else {
        if (!transform_yaw(yaw, frame_id, map_frame_id_, get_transform_lookup_time(frame_id, map_frame_id_), map_yaw_enu)) {
            return false;
        }
    }
    out_ned_yaw = TransformPX4::enuToNedYaw(map_yaw_enu);
    return true;
}

void OffboardControlNode::init_tf_frames() {
    // Initialize TF frames
    body_transform_.child_frame_id = "body";
    target_transform_.child_frame_id = "navigate_target";
    setpoint_transform_.child_frame_id = "setpoint";
    terrain_transform_.child_frame_id = "terrain";
    terrain_transform_.header.frame_id = body_frame_id_;
}

bool OffboardControlNode::wait_for_transform(
    const std::string& target_frame,
    const std::string& source_frame,
    const rclcpp::Time& time,
    const rclcpp::Duration& timeout) 
{
    rclcpp::Rate rate(100); // 100 Hz
    auto start_time = this->now();
    
    while (rclcpp::ok()) {
        if ((this->now() - start_time) > timeout) {
            RCLCPP_WARN(this->get_logger(), 
                "Transform timeout: %s -> %s after %.2f seconds",
                source_frame.c_str(), target_frame.c_str(), 
                timeout.seconds());
            return false;
        }
        
        try {
            if (tf_buffer_.canTransform(target_frame, source_frame, time)) {
                return true;
            }
        } catch (const tf2::TransformException& ex) {
            RCLCPP_DEBUG(this->get_logger(), "Transform check exception: %s", ex.what());
        }
        
        rate.sleep();
    }
    return false;
}

bool OffboardControlNode::transform_point(
    const geometry_msgs::msg::Point& point,
    const std::string& source_frame,
    const std::string& target_frame,
    const rclcpp::Time& stamp,
    geometry_msgs::msg::Point& result)
{
    geometry_msgs::msg::PointStamped source_point, target_point;
    source_point.header.frame_id = source_frame;
    source_point.header.stamp = stamp;
    source_point.point = point;
    
    if (!wait_for_transform(target_frame, source_frame, stamp, 
                          rclcpp::Duration::from_seconds(transform_timeout_))) {
        RCLCPP_ERROR(this->get_logger(), 
            "Can't transform point from %s to %s", 
            source_frame.c_str(), target_frame.c_str());
        return false;
    }
    
    try {
        target_point = tf_buffer_.transform(source_point, target_frame);
        result = target_point.point;
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Point transform failed: %s", ex.what());
        return false;
    }
}

bool OffboardControlNode::transform_yaw(
    double yaw,
    const std::string& source_frame,
    const std::string& target_frame,
    const rclcpp::Time& stamp,
    double& result_yaw)
{
    geometry_msgs::msg::QuaternionStamped source_quat, target_quat;
    source_quat.header.frame_id = source_frame;
    source_quat.header.stamp = stamp;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    source_quat.quaternion = tf2::toMsg(q);
    
    if (!wait_for_transform(target_frame, source_frame, stamp, 
                          rclcpp::Duration::from_seconds(transform_timeout_))) {
        RCLCPP_ERROR(this->get_logger(), 
            "Can't transform yaw from %s to %s", 
            source_frame.c_str(), target_frame.c_str());
        return false;
    }
    
    try {
        target_quat = tf_buffer_.transform(source_quat, target_frame);
        tf2::Quaternion q_out;
        tf2::fromMsg(target_quat.quaternion, q_out);
        result_yaw = tf2::impl::getYaw(q_out);
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Yaw transform failed: %s", ex.what());
        return false;
    }
}

bool OffboardControlNode::transform_velocity(
    const geometry_msgs::msg::Vector3& velocity,
    const std::string& source_frame,
    const std::string& target_frame,
    const rclcpp::Time& stamp,
    geometry_msgs::msg::Vector3& result)
{
    geometry_msgs::msg::Vector3Stamped source_vel, target_vel;
    source_vel.header.frame_id = source_frame;
    source_vel.header.stamp = stamp;
    source_vel.vector = velocity;
    
    if (!wait_for_transform(target_frame, source_frame, stamp, 
                          rclcpp::Duration::from_seconds(transform_timeout_))) {
        // Fallback for body-referenced velocity commands: use current odometry yaw
        // to rotate body-frame XY velocity into map frame when TF is temporarily unavailable.
        const bool source_is_body = (source_frame == "body" || source_frame == body_frame_id_);
        const bool target_is_map = (target_frame == map_frame_id_ || target_frame == "map");
        if (source_is_body && target_is_map) {
            const double yaw_enu = TransformPX4::nedToEnuYaw(snapshot_local_position().heading);
            const double c = std::cos(yaw_enu);
            const double s = std::sin(yaw_enu);

            result.x = c * velocity.x - s * velocity.y;
            result.y = s * velocity.x + c * velocity.y;
            result.z = velocity.z;

            RCLCPP_WARN(this->get_logger(),
                "TF unavailable for %s -> %s, using odometry-yaw fallback for velocity transform",
                source_frame.c_str(), target_frame.c_str());
            return true;
        }
        RCLCPP_ERROR(this->get_logger(), 
            "Can't transform velocity from %s to %s", 
            source_frame.c_str(), target_frame.c_str());
        return false;
    }
    
    try {
        target_vel = tf_buffer_.transform(source_vel, target_frame);
        result = target_vel.vector;
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Velocity transform failed: %s", ex.what());
        return false;
    }
}

void OffboardControlNode::publish_static_transforms() {
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    
    // Static transform for navigation target frame
    geometry_msgs::msg::TransformStamped target_static;
    target_static.header.stamp = this->now();
    target_static.header.frame_id = map_frame_id_;
    target_static.child_frame_id = "navigate_target";
    target_static.transform.translation.x = 0.0;
    target_static.transform.translation.y = 0.0;
    target_static.transform.translation.z = 0.0;
    target_static.transform.rotation = tf2::toMsg(q);
    static_tf_broadcaster_.sendTransform(target_static);

    // Static transform for terrain frame
    geometry_msgs::msg::TransformStamped terrain_static;
    terrain_static.header.stamp = this->now();
    terrain_static.header.frame_id = body_frame_id_;
    terrain_static.child_frame_id = "terrain";
    terrain_static.transform.translation.x = 0.0;
    terrain_static.transform.translation.y = 0.0;
    terrain_static.transform.translation.z = 0.0;
    terrain_static.transform.rotation = tf2::toMsg(q);
    static_tf_broadcaster_.sendTransform(terrain_static);
    
    RCLCPP_INFO(this->get_logger(), "Published static transforms");
}

void OffboardControlNode::publish_terrain_frame(double altitude_above_ground) {
    if (terrain_transform_.child_frame_id.empty()) return;
    
    terrain_transform_.header.stamp = this->now();
    terrain_transform_.transform.translation.x = 0.0;
    terrain_transform_.transform.translation.y = 0.0;
    terrain_transform_.transform.translation.z = -altitude_above_ground;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    terrain_transform_.transform.rotation = tf2::toMsg(q);
    
    static_tf_broadcaster_.sendTransform(terrain_transform_);
}

void OffboardControlNode::publish_odometry_transform(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg) 
{
    if (msg->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) {
        RCLCPP_WARN(this->get_logger(), "Odometry frame not in NED, skipping transform");
        return;
    }

    // ВАЖНО: штамповать ИМЕННО now() (время прихода), а НЕ timestamp_sample
    // одометрии. Пробовали sample-время ради точности attitude-prior в
    // aruco_loc (2026-07-24) — оно ставит TF в прошлое на 10-50 мс, и все
    // потребители, делающие lookup «на сейчас» с таймаутом (navigate/
    // SpeedController в этой же ноде), начинают ждать следующий сэмпл КАЖДЫЙ
    // цикл → управление вязнет (в OF-полёте дрон медленно/не возвращался в
    // точку). aruco_loc prior от этого не страдает: lookup идёт на stamp
    // кадра (он в прошлом), arrival-штампованный TF его покрывает, сдвиг
    // таймлайна равен транспортной латентности (~15 мс) — приемлемо.
    auto timestamp = this->now();
    Eigen::Vector3d ned_pos(msg->position[0], msg->position[1], msg->position[2]);
    Eigen::Vector3d enu_pos = px4_ros_com::frame_transforms::ned_to_enu_local_frame(ned_pos);
    
    Eigen::Quaterniond q_ned_to_body(
        msg->q[0],  // w
        msg->q[1],  // x
        msg->q[2],  // y
        msg->q[3]   // z
    );
    
    geometry_msgs::msg::TransformStamped body_transform;
    body_transform.header.stamp = timestamp;
    body_transform.header.frame_id = map_frame_id_;
    body_transform.child_frame_id = "body";
    
    body_transform.transform.translation.x = enu_pos.x();
    body_transform.transform.translation.y = enu_pos.y();
    body_transform.transform.translation.z = enu_pos.z();
    
    Eigen::Quaterniond q_enu_to_body = px4_ros_com::frame_transforms::ned_to_enu_orientation(q_ned_to_body);
    double yaw_only = px4_ros_com::frame_transforms::utils::quaternion::quaternion_get_yaw(q_enu_to_body);
    
    tf2::Quaternion q_yaw_only;
    q_yaw_only.setRPY(0, 0, yaw_only);
    
    body_transform.transform.rotation = tf2::toMsg(q_yaw_only);
    tf_broadcaster_.sendTransform(body_transform);
    
    geometry_msgs::msg::TransformStamped base_link_transform;
    base_link_transform.header.stamp = timestamp;
    base_link_transform.header.frame_id = map_frame_id_;
    base_link_transform.child_frame_id = body_frame_id_;
    
    base_link_transform.transform.translation.x = enu_pos.x();
    base_link_transform.transform.translation.y = enu_pos.y();
    base_link_transform.transform.translation.z = enu_pos.z();
    
    Eigen::Quaterniond q_aircraft_to_ned = q_ned_to_body;
    Eigen::Quaterniond q_base_link_to_enu = px4_ros_com::frame_transforms::px4_to_ros_orientation(q_aircraft_to_ned);
    Eigen::Quaterniond q_enu_to_base_link = q_base_link_to_enu;
    
    base_link_transform.transform.rotation.x = q_enu_to_base_link.x();
    base_link_transform.transform.rotation.y = q_enu_to_base_link.y();
    base_link_transform.transform.rotation.z = q_enu_to_base_link.z();
    base_link_transform.transform.rotation.w = q_enu_to_base_link.w();
    
    tf_broadcaster_.sendTransform(base_link_transform);
    current_odom_transform_ = base_link_transform;
}

void OffboardControlNode::publish_target_frame(
    const Pose& target_pose, 
    const std::string& frame_id,
    const rclcpp::Time& stamp)
{
    if (target_transform_.child_frame_id.empty()) return;
    
    try {
        // Resolve reference frame for target frame
        std::string reference_frame = get_reference_frame(frame_id);
        
        // Transform target position into reference frame
        geometry_msgs::msg::PointStamped target_point;
        target_point.header.frame_id = frame_id;
        target_point.header.stamp = stamp;
        target_point.point.x = target_pose.x;
        target_point.point.y = target_pose.y;
        target_point.point.z = target_pose.z;
        
        geometry_msgs::msg::PointStamped transformed_point = 
            tf_buffer_.transform(target_point, reference_frame);
        
        // Publish transform
        target_transform_.header.frame_id = reference_frame;
        target_transform_.header.stamp = stamp;
        target_transform_.transform.translation.x = transformed_point.point.x;
        target_transform_.transform.translation.y = transformed_point.point.y;
        target_transform_.transform.translation.z = transformed_point.point.z;
        
        // Orientation: use current yaw or transform
        tf2::Quaternion q;
        q.setRPY(0, 0, target_pose.yaw);
        target_transform_.transform.rotation = tf2::toMsg(q);
        
        static_tf_broadcaster_.sendTransform(target_transform_);
        
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to publish target frame: %s", ex.what());
    }
}

void OffboardControlNode::publish_setpoint_frame(const Pose& setpoint_pose_ned, const rclcpp::Time& stamp) {
    if (setpoint_transform_.child_frame_id.empty()) return;
    
    setpoint_transform_.header.frame_id = map_frame_id_;
    setpoint_transform_.header.stamp = stamp;
    
    // NED to ENU for TF
    Pose setpoint_enu = TransformPX4::nedToEnuPose(setpoint_pose_ned);
    setpoint_transform_.transform.translation.x = setpoint_enu.x;
    setpoint_transform_.transform.translation.y = setpoint_enu.y;
    setpoint_transform_.transform.translation.z = setpoint_enu.z;
    
    // Do not publish NaN orientation — TF rejects it and breaks the tree
    double yaw_enu = setpoint_enu.yaw;
    if (!std::isfinite(yaw_enu)) {
        yaw_enu = TransformPX4::nedToEnuYaw(snapshot_local_position().heading);
    }
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_enu);
    setpoint_transform_.transform.rotation = tf2::toMsg(q);
    
    tf_broadcaster_.sendTransform(setpoint_transform_);
}
#pragma endregion TF2

#pragma region PX4 command
void OffboardControlNode::publish_idle_setpoint() {
    px4_msgs::msg::TrajectorySetpoint last_pose;
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        last_pose = last_sended_pose;
    }
    px4_msgs::msg::TrajectorySetpoint msg{};
    // Hold current position
    if (last_pose.timestamp != 0){
        msg.position[0] = last_pose.position[0];
        msg.position[1] = last_pose.position[1];
        msg.position[2] = last_pose.position[2];
        msg.yaw = last_pose.yaw;
    }
    else{
        const CurrentPose pos = snapshot_local_position();
        msg.position[0] = pos.x;
        msg.position[1] = pos.y;
        msg.position[2] = pos.z;
        msg.yaw = pos.heading;
    }
    msg.timestamp = getTimestamp();
    trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControlNode::publish_offboard_control_mode() {
    SetpointType sp_type;
    FlipPhase flip_phase;
    bool traj_has_pos = false, traj_has_vel = false, traj_has_acc = false;
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        sp_type = setpoint_type_;
        flip_phase = flip_phase_;
        if (sp_type == TRAJECTORY) {
            traj_has_pos = any_finite_3(trajectory_position_);
            traj_has_vel = any_finite_3(trajectory_velocity_);
            traj_has_acc = any_finite_3(trajectory_acceleration_);
        }
    }
    // A fresh feed-forward correction can enable the velocity/acceleration flags
    // even when the base service target set only a position (corr_mutex_ read is
    // separate from cmd_mutex_ above — never nested).
    if (sp_type == TRAJECTORY) {
        int64_t recv_ns;
        bool corr_has_vel, corr_has_acc;
        {
            std::lock_guard<std::mutex> lk(corr_mutex_);
            recv_ns = correction_recv_ns_;
            corr_has_vel = any_finite_3(correction_velocity_);
            corr_has_acc = any_finite_3(correction_acceleration_);
        }
        const bool fresh = recv_ns != 0 && correction_timeout_ > 0.0 &&
                           ((this->now().nanoseconds() - recv_ns) / 1e9) <= correction_timeout_;
        if (fresh) {
            traj_has_vel = traj_has_vel || corr_has_vel;
            traj_has_acc = traj_has_acc || corr_has_acc;
        }
    }
    px4_msgs::msg::OffboardControlMode msg{};
    bool is_flip = (sp_type == FLIP);
    bool is_excite = (sp_type == EXCITE);
    bool is_trajectory = (sp_type == TRAJECTORY);
    bool excite_revert = is_excite && (excite_phase_ == ExcitePhase::REVERT);
    // TRAJECTORY raises the position/velocity/acceleration flags group-wise so PX4
    // accepts a mixed (e.g. hold-point + feed-forward velocity) setpoint.
    msg.position = (sp_type == NAVIGATE || sp_type == POSITION || sp_type == NONE) ||
                   (is_trajectory && traj_has_pos) ||
                   (is_flip && flip_phase == FlipPhase::POSITION_HOLD) || excite_revert;
    msg.velocity = (sp_type == VELOCITY) || (is_trajectory && traj_has_vel) ||
                   (is_flip && flip_phase == FlipPhase::CLIMB);
    msg.acceleration = is_trajectory && traj_has_acc;
    msg.attitude = (sp_type == ATTITUDE) || (is_excite && !excite_rate_loop_ && !excite_revert);
    msg.body_rate = (sp_type == RATES) || (is_flip && flip_phase == FlipPhase::ROTATE) ||
                    (is_excite && excite_rate_loop_ && !excite_revert);
    msg.timestamp = getTimestamp();
    offboard_mode_publisher_->publish(msg);
}

void OffboardControlNode::switch_to_offboard_mode(){
    RCLCPP_INFO(this->get_logger(), "Requesting switch to Offboard mode");
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
}

void OffboardControlNode::publish_vehicle_command(uint16_t command, float param1, float param2) {
    auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = getTimestamp();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    request->request = msg;
    service_done_ = false;
    auto result = vehicle_command_client_->async_send_request(request, std::bind(&OffboardControlNode::response_callback, this,
                           std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Sent vehicle command %d", msg.command);
}

bool OffboardControlNode::land() {
    stopOffboardHeartbeat();
    RCLCPP_INFO(this->get_logger(), "Sending land command...");
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    return true;
}

void OffboardControlNode::arm()
{
    RCLCPP_INFO(this->get_logger(), "Arming vehicle...");
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

void OffboardControlNode::disarm()
{
    RCLCPP_INFO(this->get_logger(), "DisArming vehicle...");
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}
#pragma endregion PX4 command

#pragma region Service
std::shared_ptr<SpeedController> OffboardControlNode::start_navigate(
    float x, float y, float z, float yaw, float speed,
    const std::string & frame_id, bool auto_arm) {
    // Frame check
    if (!validate_frame_id(frame_id)) {
        throw std::runtime_error("Invalid frame_id: " + frame_id);
    }
    // Central validation (prompt 99): NaN/Inf patterns + speed limit.
    std::string validation_reason;
    if (!validate_setpoint(NAVIGATE, x, y, z, speed, validation_reason)) {
        throw std::runtime_error(validation_reason);
    }
    Pose current_fcu_pose_ned(snapshot_local_position());
    Pose user_target_enu(x, y, z, yaw);
    std::string effective_frame_id = frame_id;
    Pose effective_target = user_target_enu;

    // Body frame: compute target in map from the same pose snapshot as start to avoid
    // yaw mismatch (TF body orientation vs current_local_position_.heading race).
    // Target yaw in map = current heading (ENU) + user yaw (relative to body).
    if (frame_id == "body") {
        geometry_msgs::msg::Point body_point;
        body_point.x = x;
        body_point.y = y;
        body_point.z = z;
        geometry_msgs::msg::Point map_point;
        if (!transform_point(body_point, "body", map_frame_id_, get_transform_lookup_time(), map_point)) {
            throw std::runtime_error("Failed to transform body target to map frame");
        }
        double current_heading_enu = TransformPX4::nedToEnuYaw(current_fcu_pose_ned.yaw);
        effective_target = Pose(map_point.x, map_point.y, map_point.z, current_heading_enu + yaw);
        effective_frame_id = map_frame_id_;
    }

    // Optional geofence gate on the effective target (no-op unless enabled).
    if (!validate_target_in_geofence(effective_target.x, effective_target.y, effective_target.z,
                                     effective_frame_id, validation_reason)) {
        throw std::runtime_error(validation_reason);
    }

    Init_SpeedController(current_fcu_pose_ned, effective_target, effective_frame_id, speed);
    std::shared_ptr<SpeedController> controller;
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        controller = global_speed_controller_;
    }
    if (!controller) {
        throw std::runtime_error("SpeedController initialization failed (TF error?)");
    }
    RCLCPP_INFO(this->get_logger(), "Starting from FMU NED (%.2f, %.2f, %.2f, yaw=%.2f) to User target in %s frame (%.2f, %.2f, %.2f, yaw=%.2f) at pos_speed %.2f m/s with instant yaw alignment.",
            current_fcu_pose_ned.x, current_fcu_pose_ned.y, current_fcu_pose_ned.z, current_fcu_pose_ned.yaw,
            effective_frame_id.c_str(),
            effective_target.x, effective_target.y, effective_target.z, effective_target.yaw,
            speed);

    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        last_sended_pose = px4_msgs::msg::TrajectorySetpoint();
        setpoint_type_ = NAVIGATE;  // stored last: timer sees type+fields together
    }
    startOffboardHeartbeat();
    wait_armed_ = auto_arm;
    if (auto_arm) {
        if (!offboardAndArm()) {
            throw std::runtime_error("Auto-arm/offboard failed");
        }
        wait_armed_ = false;
    } else {
        const auto status = snapshot_vehicle_status();
        if (status.nav_state !=
            px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
            throw std::runtime_error("Copter is not in OFFBOARD mode, use auto_arm?");
        }
        if (status.arming_state !=
            px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
            throw std::runtime_error("Copter is not armed, use auto_arm?");
        }
    }
    return controller;
}

void OffboardControlNode::navigate_callback(const offboard_interfaces::srv::Navigate::Request::SharedPtr request,
                           const offboard_interfaces::srv::Navigate::Response::SharedPtr response) {
    RCLCPP_INFO(this->get_logger(), "Received navigate request to frame: %s",
               request->frame_id.c_str());
    if (busy_) {
        response->success = false;
        response->message = "Busy";
        RCLCPP_WARN(this->get_logger(), "Navigate failed: %s", response->message.c_str());
        return;
    }
    busy_ = true;
    response->success = false;
    response->message = "";
    try {
        // Thin wrapper over the shared launch helper — unchanged contract.
        start_navigate(request->x, request->y, request->z, request->yaw,
                       request->speed, request->frame_id, request->auto_arm);
        response->success = true;
        response->message = "Navigate started";
    } catch (const std::exception& e) {
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), "Navigate failed: %s", e.what());
        if (setpoint_timer_) {
            setpoint_timer_->cancel();
        }
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            setpoint_type_ = NONE;
            global_speed_controller_.reset();
        }
    }
    busy_ = false;
}

void OffboardControlNode::land_callback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                    const std_srvs::srv::Trigger::Response::SharedPtr response) {
    (void)request;  
    RCLCPP_INFO(this->get_logger(), "Received land request.");
    if (busy_) {
        response->success = false;
        response->message = "Busy";
        RCLCPP_WARN(this->get_logger(), "Land failed: %s", response->message.c_str());
        return;
    }
    busy_ = true;
    response->success = false;
    response->message = "";
    try {
        if (land_only_in_offboard_) {
            const auto nav_state = snapshot_vehicle_status().nav_state;
            const bool in_offboard =
                (nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
            // Additive relaxation (prompt 104): land_from_hold=true also allows
            // landing from hold-like modes; the default gate stays as before.
            const bool in_hold_like = land_from_hold_ &&
                (nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER ||
                 nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL ||
                 nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ALTCTL);
            if (!in_offboard && !in_hold_like) {
                throw std::runtime_error("Copter is not in OFFBOARD mode");
            }
        }
        if (!land()) {
            throw std::runtime_error("Failed to initiate land command");
        }
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            setpoint_type_ = NONE;
            global_speed_controller_.reset();
        }
        if (setpoint_timer_) {
            setpoint_timer_->cancel();
        }

        clear_navigation_targets();
        response->success = true;
        response->message = "Land command sent";
    } catch (const std::exception& e) {
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), "Land failed: %s", e.what());
    }
    busy_ = false;
}

offboard_interfaces::msg::Telemetry OffboardControlNode::build_telemetry(const std::string & requested_frame_id) {
    offboard_interfaces::msg::Telemetry t{};
    auto now = getTimestamp();
    t.stamp = now;

    // Choose frame for telemetry output
    std::string telemetry_frame = requested_frame_id.empty() ? map_frame_id_ : requested_frame_id;
    if (!validate_frame_id(telemetry_frame)) {
        RCLCPP_WARN(this->get_logger(),
                    "Telemetry: invalid frame_id '%s', falling back to '%s'",
                    telemetry_frame.c_str(), map_frame_id_.c_str());
        telemetry_frame = map_frame_id_;
    }

    // One consistent copy of the telemetry snapshot; TF work below runs on it.
    px4_msgs::msg::VehicleStatus vehicle_status;
    uint64_t status_ts, global_ts, battery_ts;
    CurrentPose local_position;
    double lat, lon, alt;
    float voltage;
    float angular_velocity[3];
    {
        std::lock_guard<std::mutex> lk(telemetry_mutex_);
        vehicle_status = current_vehicle_status_;
        status_ts = current_vehicle_status_timestamp_;
        local_position = current_local_position_;
        lat = lat_;
        lon = lon_;
        alt = alt_;
        global_ts = current_global_position_timestamp_;
        voltage = battery_voltage;
        battery_ts = current_battery_timestamp_;
        angular_velocity[0] = current_angular_velocity_[0];
        angular_velocity[1] = current_angular_velocity_[1];
        angular_velocity[2] = current_angular_velocity_[2];
    }

    // State checks
    if (status_ts == 0) {
        t.connected = false;
    } else {
        t.connected = true;
        t.armed = (vehicle_status.arming_state ==
                            px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
        t.mode = nav_state_to_string(vehicle_status.nav_state);
    }

    const bool local_pose_fresh =
        (now - local_position.timestamp) / 1000000.0 <= local_position_timeout_;

    if (local_pose_fresh) {
        // Body pose in requested telemetry_frame
        geometry_msgs::msg::Point body_origin;
        body_origin.x = 0.0;
        body_origin.y = 0.0;
        body_origin.z = 0.0;

        geometry_msgs::msg::Point pose_in_frame;
        rclcpp::Time tf_time = get_transform_lookup_time("body", telemetry_frame);

        if (transform_point(body_origin, "body", telemetry_frame, tf_time, pose_in_frame)) {
            t.x = static_cast<float>(pose_in_frame.x);
            t.y = static_cast<float>(pose_in_frame.y);
            t.z = static_cast<float>(pose_in_frame.z);
        } else {
            // If transform unavailable, return NaN position
            t.x = std::numeric_limits<float>::quiet_NaN();
            t.y = std::numeric_limits<float>::quiet_NaN();
            t.z = std::numeric_limits<float>::quiet_NaN();
        }

        // Drone yaw: body +X orientation in requested frame
        double yaw_in_frame = 0.0;
        if (transform_yaw(0.0, "body", telemetry_frame, tf_time, yaw_in_frame)) {
            // Display-only startup zeroing (zero_heading_at_start, prompt 103):
            // applied to the map-frame output only; frame-relative yaws (aruco_*)
            // and the control-side NED heading stay raw.
            if (zero_heading_at_start_ && telemetry_frame == map_frame_id_) {
                bool heading_set;
                double initial_ned;
                {
                    std::lock_guard<std::mutex> lk(telemetry_mutex_);
                    heading_set = initial_heading_set_;
                    initial_ned = initial_heading_ned_;
                }
                if (heading_set) {
                    const double initial_enu = TransformPX4::nedToEnuYaw(initial_ned);
                    yaw_in_frame = TransformPX4::normalizeYaw(yaw_in_frame - initial_enu);
                }
            }
            t.yaw = static_cast<float>(yaw_in_frame);
        } else {
            t.yaw = std::numeric_limits<float>::quiet_NaN();
        }

        // Roll/pitch still not computed — zeros
        t.roll = 0.0f;
        t.pitch = 0.0f;
    } else {
        t.x = std::numeric_limits<float>::quiet_NaN();
        t.y = std::numeric_limits<float>::quiet_NaN();
        t.z = std::numeric_limits<float>::quiet_NaN();
        t.roll = std::numeric_limits<float>::quiet_NaN();
        t.pitch = std::numeric_limits<float>::quiet_NaN();
        t.yaw = std::numeric_limits<float>::quiet_NaN();
    }

    // Velocity in requested frame (TF map -> telemetry_frame)
    if (local_pose_fresh) {
        CurrentPose enu_vel = TransformPX4::nedToEnuVel(local_position);
        geometry_msgs::msg::Vector3 vel_map;
        vel_map.x = enu_vel.vx;
        vel_map.y = enu_vel.vy;
        vel_map.z = enu_vel.vz;

        geometry_msgs::msg::Vector3 vel_in_frame;
        rclcpp::Time tf_time = get_transform_lookup_time(map_frame_id_, telemetry_frame);
        if (transform_velocity(vel_map, map_frame_id_, telemetry_frame, tf_time, vel_in_frame)) {
            t.vx = static_cast<float>(vel_in_frame.x);
            t.vy = static_cast<float>(vel_in_frame.y);
            t.vz = static_cast<float>(vel_in_frame.z);
        } else {
            t.vx = std::numeric_limits<float>::quiet_NaN();
            t.vy = std::numeric_limits<float>::quiet_NaN();
            t.vz = std::numeric_limits<float>::quiet_NaN();
        }
    } else {
        t.vx = std::numeric_limits<float>::quiet_NaN();
        t.vy = std::numeric_limits<float>::quiet_NaN();
        t.vz = std::numeric_limits<float>::quiet_NaN();
    }
    // Body angular rates (FRD): filled from the gyro subscription when the
    // high-rate DDS profile publishes vehicle_angular_velocity; zeros otherwise.
    t.roll_rate = angular_velocity[0];
    t.pitch_rate = angular_velocity[1];
    t.yaw_rate = angular_velocity[2];
    // Global position
    if ((now - global_ts) / 1000000.0 <= global_position_timeout_) {
        t.lat = lat;
        t.lon = lon;
        t.alt = alt;
    } else {
        t.lat = std::numeric_limits<float>::quiet_NaN();
        t.lon = std::numeric_limits<float>::quiet_NaN();
        t.alt = std::numeric_limits<float>::quiet_NaN();
    }
    // Battery
    if ((now - battery_ts) / 1000000.0 <= battery_timeout_) {
        t.voltage = voltage;
        t.cell_voltage = voltage;
    } else {
        t.voltage = std::numeric_limits<float>::quiet_NaN();
        t.cell_voltage = std::numeric_limits<float>::quiet_NaN();
    }

    // Actual frame for x/y/z/vx/vy/vz/yaw
    t.frame_id = telemetry_frame;
    return t;
}

void OffboardControlNode::get_telemetry_callback(const offboard_interfaces::srv::GetTelemetry::Request::SharedPtr request,
                            const offboard_interfaces::srv::GetTelemetry::Response::SharedPtr response) {
    // Thin wrapper over build_telemetry — same fields, unchanged contract.
    const auto t = build_telemetry(request->frame_id);
    response->frame_id = t.frame_id;
    response->connected = t.connected;
    response->armed = t.armed;
    response->mode = t.mode;
    response->x = t.x;
    response->y = t.y;
    response->z = t.z;
    response->lat = t.lat;
    response->lon = t.lon;
    response->alt = t.alt;
    response->vx = t.vx;
    response->vy = t.vy;
    response->vz = t.vz;
    response->roll = t.roll;
    response->pitch = t.pitch;
    response->yaw = t.yaw;
    response->roll_rate = t.roll_rate;
    response->pitch_rate = t.pitch_rate;
    response->yaw_rate = t.yaw_rate;
    response->voltage = t.voltage;
    response->cell_voltage = t.cell_voltage;
}

void OffboardControlNode::publish_telemetry() {
    telemetry_publisher_->publish(build_telemetry(""));
}

void OffboardControlNode::publish_status() {
    offboard_interfaces::msg::OffboardStatus s{};
    s.stamp = getTimestamp();

    SetpointType type;
    px4_msgs::msg::TrajectorySetpoint last_pose;
    std::string abort_reason;
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        type = setpoint_type_;
        last_pose = last_sended_pose;
        abort_reason = last_abort_reason_;
    }
    uint64_t pos_ts;
    px4_msgs::msg::VehicleStatus status;
    float battery;
    {
        std::lock_guard<std::mutex> lk(telemetry_mutex_);
        pos_ts = current_local_position_.timestamp;
        status = current_vehicle_status_;
        battery = battery_percentage;
    }

    s.setpoint_type = setpoint_type_to_string(type);
    s.busy = busy_.load();
    s.armed = (status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
    s.nav_mode = nav_state_to_string(status.nav_state);
    s.offboard_active =
        (status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
    s.last_target_x = last_pose.position[0];
    s.last_target_y = last_pose.position[1];
    s.last_target_z = last_pose.position[2];
    s.last_target_yaw = last_pose.yaw;
    if (excite_active_.load()) {
        std::lock_guard<std::mutex> lk(excite_mutex_);
        s.active_maneuver = "excitation:" + excite_run_tag_;
    } else {
        switch (type) {
            case NAVIGATE:
            case NAVIGATE_GLOBAL:
                s.active_maneuver = nav_action_active_.load() ? "navigate_to" : "navigate";
                break;
            case POSITION: s.active_maneuver = "position"; break;
            case VELOCITY: s.active_maneuver = "velocity"; break;
            case ATTITUDE: s.active_maneuver = "attitude"; break;
            case RATES:    s.active_maneuver = "rates"; break;
            case FLIP:     s.active_maneuver = "flip"; break;
            default:       s.active_maneuver = ""; break;
        }
    }
    s.last_abort_reason = abort_reason;
    s.telemetry_age_s = (pos_ts == 0)
        ? std::numeric_limits<float>::quiet_NaN()
        : static_cast<float>((getTimestamp() - pos_ts) / 1e6);
    s.battery_pct = battery;

    status_publisher_->publish(s);
}

void OffboardControlNode::apply_telemetry_rate(double hz) {
    telemetry_rate_hz_ = hz;
    if (telemetry_timer_) {
        telemetry_timer_->cancel();
        telemetry_timer_.reset();
    }
    if (hz > 0.0) {
        const std::chrono::nanoseconds period{static_cast<int64_t>(1e9 / hz)};
        telemetry_timer_ = create_wall_timer(
            period, std::bind(&OffboardControlNode::publish_telemetry, this),
            telemetry_callback_group_);
        RCLCPP_INFO(this->get_logger(), "Telemetry topic enabled at %.1f Hz", hz);
    } else {
        RCLCPP_INFO(this->get_logger(), "Telemetry topic disabled (telemetry_rate_hz = 0)");
    }
}

rcl_interfaces::msg::SetParametersResult OffboardControlNode::on_parameters_set(
    const std::vector<rclcpp::Parameter> & parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : parameters) {
        if (p.get_name() == "telemetry_rate_hz") {
            const double hz = p.as_double();
            if (hz < 0.0 || hz > 200.0) {
                result.successful = false;
                result.reason = "telemetry_rate_hz must be within [0, 200]";
                return result;
            }
            apply_telemetry_rate(hz);
        }
    }
    return result;
}

rclcpp_action::GoalResponse OffboardControlNode::handle_telemetry_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const ReceiveTelemetry::Goal> goal)
{
    if (!(goal->rate_hz > 0.0f) || goal->rate_hz > 200.0f) {
        RCLCPP_ERROR(this->get_logger(), "ReceiveTelemetry rejected: rate_hz must be in (0, 200]");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!goal->frame_id.empty() && !validate_frame_id(goal->frame_id)) {
        RCLCPP_ERROR(this->get_logger(), "ReceiveTelemetry rejected: invalid frame_id '%s'",
                     goal->frame_id.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (telemetry_stream_active_.load()) {
        RCLCPP_WARN(this->get_logger(), "ReceiveTelemetry rejected: a stream is already running");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OffboardControlNode::handle_telemetry_cancel(
    const std::shared_ptr<TelemetryGoalHandle> /*goal_handle*/)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void OffboardControlNode::handle_telemetry_accepted(const std::shared_ptr<TelemetryGoalHandle> goal_handle)
{
    if (telemetry_stream_thread_.joinable()) {
        telemetry_stream_thread_.join();  // reap the previous (finished) stream
    }
    telemetry_stream_active_ = true;
    telemetry_stream_thread_ = std::thread(
        std::bind(&OffboardControlNode::execute_telemetry_stream, this, goal_handle));
}

void OffboardControlNode::execute_telemetry_stream(const std::shared_ptr<TelemetryGoalHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ReceiveTelemetry::Feedback>();
    auto result = std::make_shared<ReceiveTelemetry::Result>();
    rclcpp::Rate rate(static_cast<double>(goal->rate_hz));
    const auto start = this->now();
    uint32_t sent = 0;
    bool canceled = false;

    while (rclcpp::ok() && !telemetry_stream_stop_.load()) {
        if (goal_handle->is_canceling()) {
            canceled = true;
            break;
        }
        feedback->telemetry = build_telemetry(goal->frame_id);
        goal_handle->publish_feedback(feedback);
        ++sent;
        if (goal->duration > 0.0f && (this->now() - start).seconds() >= goal->duration) {
            break;
        }
        rate.sleep();
    }

    result->samples_sent = sent;
    result->success = !canceled;
    result->message = canceled ? "Stream canceled" : "Stream finished";
    telemetry_stream_active_ = false;
    if (canceled) {
        goal_handle->canceled(result);
    } else {
        goal_handle->succeed(result);
    }
}

void OffboardControlNode::set_altitude_callback(const offboard_interfaces::srv::SetAltitude::Request::SharedPtr request,
                                                const offboard_interfaces::srv::SetAltitude::Response::SharedPtr response) {
    try {
        if (!validate_frame_id(request->frame_id)) {
            response->success = false;
            response->message = "Invalid frame_id: " + request->frame_id;
            return;
        }
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            altitude_override_z_ = static_cast<double>(request->z);
            altitude_override_frame_id_ = request->frame_id;
            altitude_override_set_ = true;
        }
        response->success = true;
        response->message = "Altitude override set";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}

void OffboardControlNode::set_yaw_callback(const offboard_interfaces::srv::SetYaw::Request::SharedPtr request,
                                           const offboard_interfaces::srv::SetYaw::Response::SharedPtr response) {
    try {
        if (std::isnan(request->yaw)) {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            yaw_override_set_ = false;
            yaw_rate_override_set_ = false;
            response->success = true;
            response->message = "Yaw and yaw rate override cleared";
            return;
        }
        if (!validate_frame_id(request->frame_id)) {
            response->success = false;
            response->message = "Invalid frame_id: " + request->frame_id;
            return;
        }
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            yaw_override_ = static_cast<double>(request->yaw);
            yaw_override_frame_id_ = request->frame_id;
            yaw_override_set_ = true;
            yaw_rate_override_set_ = false;
        }
        response->success = true;
        response->message = "Yaw override set";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}

void OffboardControlNode::set_yaw_rate_callback(const offboard_interfaces::srv::SetYawRate::Request::SharedPtr request,
                                                const offboard_interfaces::srv::SetYawRate::Response::SharedPtr response) {
    try {
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            yaw_rate_override_ = static_cast<double>(request->yaw_rate);
            yaw_rate_override_set_ = true;
            yaw_override_set_ = false;
        }
        response->success = true;
        response->message = "Yaw rate override set";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}

void OffboardControlNode::set_position_callback(const offboard_interfaces::srv::SetPosition::Request::SharedPtr request,
                                                const offboard_interfaces::srv::SetPosition::Response::SharedPtr response) {
    try {
        std::string frame_id = request->frame_id.empty() ? map_frame_id_ : request->frame_id;
        if (!validate_frame_id(frame_id)) {
            response->success = false;
            response->message = "Invalid frame_id: " + frame_id;
            return;
        }
        std::string validation_reason;
        if (!validate_setpoint(POSITION, request->x, request->y, request->z, 0.0, validation_reason) ||
            !validate_target_in_geofence(request->x, request->y, request->z, frame_id, validation_reason)) {
            response->success = false;
            response->message = validation_reason;
            RCLCPP_ERROR(this->get_logger(), "SetPosition rejected: %s", validation_reason.c_str());
            return;
        }
        if (request->auto_arm && !offboardAndArm()) {
            response->success = false;
            response->message = "Auto-arm/offboard failed";
            return;
        }
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            position_target_x_ = std::isfinite(request->x) ? request->x : 0.0;
            position_target_y_ = std::isfinite(request->y) ? request->y : 0.0;
            position_target_z_ = std::isfinite(request->z) ? request->z : 0.0;
            position_target_yaw_ = std::isfinite(request->yaw) ? request->yaw : 0.0;
            position_target_frame_id_ = frame_id;
            setpoint_type_ = POSITION;  // stored last under the lock
        }
        startOffboardHeartbeat();
        response->success = true;
        response->message = "Set position started";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}

void OffboardControlNode::set_trajectory_callback(const offboard_interfaces::srv::SetTrajectory::Request::SharedPtr request,
                                                  const offboard_interfaces::srv::SetTrajectory::Response::SharedPtr response) {
    try {
        std::string frame_id = request->frame_id.empty() ? map_frame_id_ : request->frame_id;
        if (!validate_frame_id(frame_id)) {
            response->success = false;
            response->message = "Invalid frame_id: " + frame_id;
            return;
        }

        std::array<double, 3> position{request->x, request->y, request->z};
        std::array<double, 3> velocity{request->vx, request->vy, request->vz};
        std::array<double, 3> acceleration{request->ax, request->ay, request->az};
        // Each group must be fully specified or fully disabled (all finite / all NaN).
        const bool valid_position = all_finite_3(position) || all_nan_3(position);
        const bool valid_velocity = all_finite_3(velocity) || all_nan_3(velocity);
        const bool valid_acceleration = all_finite_3(acceleration) || all_nan_3(acceleration);
        if (!valid_position || !valid_velocity || !valid_acceleration) {
            response->success = false;
            response->message = "Each of position/velocity/acceleration must be all finite or all NaN";
            RCLCPP_ERROR(this->get_logger(), "SetTrajectory rejected: mixed finite/NaN in a group");
            return;
        }
        if (!any_finite_3(position) && !any_finite_3(velocity) && !any_finite_3(acceleration)) {
            response->success = false;
            response->message = "At least one of position/velocity/acceleration must be set";
            RCLCPP_ERROR(this->get_logger(), "SetTrajectory rejected: all groups disabled");
            return;
        }
        // Geofence gate only when a concrete position target is given.
        if (all_finite_3(position)) {
            std::string validation_reason;
            if (!validate_target_in_geofence(position[0], position[1], position[2], frame_id, validation_reason)) {
                response->success = false;
                response->message = validation_reason;
                RCLCPP_ERROR(this->get_logger(), "SetTrajectory rejected: %s", validation_reason.c_str());
                return;
            }
        }

        if (request->auto_arm && !offboardAndArm()) {
            response->success = false;
            response->message = "Auto-arm/offboard failed";
            return;
        }
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            trajectory_position_ = position;
            trajectory_velocity_ = velocity;
            trajectory_acceleration_ = acceleration;
            trajectory_yaw_ = request->yaw;
            trajectory_yaw_rate_ = request->yaw_rate;
            trajectory_frame_id_ = frame_id;
            setpoint_type_ = TRAJECTORY;  // stored last under the lock
        }
        startOffboardHeartbeat();
        response->success = true;
        response->message = "Set trajectory started";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}

void OffboardControlNode::set_velocity_callback(const offboard_interfaces::srv::SetVelocity::Request::SharedPtr request,
                                                const offboard_interfaces::srv::SetVelocity::Response::SharedPtr response) {
    try {
        std::string frame_id = request->frame_id.empty() ? map_frame_id_ : request->frame_id;
        if (!validate_frame_id(frame_id)) {
            response->success = false;
            response->message = "Invalid frame_id: " + frame_id;
            return;
        }
        std::string validation_reason;
        if (!validate_setpoint(VELOCITY, request->vx, request->vy, request->vz, 0.0, validation_reason)) {
            response->success = false;
            response->message = validation_reason;
            RCLCPP_ERROR(this->get_logger(), "SetVelocity rejected: %s", validation_reason.c_str());
            return;
        }
        if (request->auto_arm && !offboardAndArm()) {
            response->success = false;
            response->message = "Auto-arm/offboard failed";
            return;
        }
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            velocity_target_vx_ = request->vx;
            velocity_target_vy_ = request->vy;
            velocity_target_vz_ = request->vz;
            velocity_target_yaw_ = std::isfinite(request->yaw) ? request->yaw : 0.0;
            velocity_target_frame_id_ = frame_id;
            setpoint_type_ = VELOCITY;  // stored last under the lock
        }
        startOffboardHeartbeat();
        response->success = true;
        response->message = "Set velocity started";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}

void OffboardControlNode::set_attitude_callback(const offboard_interfaces::srv::SetAttitude::Request::SharedPtr request,
                                                const offboard_interfaces::srv::SetAttitude::Response::SharedPtr response) {
    try {
        std::string frame_id = request->frame_id.empty() ? map_frame_id_ : request->frame_id;
        if (!validate_frame_id(frame_id)) {
            response->success = false;
            response->message = "Invalid frame_id: " + frame_id;
            return;
        }
        if (request->auto_arm && !offboardAndArm()) {
            response->success = false;
            response->message = "Auto-arm/offboard failed";
            return;
        }
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            attitude_roll_ = request->roll;
            attitude_pitch_ = request->pitch;
            attitude_yaw_ = request->yaw;
            attitude_thrust_ = std::max(0.0f, std::min(1.0f, request->thrust));
            attitude_frame_id_ = frame_id;
            setpoint_type_ = ATTITUDE;  // stored last under the lock
        }
        startOffboardHeartbeat();
        response->success = true;
        response->message = "Set attitude started";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}

void OffboardControlNode::set_rates_callback(const offboard_interfaces::srv::SetRates::Request::SharedPtr request,
                                             const offboard_interfaces::srv::SetRates::Response::SharedPtr response) {
    try {
        if (request->auto_arm && !offboardAndArm()) {
            response->success = false;
            response->message = "Auto-arm/offboard failed";
            return;
        }
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            rates_roll_ = request->roll_rate;
            rates_pitch_ = request->pitch_rate;
            rates_yaw_ = request->yaw_rate;
            rates_thrust_ = std::max(0.0f, std::min(1.0f, request->thrust));
            setpoint_type_ = RATES;  // stored last under the lock
        }
        startOffboardHeartbeat();
        response->success = true;
        response->message = "Set rates started";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
    }
}

void OffboardControlNode::flip_callback(const offboard_interfaces::srv::Flip::Request::SharedPtr request,
                                        const offboard_interfaces::srv::Flip::Response::SharedPtr response) {
    RCLCPP_INFO(this->get_logger(), "Received flip request, axis: %s", request->axis.c_str());
    if (busy_) {
        response->success = false;
        response->message = "Busy";
        return;
    }
    busy_ = true;
    response->success = false;
    response->message = "";
    try {
        if (request->axis == "roll") {
            flip_axis_ = FlipAxis::ROLL;
        } else if (request->axis == "pitch") {
            flip_axis_ = FlipAxis::PITCH;
        } else if (request->axis == "yaw") {
            flip_axis_ = FlipAxis::YAW;
        } else {
            throw std::runtime_error("axis must be 'roll', 'pitch' or 'yaw', got: " + request->axis);
        }
        if (request->climb_duration < 0.0f || request->rate == 0.0f || request->target_angle <= 0.0f) {
            throw std::runtime_error("climb_duration must be >= 0, rate != 0, target_angle > 0");
        }
        if (request->auto_arm && !offboardAndArm()) {
            throw std::runtime_error("Auto-arm/offboard failed");
        }
        if (!request->auto_arm) {
            const auto status = snapshot_vehicle_status();
            if (status.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                throw std::runtime_error("Copter is not in OFFBOARD mode, use auto_arm?");
            }
            if (status.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
                throw std::runtime_error("Copter is not armed, use auto_arm?");
            }
        }
        const Pose flip_start_pose(snapshot_local_position());
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            flip_initial_pose_ned_ = flip_start_pose;
            flip_vz_ = request->vz;
            flip_climb_duration_ = request->climb_duration;
            flip_rate_ = request->rate;
            flip_target_angle_ = static_cast<double>(request->target_angle);
            flip_thrust_ = std::max(0.0f, std::min(1.0f, request->thrust));
            flip_phase_ = FlipPhase::CLIMB;
            flip_climb_start_time_ = this->now();
            flip_angle_done_ = 0.0;
            setpoint_type_ = FLIP;  // stored last under the lock
        }
        startOffboardHeartbeat();
        response->success = true;
        response->message = "Flip started";
        RCLCPP_INFO(this->get_logger(), "Flip: initial NED (%.2f, %.2f, %.2f), axis=%s, rate=%.2f, target=%.2f rad",
                    flip_initial_pose_ned_.x, flip_initial_pose_ned_.y, flip_initial_pose_ned_.z,
                    request->axis.c_str(), flip_rate_, flip_target_angle_);
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), "Flip failed: %s", e.what());
    }
    busy_ = false;
}

void OffboardControlNode::return_to_launch_callback(
    const offboard_interfaces::srv::ReturnToLaunch::Request::SharedPtr request,
    const offboard_interfaces::srv::ReturnToLaunch::Response::SharedPtr response) {
    (void)request;
    RCLCPP_INFO(this->get_logger(), "Received return_to_launch request.");
    try {
        // Cleanly stop whatever maneuver is running before handing the drone
        // to the PX4 RTL navigator (same pattern as land()).
        if (excite_active_.load()) {
            request_excite_abort("return_to_launch", true);
            const auto wait_start = this->now();
            while (rclcpp::ok() && excite_active_.load() &&
                   (this->now() - wait_start).seconds() < 2.0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
        stopOffboardHeartbeat();
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
        clear_navigation_targets();
        busy_ = false;
        response->success = true;
        response->message = "Return-to-launch command sent";
    } catch (const std::exception & e) {
        response->success = false;
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), "Return-to-launch failed: %s", e.what());
    }
}

void OffboardControlNode::emergency_stop_callback(
    const offboard_interfaces::srv::EmergencyStop::Request::SharedPtr request,
    const offboard_interfaces::srv::EmergencyStop::Response::SharedPtr response) {
    RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP requested (land=%d)", request->land);

    // 1. Raise the global abort flag and wake anyone parked in offboardAndArm.
    command_abort_ = true;
    {
        std::lock_guard<std::mutex> lk(arm_wait_mutex_);
    }
    arm_wait_cv_.notify_all();

    // 2. An active excitation aborts through its OWN mechanism (immediate
    //    revert); wait for its executor to finish so we don't fight its
    //    phase machine over setpoint_type_.
    if (excite_active_.load()) {
        request_excite_abort("emergency_stop", true);
        const auto wait_start = this->now();
        while (rclcpp::ok() && excite_active_.load() &&
               (this->now() - wait_start).seconds() < 2.0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    // 3. Park or land. NavigateTo workers notice the maneuver switch themselves.
    try {
        if (request->land) {
            land();  // stops the offboard stream and sends VEHICLE_CMD_NAV_LAND
            clear_navigation_targets();
            busy_ = false;
            response->success = true;
            response->message = "Emergency stop: land initiated";
        } else {
            park_to_hold("emergency_stop");
            response->success = true;
            response->message = "Emergency stop: parked to hold";
        }
    } catch (const std::exception & e) {
        response->success = false;
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), "Emergency stop failed: %s", e.what());
    }
    command_abort_ = false;
}

#pragma endregion Service

#pragma region NavigateToAction
rclcpp_action::GoalResponse OffboardControlNode::handle_navigate_to_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const NavigateTo::Goal> goal)
{
    if (busy_ || nav_action_active_.load() || excite_active_.load()) {
        RCLCPP_WARN(this->get_logger(), "NavigateTo rejected: node busy");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!validate_frame_id(goal->frame_id)) {
        RCLCPP_ERROR(this->get_logger(), "NavigateTo rejected: invalid frame_id '%s'",
                     goal->frame_id.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OffboardControlNode::handle_navigate_to_cancel(
    const std::shared_ptr<NavigateToGoalHandle> /*goal_handle*/)
{
    RCLCPP_WARN(this->get_logger(), "NavigateTo cancel requested -> park to hold");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void OffboardControlNode::handle_navigate_to_accepted(const std::shared_ptr<NavigateToGoalHandle> goal_handle)
{
    if (nav_action_thread_.joinable()) {
        nav_action_thread_.join();  // reap the previous (finished) run
    }
    nav_action_active_ = true;
    nav_action_thread_ = std::thread(
        std::bind(&OffboardControlNode::execute_navigate_to, this, goal_handle));
}

void OffboardControlNode::execute_navigate_to(const std::shared_ptr<NavigateToGoalHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<NavigateTo::Feedback>();
    auto result = std::make_shared<NavigateTo::Result>();

    // Launch phase (mirrors the navigate service, busy_ held like the service does).
    std::shared_ptr<SpeedController> controller;
    busy_ = true;
    try {
        feedback->phase = "arming";
        feedback->distance_remaining = std::numeric_limits<float>::quiet_NaN();
        feedback->eta = std::numeric_limits<float>::quiet_NaN();
        goal_handle->publish_feedback(feedback);
        controller = start_navigate(goal->x, goal->y, goal->z, goal->yaw,
                                    goal->speed, goal->frame_id, goal->auto_arm);
    } catch (const std::exception & e) {
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            setpoint_type_ = NONE;
            global_speed_controller_.reset();
        }
        busy_ = false;
        nav_action_active_ = false;
        result->success = false;
        result->message = e.what();
        result->final_distance_error = std::numeric_limits<float>::quiet_NaN();
        RCLCPP_ERROR(this->get_logger(), "NavigateTo failed to start: %s", e.what());
        goal_handle->abort(result);
        return;
    }
    busy_ = false;  // same semantics as the service: busy only during launch

    const Pose target_ned = controller->targetPoseNED();
    const double speed = controller->targetSpeed();
    bool canceled = false;
    bool completed_by_timer = false;
    bool arrived_by_tolerance = false;
    bool timed_out = false;

    // Deadline for a physically unreachable target (wind, obstacle, EKF drift):
    // goal.timeout > 0 — explicit; == 0 — auto from the nominal flight time;
    // < 0 — no limit (wait until cancel).
    const double initial_distance =
        TransformPX4::getDistance(Pose(snapshot_local_position()), target_ned);
    double timeout_s = -1.0;
    if (goal->timeout > 0.0f) {
        timeout_s = goal->timeout;
    } else if (goal->timeout == 0.0f) {
        timeout_s = (speed > 0.0 ? 3.0 * initial_distance / speed : 0.0) + 10.0;
    }
    const auto flight_start = this->now();

    // Parks only if this maneuver is still the active one (never stomps a
    // command that superseded us). Returns false when superseded.
    auto park_if_still_ours = [&](const std::string & reason) -> bool {
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            if (!(setpoint_type_ == NAVIGATE && global_speed_controller_ == controller)) {
                return false;
            }
            global_speed_controller_.reset();
        }
        park_to_hold(reason);
        return true;
    };

    rclcpp::Rate loop(5.0);
    while (rclcpp::ok() && !nav_action_stop_.load()) {
        if (goal_handle->is_canceling()) {
            canceled = true;
            // Soft stop: hold the current pose; result reports failure.
            park_if_still_ours("navigate_to_cancel");
            break;
        }

        const Pose current_ned(snapshot_local_position());
        const double distance = TransformPX4::getDistance(current_ned, target_ned);

        // The timer detects arrival (isEnd -> NONE + controller reset); watch for it.
        bool still_ours;
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            still_ours = (setpoint_type_ == NAVIGATE && global_speed_controller_ == controller);
            if (!still_ours) {
                // NONE with no controller = the timer completed this navigation;
                // anything else = superseded by another command.
                completed_by_timer = (setpoint_type_ == NONE && !global_speed_controller_);
            }
        }
        if (!still_ours) {
            break;
        }

        // Relaxed arrival radius: report success now; the flight itself keeps
        // converging to the fine target (the drone ends up holding there).
        if (goal->tolerance > 0.0f && distance <= goal->tolerance) {
            arrived_by_tolerance = true;
            break;
        }

        // Timeout: the drone could not reach the target — park it and fail the goal.
        if (timeout_s > 0.0 && (this->now() - flight_start).seconds() >= timeout_s) {
            timed_out = true;
            park_if_still_ours("navigate_to_timeout");
            break;
        }

        feedback->phase = "moving";
        feedback->distance_remaining = static_cast<float>(distance);
        feedback->eta = (speed > 0.0)
            ? static_cast<float>(distance / speed)
            : std::numeric_limits<float>::quiet_NaN();
        goal_handle->publish_feedback(feedback);
        loop.sleep();
    }

    const Pose final_ned(snapshot_local_position());
    const double final_err = TransformPX4::getDistance(final_ned, target_ned);
    result->final_distance_error = static_cast<float>(final_err);
    nav_action_active_ = false;

    if (canceled) {
        result->success = false;
        result->message = "Canceled: parked to hold";
        goal_handle->canceled(result);
        return;
    }
    if (timed_out) {
        result->success = false;
        result->message = "Timeout after " + std::to_string(timeout_s) +
                          " s: target not reached, parked to hold";
        goal_handle->abort(result);
        RCLCPP_WARN(this->get_logger(), "NavigateTo timeout: err=%.2f m after %.1f s",
                    final_err, timeout_s);
        return;
    }
    // Arrived when the timer completed this navigation (its isEnd fired) or the
    // relaxed tolerance radius was reached — a later re-check against a fresh
    // pose would flake on hover jitter right at the tolerance boundary. The
    // tolerance check is only a fallback for exits not driven by the timer
    // (e.g. node shutdown).
    const bool arrived = completed_by_timer || arrived_by_tolerance || controller->isEnd(final_ned);
    feedback->phase = "arrived";
    feedback->distance_remaining = static_cast<float>(final_err);
    feedback->eta = 0.0f;
    goal_handle->publish_feedback(feedback);
    result->success = arrived;
    result->message = arrived
        ? (arrived_by_tolerance ? "Arrived (within tolerance)" : "Arrived")
        : "Maneuver ended before arrival (superseded or aborted)";
    if (arrived) {
        goal_handle->succeed(result);
    } else {
        goal_handle->abort(result);
    }
    RCLCPP_INFO(this->get_logger(), "NavigateTo finished: success=%d err=%.3f m",
                result->success, final_err);
}
#pragma endregion NavigateToAction

#pragma region Excitation
// Autonomous excitation executor for closed-loop system ID (-74).
// SAFETY: this actively perturbs a flying drone. Pattern A (attitude) is the
// default; Pattern B (rate) is gated behind allow_rate_loop_excitation and does
// NOT hold horizontal position (relies on the geofence/drift abort). Test in
// SITL, then on a tethered stand, then indoors with a geofence. Required PX4
// params for RC-abort: COM_RC_OVERRIDE=3, COM_OBL_RC_ACT in {0,5}, COM_OF_LOSS_T=1.

namespace {
bool parse_waveform(const std::string & s, excitation::Waveform & w) {
    if (s == "step")      { w = excitation::Waveform::Step; return true; }
    if (s == "doublet")   { w = excitation::Waveform::Doublet; return true; }
    if (s == "3211")      { w = excitation::Waveform::ThreeTwoOneOne; return true; }
    if (s == "chirp")     { w = excitation::Waveform::Chirp; return true; }
    if (s == "multisine") { w = excitation::Waveform::Multisine; return true; }
    if (s == "prbs")      { w = excitation::Waveform::Prbs; return true; }
    return false;
}
// Small body-frame rotation about the selected axis by `angle` rad.
Eigen::Quaterniond axis_delta_quat(FlipAxis axis, double angle) {
    double r = (axis == FlipAxis::ROLL)  ? angle : 0.0;
    double p = (axis == FlipAxis::PITCH) ? angle : 0.0;
    double y = (axis == FlipAxis::YAW)   ? angle : 0.0;
    return px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(r, p, y);
}
// Total tilt: angle between the body-down axis and world-down (NED).
double tilt_from_quat(const Eigen::Quaterniond & q) {
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    double c = std::max(-1.0, std::min(1.0, R(2, 2)));
    return std::acos(c);
}
constexpr double kExciteHardRateCap  = 0.5;     // rad/s injected rate cap (§4)
constexpr double kExciteHardAngleCap = 0.1745;  // rad (~10 deg) attitude perturbation cap
constexpr double kExciteRunawayRate  = 3.0;     // rad/s measured → tumble abort
const char kExciteSafetyPhrase[] = "I_UNDERSTAND_WHAT_I_AM_DOING";
}  // namespace

rclcpp_action::GoalResponse OffboardControlNode::handle_excite_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const RunExcitation::Goal> goal)
{
    if (excite_active_.load() || busy_) {
        RCLCPP_WARN(this->get_logger(), "Excitation rejected: node busy");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->confirm != kExciteSafetyPhrase) {
        RCLCPP_ERROR(this->get_logger(), "Excitation rejected: safety phrase 'confirm' does not match");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->axis != "roll" && goal->axis != "pitch" && goal->axis != "yaw") {
        RCLCPP_ERROR(this->get_logger(), "Excitation rejected: axis must be roll/pitch/yaw");
        return rclcpp_action::GoalResponse::REJECT;
    }
    excitation::Waveform wf;
    if (!parse_waveform(goal->signal, wf)) {
        RCLCPP_ERROR(this->get_logger(), "Excitation rejected: unknown signal '%s'", goal->signal.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    const bool rate_loop = (goal->loop == RunExcitation::Goal::LOOP_RATE);
    if (rate_loop && !allow_rate_loop_excitation_) {
        RCLCPP_ERROR(this->get_logger(),
            "Excitation rejected: rate-loop (Pattern B) disabled; set allow_rate_loop_excitation=true");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->amplitude <= 0.0f) {
        RCLCPP_ERROR(this->get_logger(), "Excitation rejected: amplitude must be > 0");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (rate_loop && goal->amplitude > kExciteHardRateCap) {
        RCLCPP_ERROR(this->get_logger(), "Excitation rejected: rate amplitude %.3f > %.3f rad/s cap",
                     goal->amplitude, kExciteHardRateCap);
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!rate_loop && goal->amplitude > kExciteHardAngleCap) {
        RCLCPP_ERROR(this->get_logger(), "Excitation rejected: angle amplitude %.3f > %.3f rad cap",
                     goal->amplitude, kExciteHardAngleCap);
        return rclcpp_action::GoalResponse::REJECT;
    }
    const double max_tilt_deg = (goal->max_tilt_deg > 0.0f) ? goal->max_tilt_deg : 20.0;
    if (max_tilt_deg > 25.0) {
        RCLCPP_ERROR(this->get_logger(), "Excitation rejected: max_tilt_deg %.1f > 25 hard cap", max_tilt_deg);
        return rclcpp_action::GoalResponse::REJECT;
    }
    const auto status = snapshot_vehicle_status();
    if (!goal->auto_arm &&
        (status.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD ||
         status.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)) {
        RCLCPP_ERROR(this->get_logger(), "Excitation rejected: not armed+OFFBOARD (use auto_arm)");
        return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Excitation goal accepted: axis=%s signal=%s loop=%s amp=%.3f",
                goal->axis.c_str(), goal->signal.c_str(), rate_loop ? "rate" : "attitude", goal->amplitude);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OffboardControlNode::handle_excite_cancel(
    const std::shared_ptr<ExciteGoalHandle> /*goal_handle*/)
{
    RCLCPP_WARN(this->get_logger(), "Excitation cancel requested → revert");
    excite_cancel_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void OffboardControlNode::handle_excite_accepted(const std::shared_ptr<ExciteGoalHandle> goal_handle)
{
    if (excite_thread_.joinable()) {
        excite_thread_.join();  // reap the previous (already-finished) run
    }
    excite_active_ = true;
    busy_ = true;
    excite_thread_ = std::thread(
        std::bind(&OffboardControlNode::execute_excitation, this, goal_handle));
}

void OffboardControlNode::execute_excitation(const std::shared_ptr<ExciteGoalHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<RunExcitation::Result>();

    // Reset flags + metrics.
    excite_ready_.store(false, std::memory_order_release);  // gate closed until state is built
    excite_abort_ = false;
    excite_emergency_ = false;
    excite_cancel_ = false;
    excite_soft_stop_ = false;
    excite_finished_ = false;
    {
        std::lock_guard<std::mutex> lk(excite_mutex_);
        excite_abort_reason_.clear();
        excite_max_tilt_seen_deg_ = 0.0;
        excite_max_drift_m_ = 0.0;
        excite_samples_injected_ = 0;
        excite_achieved_rate_hz_ = 0.0;
        excite_fb_elapsed_ = 0.0;
        excite_fb_progress_ = 0.0;
        excite_fb_phase_ = "capture";
        excite_fb_ref_ = 0.0;
        excite_fb_rate_ = 0.0;
        excite_fb_tilt_ = 0.0;
        excite_fb_drift_ = 0.0;
    }

    // 1. Arm / OFFBOARD if requested.
    const auto arm_status = snapshot_vehicle_status();
    if (goal->auto_arm &&
        (arm_status.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD ||
         arm_status.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)) {
        if (!offboardAndArm()) {
            result->success = false;
            result->message = "Auto-arm/offboard failed";
            result->abort_reason = "arm";
            excite_active_ = false;
            busy_ = false;
            goal_handle->abort(result);
            return;
        }
    }

    // 2. Build the signal and capture the hover baseline.
    excitation::Waveform wf;
    parse_waveform(goal->signal, wf);
    excitation::ExcitationParams p;
    p.waveform = wf;
    p.amplitude = goal->amplitude;
    p.duration = goal->duration;
    p.f_min = (goal->f_min > 0.0f) ? goal->f_min : 0.5;
    p.f_max = (goal->f_max > 0.0f) ? goal->f_max : 10.0;
    p.base_period = goal->base_period;
    p.seed = (goal->seed != 0u) ? goal->seed : 1u;
    p.taper = (goal->taper > 0.0f) ? goal->taper : 0.15;
    excite_signal_ = std::make_unique<excitation::ExcitationSignal>(p);

    excite_axis_ = (goal->axis == "roll") ? FlipAxis::ROLL
                 : (goal->axis == "pitch") ? FlipAxis::PITCH
                 : FlipAxis::YAW;
    excite_rate_loop_ = (goal->loop == RunExcitation::Goal::LOOP_RATE);
    excite_amplitude_ = goal->amplitude;
    excite_max_tilt_rad_ = ((goal->max_tilt_deg > 0.0f) ? goal->max_tilt_deg : 20.0) * M_PI / 180.0;
    excite_geofence_radius_ = (goal->geofence_radius > 0.0f)
        ? goal->geofence_radius : excite_geofence_radius_default_;
    // One-time capture from the execute thread, consistent under telemetry_mutex_.
    const CurrentPose start_position = snapshot_local_position();
    excite_initial_pose_ned_ = Pose(start_position);
    excite_start_yaw_ned_ = start_position.heading;
    const double current_agl = -start_position.z;
    excite_hold_alt_m_ = (goal->hold_alt > 0.0f) ? goal->hold_alt : current_agl;
    if (goal->hold_alt > 0.0f) {
        // P0-3: an absolute hold altitude on top of a biased EKF (SITL/indoor baro
        // drift) sends the drone to the wrong height — it once flew a chirp into the
        // ground. hold_alt=0 means "hold the altitude captured at target start".
        RCLCPP_WARN(this->get_logger(),
            "Excitation hold_alt=%.2f is ABSOLUTE AGL — on a biased estimator this is "
            "unsafe; prefer hold_alt=0 (hold current altitude at start).", goal->hold_alt);
    }
    // P0-1: enter the baseline at PX4's estimated hover thrust, not the fixed param —
    // a stale 0.5 while the real hover is ~0.7 drops the drone on Pattern B entry.
    {
        std::lock_guard<std::mutex> lk(telemetry_mutex_);
        const double est_age_s = current_hover_thrust_timestamp_ > 0
            ? (getTimestamp() - current_hover_thrust_timestamp_) / 1e6 : 1e9;
        if (current_hover_thrust_valid_ && est_age_s < 2.0 &&
            current_hover_thrust_ >= 0.1 && current_hover_thrust_ <= 0.9) {
            excite_hover_thrust_used_ = current_hover_thrust_;
        } else {
            excite_hover_thrust_used_ = excite_hover_thrust_;  // param fallback
        }
    }
    RCLCPP_INFO(this->get_logger(), "Excitation hover thrust: %.3f (%s)",
        excite_hover_thrust_used_,
        (excite_hover_thrust_used_ == excite_hover_thrust_) ? "param fallback" : "PX4 estimate");
    excite_washout_state_ = 0.0;
    excite_signal_str_ = goal->signal;
    excite_run_tag_ = goal->axis + "_" + goal->signal + "_" + std::to_string(getTimestamp());

    // 3. Raise the setpoint stream and enter the EXCITE phase machine.
    excite_prev_control_rate_ = control_rate_;
    const double hz = std::max(100.0, std::min(excite_stream_hz_, 250.0));  // >=100 Hz floor (rate-loop bandwidth)
    control_rate_ = hz;
    excite_session_start_ = this->now();
    excite_phase_ = ExcitePhase::SETTLE_IN;
    excite_phase_start_ = excite_session_start_;
    // Publish (release) after every EXCITE field above is written, so the timer's
    // acquire-load in publish_excite_setpoint sees a fully-built excite_signal_.
    excite_ready_.store(true, std::memory_order_release);
    {
        // Only the type handoff goes under cmd_mutex_; EXCITE runtime state keeps
        // its own synchronization (excite_ready_/excite_mutex_) — see lock order doc.
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        setpoint_type_ = EXCITE;
    }
    // Re-create the timer at the higher rate. Same cross-thread Init_timer() pattern
    // the FLIP path already uses (service thread vs timer thread under the
    // MultiThreadedExecutor); control_rate_ is atomic to keep the period read clean.
    Init_timer();

    RCLCPP_INFO(this->get_logger(),
        "Excitation start: tag=%s axis=%s signal=%s loop=%s dur=%.2fs stream=%.0fHz geofence=%.1fm",
        excite_run_tag_.c_str(), goal->axis.c_str(), goal->signal.c_str(),
        excite_rate_loop_ ? "rate" : "attitude",
        excite_signal_->duration(), hz, excite_geofence_radius_);

    // 4. Publish feedback until the timer-side phase machine signals completion.
    rclcpp::Rate loop(20.0);
    auto feedback = std::make_shared<RunExcitation::Feedback>();
    while (rclcpp::ok() && !excite_finished_.load()) {
        if (goal_handle->is_canceling()) {
            excite_cancel_ = true;
        }
        {
            std::lock_guard<std::mutex> lk(excite_mutex_);
            feedback->elapsed = static_cast<float>(excite_fb_elapsed_);
            feedback->progress = static_cast<float>(excite_fb_progress_);
            feedback->phase = excite_fb_phase_;
            feedback->current_ref = static_cast<float>(excite_fb_ref_);
            feedback->current_rate = static_cast<float>(excite_fb_rate_);
            feedback->current_tilt_deg = static_cast<float>(excite_fb_tilt_);
            feedback->current_drift_m = static_cast<float>(excite_fb_drift_);
        }
        goal_handle->publish_feedback(feedback);
        loop.sleep();
    }

    // 5. Restore the nominal stream rate.
    control_rate_ = excite_prev_control_rate_;
    Init_timer();

    // 6. Compose the result.
    std::string reason;
    {
        std::lock_guard<std::mutex> lk(excite_mutex_);
        reason = excite_abort_reason_;
        result->samples_injected = excite_samples_injected_;
        result->achieved_rate_hz = static_cast<float>(excite_achieved_rate_hz_);
        result->max_tilt_seen_deg = static_cast<float>(excite_max_tilt_seen_deg_);
        result->max_drift_m = static_cast<float>(excite_max_drift_m_);
        result->run_tag = excite_run_tag_;
    }
    result->abort_reason = reason;
    result->success = reason.empty();
    result->message = reason.empty() ? "Excitation complete" : ("Aborted: " + reason);

    excite_active_ = false;
    busy_ = false;

    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
    } else if (result->success) {
        goal_handle->succeed(result);
    } else {
        goal_handle->abort(result);
    }
    RCLCPP_INFO(this->get_logger(),
        "Excitation done: success=%d reason=%s samples=%u drift=%.2fm tilt=%.1fdeg",
        result->success, reason.c_str(), result->samples_injected,
        result->max_drift_m, result->max_tilt_seen_deg);
}

void OffboardControlNode::stop_excitation_callback(
    const offboard_interfaces::srv::StopExcitation::Request::SharedPtr request,
    const offboard_interfaces::srv::StopExcitation::Response::SharedPtr response)
{
    if (!excite_active_.load()) {
        response->success = false;
        response->message = "No excitation running";
        return;
    }
    if (request->emergency) {
        request_excite_abort("emergency_stop", true);
        response->message = "Emergency stop: immediate revert to hold";
    } else {
        excite_soft_stop_ = true;   // graceful: jump to settle-out then revert
        response->message = "Stop requested: settle-out then revert";
    }
    response->success = true;
    RCLCPP_WARN(this->get_logger(), "StopExcitation: %s", response->message.c_str());
}

void OffboardControlNode::request_excite_abort(const std::string & reason, bool emergency)
{
    if (!excite_active_.load()) {
        return;
    }
    {
        std::lock_guard<std::mutex> lk(excite_mutex_);
        if (excite_abort_reason_.empty()) {
            excite_abort_reason_ = reason;
        }
    }
    if (emergency) {
        excite_emergency_ = true;
    }
    excite_abort_ = true;
}

bool OffboardControlNode::excite_compute_baseline(Eigen::Quaterniond & q_hover, float & thrust_cmd,
                                                  double & tilt_deg, double & drift_m)
{
    // Slow horizontal position hold (PX4 position controller is bypassed): small P
    // on NED position error → desired tilt toward the captured start point.
    CurrentPose local_position;
    Eigen::Quaterniond q_ned_body;
    {
        std::lock_guard<std::mutex> lk(telemetry_mutex_);
        local_position = current_local_position_;
        q_ned_body = current_q_ned_body_;
    }
    const double ex = excite_initial_pose_ned_.x - local_position.x;  // north error
    const double ey = excite_initial_pose_ned_.y - local_position.y;  // east error
    drift_m = std::sqrt(ex * ex + ey * ey);

    const double lim = excite_max_baseline_tilt_;
    // NED FRD: +north needs nose-down (negative pitch); +east needs positive roll.
    // P0-4: add a velocity D-term so the baseline damps drift instead of only pulling
    // back on position — pure-P let Pattern B chirps drift ~8 m in 13 s and the
    // geofence chopped the ID window. vx/vy are the NED odometry velocities.
    const double pitch_des = std::max(-lim, std::min(lim,
        -excite_kp_pos_ * ex + excite_kd_vel_ * local_position.vx));
    const double roll_des  = std::max(-lim, std::min(lim,
         excite_kp_pos_ * ey - excite_kd_vel_ * local_position.vy));
    excite_bl_roll_ = roll_des;    // stash for the ExcitationSample baseline field
    excite_bl_pitch_ = pitch_des;

    q_hover = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(
        roll_des, pitch_des, excite_start_yaw_ned_);

    // Altitude hold: P on AGL error around the hover throttle.
    const double agl = -local_position.z;
    const double alt_err = excite_hold_alt_m_ - agl;
    const double t = excite_hover_thrust_used_ + excite_kp_alt_ * alt_err;
    thrust_cmd = static_cast<float>(std::max(0.1, std::min(0.9, t)));

    tilt_deg = tilt_from_quat(q_ned_body) * 180.0 / M_PI;
    return true;
}

void OffboardControlNode::excite_publish_attitude(const Eigen::Quaterniond & q_ned, float thrust_cmd)
{
    px4_msgs::msg::VehicleAttitudeSetpoint msg{};
    msg.timestamp = getTimestamp();
    const Eigen::Quaterniond q = q_ned.normalized();
    msg.q_d[0] = static_cast<float>(q.w());
    msg.q_d[1] = static_cast<float>(q.x());
    msg.q_d[2] = static_cast<float>(q.y());
    msg.q_d[3] = static_cast<float>(q.z());
    msg.thrust_body[0] = 0.0f;
    msg.thrust_body[1] = 0.0f;
    msg.thrust_body[2] = -std::max(0.0f, std::min(1.0f, thrust_cmd));
    msg.yaw_sp_move_rate = std::numeric_limits<float>::quiet_NaN();
    vehicle_attitude_setpoint_publisher_->publish(msg);
}

void OffboardControlNode::excite_publish_rates(double wx, double wy, double wz, float thrust_cmd)
{
    px4_msgs::msg::VehicleRatesSetpoint msg{};
    msg.timestamp = getTimestamp();
    msg.roll = static_cast<float>(wx);
    msg.pitch = static_cast<float>(wy);
    msg.yaw = static_cast<float>(wz);
    msg.thrust_body[0] = 0.0f;
    msg.thrust_body[1] = 0.0f;
    msg.thrust_body[2] = -std::max(0.0f, std::min(1.0f, thrust_cmd));
    vehicle_rates_setpoint_publisher_->publish(msg);
}

bool OffboardControlNode::excite_is_stationary()
{
    // The drone is "settled" when body rates and horizontal velocity are both small
    // — used to gate the SETTLE_IN → INJECT transition so a goal never starts
    // injecting on top of the previous goal's residual transient (P2-9).
    float w[3];
    double vx, vy;
    {
        std::lock_guard<std::mutex> lk(telemetry_mutex_);
        w[0] = current_angular_velocity_[0];
        w[1] = current_angular_velocity_[1];
        w[2] = current_angular_velocity_[2];
        vx = current_local_position_.vx;
        vy = current_local_position_.vy;
    }
    const double wmag = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    const double vmag = std::sqrt(vx * vx + vy * vy);
    return wmag < excite_settle_rate_thresh_ && vmag < excite_settle_vel_thresh_;
}

bool OffboardControlNode::excite_safety_ok(std::string & reason, bool settle_grace)
{
    CurrentPose local_position;
    Eigen::Quaterniond q_ned_body;
    float angular_velocity[3];
    uint64_t angular_velocity_ts;
    bool landed;
    uint64_t landed_ts;
    {
        std::lock_guard<std::mutex> lk(telemetry_mutex_);
        local_position = current_local_position_;
        q_ned_body = current_q_ned_body_;
        angular_velocity[0] = current_angular_velocity_[0];
        angular_velocity[1] = current_angular_velocity_[1];
        angular_velocity[2] = current_angular_velocity_[2];
        angular_velocity_ts = current_angular_velocity_timestamp_;
        landed = current_landed_;
        landed_ts = current_landed_timestamp_;
    }
    // Ground-guard (P0-2): the land detector is groundtruth-independent, so it catches
    // the case a biased estimator masks — the drone once "excited" a full 18 s chirp
    // sitting on the pad because every AGL/geofence check trusts the offset z_est.
    // Only act on a fresh sample so a silent detector never forces a false abort.
    if (landed && landed_ts > 0 && (getTimestamp() - landed_ts) / 1e6 < 1.5) {
        RCLCPP_ERROR(this->get_logger(),
            "Excitation ground-guard abort: vehicle_land_detected.landed while injecting");
        reason = "ground";
        return false;
    }
    // Freshness first (prompt 98): a stale pose makes the remaining checks blind.
    if (enforce_freshness_ && local_position_timeout_ > 0.0 && local_position.timestamp > 0) {
        const double age_s = (getTimestamp() - local_position.timestamp) / 1000000.0;
        if (age_s > local_position_timeout_) {
            reason = "stale_telemetry";
            return false;
        }
    }
    // Same cylinder math as the general geofence (check_cylinder), but relative
    // to the captured start pose and in AGL.
    const double ex = excite_initial_pose_ned_.x - local_position.x;
    const double ey = excite_initial_pose_ned_.y - local_position.y;
    const double agl = -local_position.z;
    const std::string violated = check_cylinder(
        ex, ey, excite_geofence_radius_, agl, excite_agl_floor_, excite_agl_ceil_);
    if (!violated.empty()) {
        reason = violated;
        return false;
    }
    // Attitude-transient checks (tilt, runaway-rate) are suppressed during the
    // settle grace at the very start of a goal — the drone may still be shedding
    // the previous goal's REVERT transient, which is not a fault (P2-9). Geofence,
    // freshness and ground-guard above stay active even in grace.
    if (!settle_grace && tilt_from_quat(q_ned_body) > excite_max_tilt_rad_ * 1.3) {
        reason = "tilt";
        return false;
    }
    for (int i = 0; !settle_grace && i < 3; ++i) {
        if (std::abs(angular_velocity[i]) > kExciteRunawayRate) {
            RCLCPP_ERROR(this->get_logger(),
                "Excitation runaway-rate abort: axis %d w=[%.3f %.3f %.3f] rad/s, sample age %.3f s",
                i, angular_velocity[0], angular_velocity[1],
                angular_velocity[2],
                (getTimestamp() - angular_velocity_ts) / 1e6);
            reason = "rate";
            return false;
        }
    }
    return true;
}

bool OffboardControlNode::excite_rc_abort()
{
    // Shared RC gate (rc_should_abort) with the excitation's own stick
    // threshold; the SITL shortcut is inside the helper (enforce_rc_in_sim).
    std::string reason;
    return rc_should_abort(reason, excite_stick_abort_thresh_);
}

void OffboardControlNode::publish_excite_setpoint()
{
    // Acquire: pairs with the release store in execute_excitation. Until the run is
    // fully built, hold position (idle) rather than dereference a half-published state.
    if (!excite_ready_.load(std::memory_order_acquire)) {
        publish_idle_setpoint();
        return;
    }
    const auto now = this->now();
    const double dt = 1.0 / control_rate_;

    // --- Safety / abort evaluation (skip once already reverting) ---
    if (excite_phase_ != ExcitePhase::REVERT) {
        std::string reason;
        if (excite_emergency_.load()) {
            excite_phase_ = ExcitePhase::REVERT;  // hard stop: skip settle-out
        } else {
            if (excite_cancel_.load()) {
                request_excite_abort("cancel", false);
            }
            if (!excite_safety_ok(reason, excite_phase_ == ExcitePhase::SETTLE_IN)) {
                request_excite_abort(reason, false);
            }
            if ((now - excite_session_start_).seconds() > excite_max_session_s_) {
                request_excite_abort("timeout", false);
            }
            if (excite_rc_abort()) {
                request_excite_abort("rc", false);
            }
            if (excite_soft_stop_.load() &&
                (excite_phase_ == ExcitePhase::SETTLE_IN || excite_phase_ == ExcitePhase::INJECT)) {
                excite_phase_ = ExcitePhase::SETTLE_OUT;
                excite_phase_start_ = now;
            }
            if (excite_abort_.load()) {
                excite_phase_ = ExcitePhase::REVERT;
            }
        }
        if (excite_phase_ == ExcitePhase::REVERT) {
            // Skip this tick; the next one emits a clean position hold with the
            // correct offboard mode bit already set. Stream stays > 2 Hz.
            return;
        }
    }

    // --- Hover baseline (Pattern A) ---
    Eigen::Quaterniond q_hover;
    float thrust_cmd = static_cast<float>(excite_hover_thrust_used_);
    double tilt_deg = 0.0, drift_m = 0.0;
    excite_compute_baseline(q_hover, thrust_cmd, tilt_deg, drift_m);
    {
        std::lock_guard<std::mutex> lk(excite_mutex_);
        if (drift_m > excite_max_drift_m_) {
            excite_max_drift_m_ = drift_m;
        }
        if (tilt_deg > excite_max_tilt_seen_deg_) {  // measured tilt (both patterns)
            excite_max_tilt_seen_deg_ = tilt_deg;
        }
    }

    double ref = 0.0;
    std::string phase_name = "settle_in";

    // Publish the baseline plus a perturbation `delta` on the selected axis,
    // honoring the loop mode so the offboard-mode bit and the setpoint type match
    // in every phase. Returns the actually-applied (clamped) reference.
    auto publish_axis = [&](double delta) -> double {
        if (excite_rate_loop_) {
            // Pattern B (advanced): omega_sp = attitude-P stabilizer + delta.
            const double d = std::max(-kExciteHardRateCap, std::min(kExciteHardRateCap, delta));
            // Body-frame attitude error: both quats map body→NED, so
            // q_err = q_cur⁻¹ ⊗ q_hover rotates the CURRENT body axes onto the
            // hover attitude, expressed in the current body frame — which is what
            // VehicleRatesSetpoint (FRD body rates) expects. The previous
            // world-frame error (q_hover ⊗ q_cur⁻¹) mixed roll/pitch at yaw≠0.
            Eigen::Quaterniond q_cur;
            {
                std::lock_guard<std::mutex> lk(telemetry_mutex_);
                q_cur = current_q_ned_body_;
            }
            Eigen::Quaterniond q_err = q_cur.inverse() * q_hover;
            if (q_err.w() < 0.0) {
                q_err.coeffs() *= -1.0;  // shortest path: w<0 gives ~2π axis-angle → clamp slam
            }
            q_err.normalize();
            const Eigen::AngleAxisd aa(q_err);
            const Eigen::Vector3d e = aa.axis() * aa.angle();
            const double kp_att = 4.0;  // rad/s per rad attitude error
            double wx = kp_att * e.x();
            double wy = kp_att * e.y();
            double wz = kp_att * e.z();
            if (excite_axis_ == FlipAxis::ROLL)  { wx += d; }
            if (excite_axis_ == FlipAxis::PITCH) { wy += d; }
            if (excite_axis_ == FlipAxis::YAW)   { wz += d; }
            const double wcap = 2.0;  // rad/s per-axis clamp
            wx = std::max(-wcap, std::min(wcap, wx));
            wy = std::max(-wcap, std::min(wcap, wy));
            wz = std::max(-wcap, std::min(wcap, wz));
            excite_publish_rates(wx, wy, wz, thrust_cmd);
            return d;
        }
        // Pattern A (default): q_d = q_hover ⊗ δq(delta).
        const double ang = std::max(-kExciteHardAngleCap, std::min(kExciteHardAngleCap, delta));
        Eigen::Quaterniond q_d = q_hover * axis_delta_quat(excite_axis_, ang);
        q_d.normalize();
        if (tilt_from_quat(q_d) > excite_max_tilt_rad_) {
            // Over the hard tilt cap: abort AND publish the safe baseline this tick,
            // never the over-tilt command (the abort only takes effect next tick).
            request_excite_abort("tilt", false);
            excite_publish_attitude(q_hover, thrust_cmd);
            return 0.0;
        }
        excite_publish_attitude(q_d, thrust_cmd);
        return ang;
    };

    switch (excite_phase_) {
        case ExcitePhase::SETTLE_IN: {
            phase_name = "settle_in";
            publish_axis(0.0);
            // P2-9: don't start injecting until the minimum settle time has passed
            // AND the drone is actually stationary (rates + horizontal velocity
            // below threshold), so a residual transient from the previous goal's
            // REVERT can't corrupt the identification window or trip a false abort.
            // A dead-man of 3× settle_in bounds the wait if it never fully settles.
            const double settled_for = (now - excite_phase_start_).seconds();
            const bool min_elapsed = settled_for >= excite_settle_in_s_;
            const bool deadman = settled_for >= 3.0 * excite_settle_in_s_ + 2.0;
            if ((min_elapsed && excite_is_stationary()) || deadman) {
                if (deadman && !excite_is_stationary()) {
                    RCLCPP_WARN(this->get_logger(),
                        "Excitation settle_in dead-man: injecting though not fully "
                        "stationary after %.1fs", settled_for);
                }
                excite_phase_ = ExcitePhase::INJECT;
                excite_inject_start_ = now;
                excite_washout_state_ = 0.0;
            }
            break;
        }
        case ExcitePhase::INJECT: {
            phase_name = "inject";
            const double t = (now - excite_inject_start_).seconds();
            const double raw = excite_signal_ ? excite_signal_->value(t) : 0.0;
            const double washed = excitation::washout(raw, excite_washout_state_, dt, 0.2);
            ref = publish_axis(washed);
            {
                std::lock_guard<std::mutex> lk(excite_mutex_);
                excite_samples_injected_++;
                const double inj = (now - excite_inject_start_).seconds();
                if (inj > 0.1) {
                    excite_achieved_rate_hz_ = excite_samples_injected_ / inj;
                }
            }
            if (excite_signal_ && (excite_signal_->finished(t) || t > excite_max_inject_s_)) {
                excite_phase_ = ExcitePhase::SETTLE_OUT;
                excite_phase_start_ = now;
            }
            break;
        }
        case ExcitePhase::SETTLE_OUT: {
            phase_name = "settle_out";
            publish_axis(0.0);
            if ((now - excite_phase_start_).seconds() >= excite_settle_out_s_) {
                excite_phase_ = ExcitePhase::REVERT;
            }
            break;
        }
        case ExcitePhase::REVERT: {
            phase_name = "revert";
            px4_msgs::msg::TrajectorySetpoint msg{};
            msg.timestamp = getTimestamp();
            msg.position[0] = static_cast<float>(excite_initial_pose_ned_.x);
            msg.position[1] = static_cast<float>(excite_initial_pose_ned_.y);
            msg.position[2] = static_cast<float>(excite_initial_pose_ned_.z);
            msg.yaw = static_cast<float>(excite_initial_pose_ned_.yaw);
            msg.yawspeed = std::numeric_limits<float>::quiet_NaN();
            trajectory_setpoint_publisher_->publish(msg);
            {
                std::lock_guard<std::mutex> lk(cmd_mutex_);
                last_sended_pose = msg;
                setpoint_type_ = NONE;  // idle position-hold takes over
            }
            excite_finished_ = true;    // release the execute() thread
            break;
        }
    }

    // --- Feedback snapshot ---
    double meas_rate = 0.0;
    {
        std::lock_guard<std::mutex> lk(telemetry_mutex_);
        switch (excite_axis_) {
            case FlipAxis::ROLL:  meas_rate = current_angular_velocity_[0]; break;
            case FlipAxis::PITCH: meas_rate = current_angular_velocity_[1]; break;
            case FlipAxis::YAW:   meas_rate = current_angular_velocity_[2]; break;
        }
    }
    double progress = 0.0;
    if (excite_signal_ && excite_signal_->duration() > 0.0 && excite_phase_ == ExcitePhase::INJECT) {
        progress = std::min((now - excite_inject_start_).seconds() / excite_signal_->duration(), 1.0);
    } else if (excite_phase_ == ExcitePhase::SETTLE_OUT || excite_phase_ == ExcitePhase::REVERT) {
        progress = 1.0;
    }
    {
        std::lock_guard<std::mutex> lk(excite_mutex_);
        excite_fb_elapsed_ = (now - excite_session_start_).seconds();
        excite_fb_progress_ = progress;
        excite_fb_phase_ = phase_name;
        excite_fb_ref_ = ref;
        excite_fb_rate_ = meas_rate;
        excite_fb_tilt_ = tilt_deg;
        excite_fb_drift_ = drift_m;
    }

    // Emit the reference sample for the analyzer. Published every phase
    // (ref==0 outside inject) so the analyzer's forward-fill of exc_phase tracks the
    // true phase — the timestamp is the SAME clock (getTimestamp) as the setpoint
    // published this tick inside publish_axis, so r and u share one clock.
    {
        offboard_interfaces::msg::ExcitationSample s{};
        s.timestamp = getTimestamp();
        s.axis = (excite_axis_ == FlipAxis::ROLL) ? "roll"
               : (excite_axis_ == FlipAxis::PITCH) ? "pitch" : "yaw";
        s.loop = excite_rate_loop_ ? 1u : 0u;
        s.signal = excite_signal_str_;
        s.ref = static_cast<float>(ref);
        s.baseline = static_cast<float>(
            excite_rate_loop_ ? 0.0
            : (excite_axis_ == FlipAxis::ROLL)  ? excite_bl_roll_
            : (excite_axis_ == FlipAxis::PITCH) ? excite_bl_pitch_ : 0.0);
        s.phase = phase_name;
        s.run_tag = excite_run_tag_;
        excitation_reference_publisher_->publish(s);
    }
}
#pragma endregion Excitation

#pragma region Inits
void OffboardControlNode::Init_publishers(std::string px4_namespace)
{
    vehicle_command_client_ = create_client<px4_msgs::srv::VehicleCommand>(
        px4_namespace+"vehicle_command"
    );
    while (!vehicle_command_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    offboard_mode_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        px4_namespace + "in/offboard_control_mode", QoSConfigurator::commands());
    trajectory_setpoint_publisher_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        px4_namespace + "in/trajectory_setpoint", QoSConfigurator::commands());
    vehicle_attitude_setpoint_publisher_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
        px4_namespace + "in/vehicle_attitude_setpoint", QoSConfigurator::commands());
    vehicle_rates_setpoint_publisher_ = create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
        px4_namespace + "in/vehicle_rates_setpoint", QoSConfigurator::commands());
    // Excitation reference r(t) for the analyzer. Reliable so ID data
    // is not silently dropped; node-relative topic, not /fmu/in.
    excitation_reference_publisher_ = create_publisher<offboard_interfaces::msg::ExcitationSample>(
        "~/excitation/reference", rclcpp::QoS(rclcpp::KeepLast(200)).reliable());
}

void OffboardControlNode::Init_subscribers(std::string px4_namespace)
{
    vehicle_status_subscriber_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        px4_namespace + "out/vehicle_status",
        QoSConfigurator::system_status(),
        std::bind(&OffboardControlNode::vehicleStatusCallback, this, std::placeholders::_1));
    vehicle_global_pos_subscriber_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        px4_namespace + "out/vehicle_global_position",
        QoSConfigurator::sensor_data(),
        std::bind(&OffboardControlNode::vehicleGlobalPosCallback, this, std::placeholders::_1));
    vehicle_odom_subscriber_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        px4_namespace + "out/vehicle_odometry",
        QoSConfigurator::sensor_data(),
        std::bind(&OffboardControlNode::vehicleOdomCallback, this, std::placeholders::_1));
    battery_status_subscriber_ = create_subscription<px4_msgs::msg::BatteryStatus>(
        px4_namespace + "out/battery_status",
        QoSConfigurator::system_status(),
        std::bind(&OffboardControlNode::batteryCallback, this, std::placeholders::_1));
    manual_control_subscriber_ = this->create_subscription<px4_msgs::msg::ManualControlSetpoint>(
        px4_namespace + "out/manual_control_setpoint",
        QoSConfigurator::system_status(),
        std::bind(&OffboardControlNode::manualControlCallback, this, std::placeholders::_1));
    // Gyro (rate-loop output). Only published when the high-rate PID DDS profile is
    // active (fmu/matek_h743-slim_pid_on.px4); absent → stays zero, feedback shows 0.
    angular_velocity_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
        px4_namespace + "out/vehicle_angular_velocity",
        QoSConfigurator::sensor_data(),
        std::bind(&OffboardControlNode::vehicleAngularVelocityCallback, this, std::placeholders::_1));
    // Hover-thrust estimate (P0-1) — the excitation baseline reads it at target start.
    hover_thrust_subscriber_ = this->create_subscription<px4_msgs::msg::HoverThrustEstimate>(
        px4_namespace + "out/hover_thrust_estimate",
        QoSConfigurator::sensor_data(),
        std::bind(&OffboardControlNode::hoverThrustCallback, this, std::placeholders::_1));
    // Land detector (P0-2) — ground-guard for excitation.
    land_detected_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
        px4_namespace + "out/vehicle_land_detected",
        QoSConfigurator::system_status(),
        std::bind(&OffboardControlNode::landDetectedCallback, this, std::placeholders::_1));
    // High-rate feed-forward correction for TRAJECTORY (best-effort, newest wins).
    trajectory_correction_subscriber_ = this->create_subscription<offboard_interfaces::msg::TrajectoryCorrection>(
        "~/trajectory_correction",
        QoSConfigurator::commands(),
        std::bind(&OffboardControlNode::trajectoryCorrectionCallback, this, std::placeholders::_1));
}

void OffboardControlNode::Init_services()
{
    navigate_service_ = create_service<offboard_interfaces::srv::Navigate>(
        "navigate",
        std::bind(&OffboardControlNode::navigate_callback, this, 
                    std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_callback_group_);
    land_service_ = create_service<std_srvs::srv::Trigger>(
        "land",
        std::bind(&OffboardControlNode::land_callback, this,
                    std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_callback_group_);
    get_telemetry_service_ = create_service<offboard_interfaces::srv::GetTelemetry>(
        "get_telemetry",
        std::bind(&OffboardControlNode::get_telemetry_callback, this,
                    std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_callback_group_);
    set_altitude_service_ = create_service<offboard_interfaces::srv::SetAltitude>(
        "set_altitude",
        std::bind(&OffboardControlNode::set_altitude_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);
    set_yaw_service_ = create_service<offboard_interfaces::srv::SetYaw>(
        "set_yaw",
        std::bind(&OffboardControlNode::set_yaw_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);
    set_yaw_rate_service_ = create_service<offboard_interfaces::srv::SetYawRate>(
        "set_yaw_rate",
        std::bind(&OffboardControlNode::set_yaw_rate_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);
    set_position_service_ = create_service<offboard_interfaces::srv::SetPosition>(
        "set_position",
        std::bind(&OffboardControlNode::set_position_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);
    set_trajectory_service_ = create_service<offboard_interfaces::srv::SetTrajectory>(
        "set_trajectory",
        std::bind(&OffboardControlNode::set_trajectory_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);
    set_velocity_service_ = create_service<offboard_interfaces::srv::SetVelocity>(
        "set_velocity",
        std::bind(&OffboardControlNode::set_velocity_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);
    set_attitude_service_ = create_service<offboard_interfaces::srv::SetAttitude>(
        "set_attitude",
        std::bind(&OffboardControlNode::set_attitude_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);
    set_rates_service_ = create_service<offboard_interfaces::srv::SetRates>(
        "set_rates",
        std::bind(&OffboardControlNode::set_rates_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);
    flip_service_ = create_service<offboard_interfaces::srv::Flip>(
        "flip",
        std::bind(&OffboardControlNode::flip_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);
    // RTL (prompt 104) — same group and shape as land.
    return_to_launch_service_ = create_service<offboard_interfaces::srv::ReturnToLaunch>(
        "return_to_launch",
        std::bind(&OffboardControlNode::return_to_launch_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);
}

void OffboardControlNode::Init_timer()
{
    // (Re)creates the setpoint timer at control_rate_. Called from the
    // constructor (once) and from the excitation path to switch the stream rate
    // (30 <-> excite_stream_hz). Regular services must NOT call this — they use
    // startOffboardHeartbeat() to resume the persistent timer instead.
    if (setpoint_timer_ && !setpoint_timer_->is_canceled()) {
        setpoint_timer_->cancel();
    }
    const std::chrono::nanoseconds timer_period{static_cast<int64_t>(1e9/control_rate_)};
    setpoint_timer_ = create_wall_timer(
        timer_period,
        std::bind(&OffboardControlNode::publish_setpoint, this));
}

void OffboardControlNode::Init_actions()
{
    using namespace std::placeholders;
    // Reentrant group so feedback publishing / cancel can run alongside execute().
    excite_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    excitation_action_server_ = rclcpp_action::create_server<RunExcitation>(
        this,
        "run_excitation",
        std::bind(&OffboardControlNode::handle_excite_goal, this, _1, _2),
        std::bind(&OffboardControlNode::handle_excite_cancel, this, _1),
        std::bind(&OffboardControlNode::handle_excite_accepted, this, _1),
        rcl_action_server_get_default_options(),
        excite_callback_group_);
    stop_excitation_service_ = create_service<offboard_interfaces::srv::StopExcitation>(
        "stop_excitation",
        std::bind(&OffboardControlNode::stop_excitation_callback, this, _1, _2),
        rmw_qos_profile_services_default, excite_callback_group_);
    // Universal emergency stop (prompt 100): lives in the reentrant group so it
    // responds even while the mutually-exclusive service group is blocked
    // (e.g. a service waiting inside offboardAndArm).
    emergency_stop_service_ = create_service<offboard_interfaces::srv::EmergencyStop>(
        "emergency_stop",
        std::bind(&OffboardControlNode::emergency_stop_callback, this, _1, _2),
        rmw_qos_profile_services_default, excite_callback_group_);
    // Telemetry stream action (prompt 96) — its own worker thread + the shared
    // reentrant excite group for goal/cancel handling (they are tiny callbacks).
    receive_telemetry_action_server_ = rclcpp_action::create_server<ReceiveTelemetry>(
        this,
        "receive_telemetry",
        std::bind(&OffboardControlNode::handle_telemetry_goal, this, _1, _2),
        std::bind(&OffboardControlNode::handle_telemetry_cancel, this, _1),
        std::bind(&OffboardControlNode::handle_telemetry_accepted, this, _1),
        rcl_action_server_get_default_options(),
        excite_callback_group_);
    // NavigateTo action (prompt 97) — its own reentrant group + worker thread,
    // so feedback/cancel never block services or the setpoint timer.
    nav_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    navigate_to_action_server_ = rclcpp_action::create_server<NavigateTo>(
        this,
        "navigate_to",
        std::bind(&OffboardControlNode::handle_navigate_to_goal, this, _1, _2),
        std::bind(&OffboardControlNode::handle_navigate_to_cancel, this, _1),
        std::bind(&OffboardControlNode::handle_navigate_to_accepted, this, _1),
        rcl_action_server_get_default_options(),
        nav_callback_group_);
}
#pragma endregion Inits

#pragma region Callback
void OffboardControlNode::batteryCallback(const px4_msgs::msg::BatteryStatus::SharedPtr msg){
    std::lock_guard<std::mutex> lk(telemetry_mutex_);
    battery_percentage = msg->remaining * 100.0;
    battery_voltage = msg->voltage_v;
    current_battery_timestamp_ = getTimestamp();
}

void OffboardControlNode::manualControlCallback(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg){
    std::lock_guard<std::mutex> lk(telemetry_mutex_);
    current_manual_control_ = *msg;
    current_manual_control_timestamp_ = getTimestamp();
}

void OffboardControlNode::vehicleAngularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg){
    std::lock_guard<std::mutex> lk(telemetry_mutex_);
    current_angular_velocity_[0] = msg->xyz[0];
    current_angular_velocity_[1] = msg->xyz[1];
    current_angular_velocity_[2] = msg->xyz[2];
    current_angular_velocity_timestamp_ = getTimestamp();
}

void OffboardControlNode::hoverThrustCallback(const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(telemetry_mutex_);
    current_hover_thrust_ = msg->hover_thrust;
    current_hover_thrust_valid_ = msg->valid;
    current_hover_thrust_timestamp_ = getTimestamp();
}

void OffboardControlNode::landDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(telemetry_mutex_);
    current_landed_ = msg->landed;
    current_landed_timestamp_ = getTimestamp();
}

void OffboardControlNode::trajectoryCorrectionCallback(const offboard_interfaces::msg::TrajectoryCorrection::SharedPtr msg) {
    // Reject an invalid explicit frame outright (empty = resolve to base frame at
    // publish time). Group validity (all-finite/all-NaN) is enforced on the read
    // side per tick, so a partially-set group simply disables itself.
    if (!msg->frame_id.empty() && !validate_frame_id(msg->frame_id)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "TrajectoryCorrection dropped: invalid frame_id '%s'", msg->frame_id.c_str());
        return;
    }
    std::lock_guard<std::mutex> lk(corr_mutex_);
    correction_velocity_ = {msg->vx, msg->vy, msg->vz};
    correction_acceleration_ = {msg->ax, msg->ay, msg->az};
    correction_yaw_rate_ = msg->yaw_rate;
    correction_frame_id_ = msg->frame_id;
    // Node-side receive time — the freshness clock (see snapshot_cmd_state).
    correction_recv_ns_ = this->now().nanoseconds();
}

void OffboardControlNode::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lk(telemetry_mutex_);
        current_vehicle_status_ = *msg;
        current_vehicle_status_timestamp_ = getTimestamp();
    }
    {
        // Empty critical section: a waiter that just evaluated its predicate but
        // has not parked yet holds arm_wait_mutex_ — this blocks until it parks,
        // so the notify below cannot be lost.
        std::lock_guard<std::mutex> lk(arm_wait_mutex_);
    }
    arm_wait_cv_.notify_all();
    if (!prev_vehicle_status_msg_) {
        RCLCPP_INFO(this->get_logger(), "Initial vehicle status received");
    } else {
        // Check arming state changes
        if (prev_vehicle_status_msg_->arming_state != msg->arming_state) {
            if (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
                RCLCPP_INFO(this->get_logger(), "Vehicle disarmed");
            } else if (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
                RCLCPP_INFO(this->get_logger(), "Vehicle armed");
            }
        }
        // Check navigation state changes
        if (prev_vehicle_status_msg_->nav_state != msg->nav_state) {
            switch (msg->nav_state) {
                case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF:
                    RCLCPP_INFO(this->get_logger(), "Vehicle in takeoff mode");
                    break;
                case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND:
                    RCLCPP_INFO(this->get_logger(), "Vehicle in land mode");
                    break;
                case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:
                    RCLCPP_INFO(this->get_logger(), "Vehicle in hold mode");
                    break;
                case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD:
                    RCLCPP_INFO(this->get_logger(), "Vehicle in offboard mode");
                    break;
            }
        }
    }
    prev_vehicle_status_msg_ = msg;
}

void OffboardControlNode::vehicleGlobalPosCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(telemetry_mutex_);
    lat_ = msg->lat;
    lon_ = msg->lon;
    alt_ = msg->alt;
    current_global_position_timestamp_ = getTimestamp();
}

double OffboardControlNode::normalizeYaw(double yaw) {
    // Thin wrapper kept for compatibility; logic lives in TransformPX4 (testable).
    return TransformPX4::normalizeYaw(yaw);
}

void OffboardControlNode::vehicleOdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    Eigen::Quaterniond q_ned_to_body(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    double raw_heading_ned = px4_ros_com::frame_transforms::utils::quaternion::quaternion_get_yaw(q_ned_to_body);
    {
        // Update the snapshot under a short lock; TF publishing stays outside.
        std::lock_guard<std::mutex> lk(telemetry_mutex_);
        current_local_position_.x = msg->position[0];
        current_local_position_.y = msg->position[1];
        current_local_position_.z = msg->position[2];
        current_q_ned_body_ = q_ned_to_body;
        if (zero_heading_at_start_ && !initial_heading_set_) {
            initial_heading_ned_ = raw_heading_ned;
            initial_heading_set_ = true;
        }
        // Control-side heading is ALWAYS the raw NED yaw — the zeroing offset is
        // display-only (see build_telemetry), otherwise commands would shift.
        current_local_position_.heading = raw_heading_ned;
        current_local_position_.vx = msg->velocity[0];
        current_local_position_.vy = msg->velocity[1];
        current_local_position_.vz = msg->velocity[2];
        current_local_position_.timestamp = getTimestamp();
    }

    publish_odometry_transform(msg);
    publish_terrain_frame(-msg->position[2]);

    double log_yaw = raw_heading_ned;
    if (zero_heading_at_start_ && initial_heading_set_) {
        log_yaw = normalizeYaw(raw_heading_ned - initial_heading_ned_);
    }
    RCLCPP_DEBUG(this->get_logger(), "Drone pose in NED (%.2f, %.2f, %.2f, yaw=%.2f)", msg->position[0], msg->position[1], msg->position[2], log_yaw);
}
#pragma endregion Callback

#pragma region Utils
uint64_t OffboardControlNode::getTimestamp() {
    return static_cast<uint64_t>(this->now().nanoseconds() / 1000);
}

void OffboardControlNode::response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
        auto reply = future.get()->reply;
        service_result_ = reply.result;
        switch (service_result_)
        {
        case reply.VEHICLE_CMD_RESULT_ACCEPTED:
            RCLCPP_INFO(this->get_logger(), "command accepted");
            break;
        case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
            RCLCPP_WARN(this->get_logger(), "command temporarily rejected");
            break;
        case reply.VEHICLE_CMD_RESULT_DENIED:
            RCLCPP_WARN(this->get_logger(), "command denied");
            break;
        case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
            RCLCPP_WARN(this->get_logger(), "command unsupported");
            break;
        case reply.VEHICLE_CMD_RESULT_FAILED:
            RCLCPP_WARN(this->get_logger(), "command failed");
            break;
        case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
            RCLCPP_WARN(this->get_logger(), "command in progress");
            break;
        case reply.VEHICLE_CMD_RESULT_CANCELLED:
            RCLCPP_WARN(this->get_logger(), "command cancelled");
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "command reply unknown");
            break;
        }
        service_done_ = true;
    } else {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}
#pragma endregion Utils

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardControlNode>("/fmu/");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}