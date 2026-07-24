# Node diagnostics snapshot, published on ~/status at a low rate (~2 Hz) for
# UI/qgc/debugging. Does not affect the control path.
uint64 stamp              # us, node clock
string setpoint_type      # NONE/NAVIGATE/POSITION/VELOCITY/ATTITUDE/RATES/FLIP/EXCITE/...
bool busy
bool armed
string nav_mode           # OFFBOARD/POSCTL/...
bool offboard_active
float32 last_target_x     # NED, from the last published trajectory setpoint
float32 last_target_y
float32 last_target_z
float32 last_target_yaw
string active_maneuver    # "", "navigate", "navigate_to", "flip", "excitation:<tag>", ...
string last_abort_reason  # last safe-fallback reason, if any
float32 telemetry_age_s   # age of the odometry snapshot
float32 battery_pct
