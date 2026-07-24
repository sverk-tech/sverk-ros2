# Telemetry snapshot (same fields as the GetTelemetry.srv response, plus stamp).
# Published on ~/telemetry at telemetry_rate_hz (0 = disabled) and streamed as
# ReceiveTelemetry action feedback.
uint64 stamp        # us, node clock
string frame_id     # actual frame of x/y/z, vx/vy/vz, yaw
bool connected
bool armed
string mode
float32 x
float32 y
float32 z
float64 lat
float64 lon
float32 alt
float32 vx
float32 vy
float32 vz
float32 roll
float32 pitch
float32 yaw
float32 roll_rate
float32 pitch_rate
float32 yaw_rate
float32 voltage
float32 cell_voltage
