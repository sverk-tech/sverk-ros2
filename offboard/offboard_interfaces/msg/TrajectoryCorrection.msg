# High-rate feed-forward correction streamed on top of an active TRAJECTORY
# setpoint (base target set once via the set_trajectory service).
# Each group (velocity vx/vy/vz, acceleration ax/ay/az) is ALL finite or ALL NaN
# (NaN = that group not corrected -> falls back to the service value).
# yaw_rate: NaN = not corrected. Vectors are expressed in frame_id (empty = the
# base trajectory frame). Applied only while fresh: if no message arrives within
# correction_timeout, the node drops the correction and holds the base target.
uint64 stamp        # us, node clock (informational)
string frame_id     # frame of vx/vy/vz, ax/ay/az; empty = base trajectory frame
float32 vx
float32 vy
float32 vz
float32 ax
float32 ay
float32 az
float32 yaw_rate
