# One injected-reference sample from the excitation node.
# Published on ~/excitation/reference for the analyzer ONLY — NOT sent to PX4.
# Enables unbiased reference-based closed-loop ID: Ĝ = Φ_ry / Φ_ru.
uint64 timestamp        # PX4-µs, the SAME clock stamped on the outgoing setpoint
string axis             # roll | pitch | yaw
uint8 loop              # 0 attitude, 1 rate
string signal           # waveform type (step|doublet|3211|chirp|multisine|prbs)
float32 ref             # r(t): rad/s (rate loop) or rad (attitude loop)
float32 baseline        # axis baseline command the reference was overlaid on
string phase            # settle_in | inject | settle_out | revert
string run_tag          # same tag as in RunExcitation.result
