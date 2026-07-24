"""
Example: telemetry monitoring.

Phase 1 — a 3-second high-rate stream via the /receive_telemetry action
(drone.control.stream_telemetry): the node pushes samples at the requested
rate, no request/response latency per sample.

Phase 2 — the classic get_telemetry polling loop. Stop with Ctrl+C.
"""

import time
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="example_telemetry_monitor")
try:
    # --- Phase 1: onboard-paced stream (new API) ---
    print("--- stream_telemetry: 20 Hz for 3 s ---")

    def on_sample(t):
        print(f"[stream] pos=({t.x:.2f}, {t.y:.2f}, {t.z:.2f}) "
              f"vel=({t.vx:.2f}, {t.vy:.2f}, {t.vz:.2f}) mode={t.mode}")

    res = drone.control.stream_telemetry(on_sample, rate_hz=20.0, duration=3.0)
    print(f"--- stream finished: samples_sent={res.samples_sent} ---")

    # --- Phase 2: polling loop (classic API) ---
    while True:
        t = drone.control.get_telemetry(frame_id="map")
        print(
            f"connected={t.connected} armed={t.armed} mode={t.mode} "
            f"pos=({t.x:.2f}, {t.y:.2f}, {t.z:.2f}) yaw={t.yaw:.2f} "
            f"vel=({t.vx:.2f}, {t.vy:.2f}, {t.vz:.2f}) "
            f"lat={t.lat:.6f} lon={t.lon:.6f} alt={t.alt:.2f}"
        )
        time.sleep(0.5)
except KeyboardInterrupt:
    print("Stop by Ctrl+C")
finally:
    drone.close()