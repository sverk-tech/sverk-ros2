"""
Download the latest .ulg from the FCU's SD card over pure uXRCE-DDS (no
MAVLink cable, no physical SD-card access) using the log_download_bridge
firmware module via drone.fcu.download_log().

Measured throughput on the real UART/DDS link plateaus at ~27 KB/s
(window-size sweep on real hardware, see fmu_control/README.md) — a
multi-MB log can legitimately take several minutes. timeout_s=0 (default)
auto-computes a deadline from the reported log size; client_timeout=None
(default) blocks as long as the download legitimately runs, so the client
does not give up before the server's own deadline does.
"""

import time

import sverk_interfaces

OUT_PATH = "/home/sverk/last.ulg"

drone = sverk_interfaces.init(Nodename="example_log_download")
try:
    print(f"Downloading the latest log -> {OUT_PATH} ...")
    t0 = time.time()
    result = drone.fcu.download_log(OUT_PATH)
    elapsed = time.time() - t0

    print(f"success={result.success}  {result.message}")
    if result.success:
        kbps = result.bytes_received / 1024 / max(elapsed, 0.001)
        print(f"  log_name={result.log_name}  "
              f"{result.bytes_received}/{result.file_size} bytes -> {OUT_PATH}")
        print(f"  took {elapsed:.1f}s (~{kbps:.1f} KB/s)")
finally:
    drone.close()
