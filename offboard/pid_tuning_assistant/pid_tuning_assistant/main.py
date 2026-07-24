"""
Entry point: starts the ROS 2 node in a background thread and the
FastAPI/uvicorn server in the main thread.

Usage
-----
    ros2 run pid_tuning_assistant pid_tuning_assistant \
        --ros-args -p web_port:=8086
"""
import argparse
import os
import threading
import logging
from pathlib import Path

import yaml
import uvicorn

from .data_buffer import RecordingBuffer
from .web_server import create_app
from .analyzer import _PROFILES, _QUALITY_DEFAULTS

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
log = logging.getLogger("pid_tuning_assistant")


def main():
    parser = argparse.ArgumentParser(description="PID Tuning Assistant")
    parser.add_argument("--port", type=int, default=8086, help="Web UI port")
    parser.add_argument("--host", type=str, default="0.0.0.0",
                        help="Web UI host. Default 0.0.0.0 exposes the API to the "
                             "whole LAN — pair it with PID_ASSIST_TOKEN (or "
                             "auth_token in params.yaml) to require a bearer token.")
    parser.add_argument("--no-ros", action="store_true",
                        help="Skip ROS node (offline analysis mode)")
    parser.add_argument("--fcu-namespace", type=str, default="/fmu_control",
                        help="ROS namespace of the fmu_control node (default: /fmu_control)")
    # Pass through unknown args to rclpy
    args, _ = parser.parse_known_args()

    # Load config/params.yaml
    _cfg_path = Path(__file__).parent.parent / "config" / "params.yaml"
    _cfg: dict = {}
    try:
        with open(_cfg_path) as _f:
            _cfg = yaml.safe_load(_f) or {}
    except Exception as e:
        log.warning(f"Could not load params.yaml: {e}. Using defaults.")

    _vehicle_profile = _cfg.get("vehicle_profile", "default")
    _profiles_raw = _cfg.get("vehicle_profiles", {})
    _profile_thresholds = _profiles_raw.get(_vehicle_profile,
                                             _PROFILES.get(_vehicle_profile, _PROFILES["default"]))
    _quality_cfg = {k: _cfg.get(k, v) for k, v in _QUALITY_DEFAULTS.items()}
    _buffer_maxlen = _cfg.get("buffer_maxlen", 300_000)
    _max_ulog_mb = _cfg.get("max_ulog_mb", 256)
    _vio_expected_rate_hz = float(_cfg.get("vio_expected_rate_hz", 30))
    _max_runs_in_memory = int(_cfg.get("max_runs_in_memory", 5))

    # Optional API token: env wins over config. Empty → auth disabled (create_app
    # logs a warning). Never printed to logs.
    _auth_token = (os.environ.get("PID_ASSIST_TOKEN")
                   or _cfg.get("auth_token") or "").strip() or None

    buffer = RecordingBuffer(maxlen=_buffer_maxlen)
    stop_event = threading.Event()

    # ── ROS node thread ───────────────────────────────────────────────────────
    ros_thread = None
    if not args.no_ros:
        try:
            from .telemetry_node import spin_node
            ros_thread = threading.Thread(
                target=spin_node,
                args=(buffer, stop_event),
                kwargs={"vio_expected_rate_hz": _vio_expected_rate_hz},
                daemon=True,
                name="ros_spin",
            )
            ros_thread.start()
            log.info("ROS node thread started")
        except Exception as e:
            log.warning(f"Could not start ROS node: {e}. Running in offline mode.")
    else:
        log.info("Offline mode: ROS node disabled")

    # ── FastAPI server ────────────────────────────────────────────────────────
    app = create_app(
        buffer,
        fcu_namespace=args.fcu_namespace,
        vehicle_profile=_vehicle_profile,
        quality_cfg=_quality_cfg,
        max_ulog_mb=_max_ulog_mb,
        vio_expected_rate_hz=_vio_expected_rate_hz,
        auth_token=_auth_token,
        max_runs_in_memory=_max_runs_in_memory,
        ros_enabled=not args.no_ros,
    )

    # No custom SIGINT/SIGTERM handlers: uvicorn.run() installs its own signal
    # handlers and only returns once they have driven a graceful shutdown, so a
    # handler here would either be overwritten (dead code) or fight uvicorn's.
    # The ROS-thread cleanup below runs after uvicorn.run() returns.
    log.info(f"Web UI available at http://{args.host}:{args.port}")
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")

    # Reached after uvicorn handles SIGINT/SIGTERM and shuts the server down.
    stop_event.set()
    if ros_thread:
        ros_thread.join(timeout=3.0)


if __name__ == "__main__":
    main()
