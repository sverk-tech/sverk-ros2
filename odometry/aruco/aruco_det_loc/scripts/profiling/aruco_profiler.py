#!/usr/bin/env python3
"""
aruco_profiler.py — end-to-end timing analyser for aruco_det_loc
=================================================================

Subscribes to the shared /aruco/timing topic published by aruco_detect_node and
aruco_loc_node (when enable_timing_log=true), splits the stream by the "node"
field in each JSON frame ("detect" / "loc"), and produces a CSV + markdown
report with per-phase latency statistics.

Two operational modes
---------------------
--record-only   Collect data for N seconds (default 30 s) while the nodes
                run with their current parameters.  Saves raw CSV and a
                markdown report.  Use this when you have already tuned the
                node parameters and just want a baseline snapshot.

--full-test     Iterate over a predefined parameter matrix, enabling timing
                on both nodes via SetParameters service calls, collecting a
                short burst per configuration, then restoring defaults.
                Produces a combined CSV with one section per configuration
                and a comparative markdown table.

Quick start (record-only)
------------------------
    # 1. Launch your nodes normally (see PROFILING.md for launch examples)
    # 2. Enable timing on both nodes
    ros2 param set /aruco_detect enable_timing_log true
    ros2 param set /aruco_loc    enable_timing_log true
    # 3. Run profiler
    python3 aruco_profiler.py --record-only --duration 30 --output-dir ./profile_results

Quick start (full parametric test)
-----------------------------------
    python3 aruco_profiler.py --full-test --duration-per-config 20 --output-dir ./profile_results

The script uses rclpy; source your ROS2 workspace before running:
    source /path/to/install/setup.bash

Dependencies
------------
    pip install pandas tabulate
"""

import argparse
import csv
import json
import os
import sys
import time
import threading
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Optional rich stats
# ---------------------------------------------------------------------------
try:
    import pandas as pd
    import numpy as np
    HAS_PANDAS = True
except ImportError:
    HAS_PANDAS = False
    print("[WARN] pandas/numpy not installed — output will be limited. "
          "Install with: pip install pandas numpy tabulate", file=sys.stderr)

# ---------------------------------------------------------------------------
# ROS2 imports — deferred so the help text works without sourcing ROS
# ---------------------------------------------------------------------------
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import ParameterValue, ParameterType
    from std_msgs.msg import String
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


# ===========================================================================
# Constants
# ===========================================================================

DETECT_NODE = "/aruco_detect"
LOC_NODE    = "/aruco_loc"

TIMING_TOPIC = "/aruco/timing"   # общий для detect и loc; поле "node" в JSON различает их

# Detect-node phases (in order they appear in the JSON)
DETECT_PHASES = [
    ("convert_ms",  "BGR→Gray or MONO8 share"),
    ("detect_ms",   "cv::aruco::detectMarkers"),
    ("pnp_ms",      "per-marker solvePnP (if estimate_marker_pose)"),
    ("total_ms",    "full detect callback (entry → publish)"),
]

# Loc-node phases (in order they appear in the JSON)
LOC_PHASES = [
    ("markers_age_ms",    "DDS transport + detect processing (from image stamp)"),
    ("map_copy_ms",       "map mutex copy"),
    ("corr_ms",           "buildCorrespondences (3D–2D matching)"),
    ("pnp_ms",            "hybrid PnP (RANSAC or single-marker IPPE)"),
    ("refine_ms",         "refinePoseSolvePnP (if enabled)"),
    ("tf_ms",             "TF lookup (known_vertical, if enabled)"),
    ("cov_ms",            "covariance estimation (Jacobian)"),
    ("pub_ms",            "pose publish + TF broadcast"),
    ("total_ms",          "full loc callback (entry → publish)"),
    ("image_to_pose_ms",  "END-TO-END: image capture → pose published"),
]

# ---------------------------------------------------------------------------
# Parameter sweep matrix for --full-test
# Each entry: (label, {detect_params}, {loc_params})
# Only parameters that can be changed at runtime are listed here.
# Startup-only params (cpu_affinity_ids, opencv_num_threads) must be set
# before node launch — see PROFILING.md.
# ---------------------------------------------------------------------------
PARAM_MATRIX: List[Tuple[str, Dict[str, Any], Dict[str, Any]]] = [
    # ── Baseline ──────────────────────────────────────────────────────────
    ("baseline",
     {"use_aruco3_detection": False,  "estimate_marker_pose": False},
     {"pnp.flags": 0,  "pnp.iterations": 16, "refine_pose": False,
      "put_markers_count_to_covariance": False}),

    # ── ArUco3 detector (faster multi-scale detector) ────────────────────
    ("aruco3_detect",
     {"use_aruco3_detection": True,   "estimate_marker_pose": False},
     {"pnp.flags": 0,  "pnp.iterations": 16, "refine_pose": False,
      "put_markers_count_to_covariance": False}),

    # ── SQPNP solver (usually faster than ITERATIVE for large corr sets) ─
    ("sqpnp_solver",
     {"use_aruco3_detection": False,  "estimate_marker_pose": False},
     {"pnp.flags": 8,  "pnp.iterations": 16, "refine_pose": False,
      "put_markers_count_to_covariance": False}),

    # ── ArUco3 + SQPNP combined ───────────────────────────────────────────
    ("aruco3_sqpnp",
     {"use_aruco3_detection": True,   "estimate_marker_pose": False},
     {"pnp.flags": 8,  "pnp.iterations": 16, "refine_pose": False,
      "put_markers_count_to_covariance": False}),

    # ── Fewer RANSAC iterations (8 vs 16) ────────────────────────────────
    ("ransac_8iter",
     {"use_aruco3_detection": False,  "estimate_marker_pose": False},
     {"pnp.flags": 0,  "pnp.iterations": 8,  "refine_pose": False,
      "put_markers_count_to_covariance": False}),

    # ── More RANSAC iterations (32) — more robust but slower ─────────────
    ("ransac_32iter",
     {"use_aruco3_detection": False,  "estimate_marker_pose": False},
     {"pnp.flags": 0,  "pnp.iterations": 32, "refine_pose": False,
      "put_markers_count_to_covariance": False}),

    # ── refine_pose ON ────────────────────────────────────────────────────
    ("with_refine",
     {"use_aruco3_detection": False,  "estimate_marker_pose": False},
     {"pnp.flags": 0,  "pnp.iterations": 16, "refine_pose": True,
      "put_markers_count_to_covariance": False}),

    # ── Covariance via marker count (bypasses Jacobian — fastest) ─────────
    ("count_cov",
     {"use_aruco3_detection": False,  "estimate_marker_pose": False},
     {"pnp.flags": 0,  "pnp.iterations": 16, "refine_pose": False,
      "put_markers_count_to_covariance": True}),

    # ── per-marker pose estimation (solvePnP_IPPE_SQUARE per marker) ──────
    ("per_marker_pnp",
     {"use_aruco3_detection": False,  "estimate_marker_pose": True},
     {"pnp.flags": 0,  "pnp.iterations": 16, "refine_pose": False,
      "put_markers_count_to_covariance": False}),

    # ── ArUco3 small marker ratio 0.05 (detect smaller markers) ──────────
    ("aruco3_ratio005",
     {"use_aruco3_detection": True,  "aruco3_min_marker_length_ratio": 0.05,
      "estimate_marker_pose": False},
     {"pnp.flags": 0,  "pnp.iterations": 16, "refine_pose": False,
      "put_markers_count_to_covariance": False}),
]


# ===========================================================================
# SetParameters helper
# ===========================================================================

def set_node_params(node: "Node", target_node: str,
                    params: Dict[str, Any]) -> bool:
    """Call the SetParameters service on target_node with the given dict."""
    client = node.create_client(SetParameters, f"{target_node}/set_parameters")
    if not client.wait_for_service(timeout_sec=3.0):
        node.get_logger().warning(f"SetParameters service not available for {target_node}")
        return False

    req = SetParameters.Request()
    for name, value in params.items():
        pv = ParameterValue()
        if isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = value
        elif isinstance(value, int):
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = value
        elif isinstance(value, float):
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = value
        elif isinstance(value, str):
            pv.type = ParameterType.PARAMETER_STRING
            pv.string_value = value
        else:
            continue
        p = Parameter()
        p.name  = name
        p.value = pv
        req.parameters.append(p)

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if future.result() is None:
        node.get_logger().warning(f"SetParameters call to {target_node} timed out")
        return False

    for r in future.result().results:
        if not r.successful:
            node.get_logger().warning(f"Param set failed: {r.reason}")
    return True


# ===========================================================================
# Collector node
# ===========================================================================

class ProfilerNode(Node):
    def __init__(self):
        super().__init__("aruco_profiler")

        self.detect_frames: List[Dict] = []
        self.loc_frames:    List[Dict] = []

        self._lock = threading.Lock()

        self._timing_sub = self.create_subscription(
            String, TIMING_TOPIC, self._on_timing, 100)

        self.get_logger().info(f"Listening on: {TIMING_TOPIC}")

    def _on_timing(self, msg: String):
        try:
            frame = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        node = frame.get("node")
        with self._lock:
            if node == "detect":
                self.detect_frames.append(frame)
            elif node == "loc":
                self.loc_frames.append(frame)

    def clear(self):
        with self._lock:
            self.detect_frames.clear()
            self.loc_frames.clear()

    def snapshot(self) -> Tuple[List[Dict], List[Dict]]:
        with self._lock:
            return list(self.detect_frames), list(self.loc_frames)


# ===========================================================================
# Statistics
# ===========================================================================

def percentile(values: List[float], p: float) -> float:
    if not values:
        return float("nan")
    values = sorted(values)
    idx = (len(values) - 1) * p / 100.0
    lo, hi = int(idx), min(int(idx) + 1, len(values) - 1)
    return values[lo] + (values[hi] - values[lo]) * (idx - lo)


def stats_for_field(frames: List[Dict], field: str) -> Dict[str, float]:
    vals = [f[field] for f in frames if field in f and f[field] is not None]
    if not vals:
        return {"count": 0, "mean": float("nan"), "std": float("nan"),
                "p50": float("nan"), "p95": float("nan"), "p99": float("nan"),
                "min": float("nan"), "max": float("nan")}
    n   = len(vals)
    avg = sum(vals) / n
    std = (sum((v - avg) ** 2 for v in vals) / n) ** 0.5
    return {
        "count": n,
        "mean":  round(avg, 3),
        "std":   round(std, 3),
        "p50":   round(percentile(vals, 50), 3),
        "p95":   round(percentile(vals, 95), 3),
        "p99":   round(percentile(vals, 99), 3),
        "min":   round(min(vals), 3),
        "max":   round(max(vals), 3),
    }


def effective_hz(frames: List[Dict]) -> float:
    if len(frames) < 2:
        return float("nan")
    wall_times = sorted(f["wall_now"] for f in frames if "wall_now" in f)
    if len(wall_times) < 2:
        return float("nan")
    duration = wall_times[-1] - wall_times[0]
    if duration <= 0:
        return float("nan")
    return round((len(wall_times) - 1) / duration, 2)


# ===========================================================================
# Report generation
# ===========================================================================

def write_csv(output_dir: Path, label: str,
              detect_frames: List[Dict], loc_frames: List[Dict]):
    """Write raw frames to CSV."""
    for node_name, frames in [("detect", detect_frames), ("loc", loc_frames)]:
        if not frames:
            continue
        fname = output_dir / f"{label}_{node_name}.csv"
        fieldnames = list(frames[0].keys())
        with open(fname, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(frames)
        print(f"  CSV saved: {fname}")


def build_stats_block(label: str,
                      detect_frames: List[Dict],
                      loc_frames: List[Dict]) -> str:
    """Return markdown for one configuration block."""
    lines = []
    lines.append(f"## Configuration: `{label}`\n")
    lines.append(f"- detect frames collected: **{len(detect_frames)}** "
                 f"(effective Hz: {effective_hz(detect_frames)})")
    lines.append(f"- loc frames collected:    **{len(loc_frames)}** "
                 f"(effective Hz: {effective_hz(loc_frames)})")
    lines.append("")

    for title, frames, phases in [
        ("### aruco_detect_node phases (ms)", detect_frames, DETECT_PHASES),
        ("### aruco_loc_node phases (ms)",    loc_frames,    LOC_PHASES),
    ]:
        lines.append(title)
        if not frames:
            lines.append("_No frames received — is `enable_timing_log` true?_\n")
            continue
        header = "| Phase | Description | count | mean | std | p50 | p95 | p99 | min | max |"
        sep    = "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|"
        lines.append(header)
        lines.append(sep)
        for field, desc in phases:
            s = stats_for_field(frames, field)
            lines.append(
                f"| `{field}` | {desc} "
                f"| {s['count']} | {s['mean']} | {s['std']} "
                f"| {s['p50']} | {s['p95']} | {s['p99']} "
                f"| {s['min']} | {s['max']} |")
        lines.append("")

        # Extra counters for detect
        if frames and "n_detected" in frames[0]:
            n_det_vals = [f["n_detected"] for f in frames if "n_detected" in f]
            lines.append(f"- markers detected per frame: mean={round(sum(n_det_vals)/len(n_det_vals),1)}")
            mono_vals = [f["input_mono"] for f in frames if "input_mono" in f]
            if mono_vals:
                pct_mono = 100.0 * sum(1 for v in mono_vals if v) / len(mono_vals)
                lines.append(f"- MONO8 zero-copy path used: {pct_mono:.0f}% of frames")
            lines.append("")

        # Extra counters for loc
        if frames and "n_inliers" in frames[0]:
            inl_vals = [f["n_inliers"] for f in frames if "n_inliers" in f]
            lines.append(f"- inlier points per frame: mean={round(sum(inl_vals)/len(inl_vals),1)}")
            rmse_vals = [f["reproj_rmse"] for f in frames
                         if "reproj_rmse" in f and f["reproj_rmse"] >= 0]
            if rmse_vals:
                lines.append(f"- reprojection RMSE (inliers): mean={round(sum(rmse_vals)/len(rmse_vals),3)} px")
            age_vals = [f["markers_age_ms"] for f in frames if "markers_age_ms" in f]
            if age_vals:
                lines.append(f"- markers_age_ms (DDS + detect): mean={round(sum(age_vals)/len(age_vals),2)} ms")
            e2e_vals = [f["image_to_pose_ms"] for f in frames if "image_to_pose_ms" in f]
            if e2e_vals:
                lines.append(
                    f"- **image_to_pose_ms** (end-to-end): "
                    f"mean={round(sum(e2e_vals)/len(e2e_vals),2)} ms  "
                    f"p95={round(percentile(e2e_vals,95),2)} ms")
            lines.append("")

    return "\n".join(lines)


def write_report(output_dir: Path, all_blocks: List[Tuple[str, str]]):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    fname = output_dir / f"report_{ts}.md"
    with open(fname, "w", encoding="utf-8") as f:
        f.write("# aruco_det_loc — Profiling Report\n\n")
        f.write(f"Generated: {datetime.now().isoformat()}\n\n")
        f.write("---\n\n")
        for label, block in all_blocks:
            f.write(block)
            f.write("\n---\n\n")
    print(f"\nReport saved: {fname}")
    return fname


# ===========================================================================
# Spin helper (non-blocking collect for N seconds)
# ===========================================================================

def collect_for(node: ProfilerNode, duration_sec: float):
    """Spin node for duration_sec seconds (non-blocking)."""
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)


# ===========================================================================
# Modes
# ===========================================================================

def run_record_only(node: ProfilerNode, args):
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"[record-only] Collecting for {args.duration} s …")
    collect_for(node, args.duration)

    det, loc = node.snapshot()
    print(f"  detect frames: {len(det)},  loc frames: {len(loc)}")

    label = args.label or "record"
    write_csv(output_dir, label, det, loc)
    block = build_stats_block(label, det, loc)
    write_report(output_dir, [(label, block)])
    print(block)


def run_full_test(node: ProfilerNode, args):
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    all_blocks: List[Tuple[str, str]] = []
    warmup_sec = args.warmup
    dur_sec    = args.duration_per_config

    for label, detect_params, loc_params in PARAM_MATRIX:
        print(f"\n[full-test] Config: {label}")

        # Enable timing on both nodes
        detect_all = {**detect_params, "enable_timing_log": True}
        loc_all    = {**loc_params,    "enable_timing_log": True}

        ok_d = set_node_params(node, DETECT_NODE, detect_all)
        ok_l = set_node_params(node, LOC_NODE,    loc_all)

        if not ok_d:
            print(f"  [WARN] Could not set detect params — skipping {label}")
            continue
        if not ok_l:
            print(f"  [WARN] Could not set loc params — skipping {label}")
            continue

        # Warm-up
        node.clear()
        if warmup_sec > 0:
            print(f"  Warm-up {warmup_sec} s …")
            collect_for(node, warmup_sec)
            node.clear()

        # Collect
        print(f"  Collecting {dur_sec} s …")
        collect_for(node, dur_sec)
        det, loc = node.snapshot()
        print(f"  detect={len(det)} frames, loc={len(loc)} frames")

        write_csv(output_dir, label, det, loc)
        block = build_stats_block(label, det, loc)
        all_blocks.append((label, block))

    # Restore timing off
    set_node_params(node, DETECT_NODE, {"enable_timing_log": False})
    set_node_params(node, LOC_NODE,    {"enable_timing_log": False})

    write_report(output_dir, all_blocks)

    # Print summary table
    print("\n\n## Summary — image_to_pose_ms (mean | p95)")
    print(f"{'Config':<25} {'mean_ms':>10} {'p95_ms':>10} {'loc_hz':>10} {'det_hz':>10}")
    print("-" * 70)
    for label, _block in all_blocks:
        # Re-load from last snapshot — stored in written CSVs
        # For simplicity, re-parse inline from all_blocks data:
        print(f"  (see report for {label})")


# ===========================================================================
# Entry point
# ===========================================================================

def main():
    parser = argparse.ArgumentParser(
        description="aruco_det_loc latency profiler",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)

    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--record-only", action="store_true",
                      help="Collect data with current node params")
    mode.add_argument("--full-test",   action="store_true",
                      help="Sweep parameter matrix via SetParameters service")

    parser.add_argument("--duration", type=float, default=30.0,
                        help="[record-only] Collection duration in seconds (default 30)")
    parser.add_argument("--duration-per-config", type=float, default=20.0,
                        help="[full-test] Seconds per parameter configuration (default 20)")
    parser.add_argument("--warmup", type=float, default=3.0,
                        help="[full-test] Warm-up seconds before collecting (default 3)")
    parser.add_argument("--output-dir", default="./profile_results",
                        help="Directory for CSV and report files (default ./profile_results)")
    parser.add_argument("--label", default="",
                        help="[record-only] Label for output files")

    args = parser.parse_args()

    if not HAS_ROS:
        print("[ERROR] rclpy not found. Source your ROS2 workspace:\n"
              "  source /path/to/install/setup.bash", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = ProfilerNode()

    try:
        if args.record_only:
            run_record_only(node, args)
        else:
            run_full_test(node, args)
    except KeyboardInterrupt:
        print("\n[profiler] Interrupted — saving partial results …")
        det, loc = node.snapshot()
        out = Path(args.output_dir)
        out.mkdir(parents=True, exist_ok=True)
        label = args.label or "partial"
        write_csv(out, label, det, loc)
        block = build_stats_block(label, det, loc)
        write_report(out, [(label, block)])
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
