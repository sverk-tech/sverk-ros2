"""
FastAPI web server — bridges ROS telemetry to the browser.

Endpoints
---------
GET  /                     → serves index.html
GET  /api/status           → topic status, recording state
POST /api/start            → start recording  {run_name?: str}
POST /api/stop             → stop recording → triggers analysis
POST /api/params           → upload QGC .params file (multipart)
POST /api/params/live      → fetch params directly from FMU via fmu_control backup_params
GET  /api/analysis         → latest analysis JSON
GET  /api/download/json    → export JSON
GET  /api/download/csv     → export CSV
GET  /api/download/parquet → export Parquet
GET  /api/download/prompt  → export LLM prompt txt
WS   /ws/stream            → live telemetry frames (JSON, ~10 Hz)
"""
import asyncio
import dataclasses
import hmac
import json
import logging
import math
import os
import re
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
from pathlib import Path
from typing import Optional, Dict, Any, List

import numpy as np
from fastapi import (
    FastAPI, WebSocket, WebSocketDisconnect, UploadFile, File, HTTPException, Request,
)
from fastapi.responses import HTMLResponse, JSONResponse, PlainTextResponse, Response
from fastapi.staticfiles import StaticFiles

from .data_buffer import RecordingBuffer, TelemetryFrame, TopicStats
from .analyzer import PIDAnalyzer, AnalysisResult, REASON_TR, _PROFILES, _QUALITY_DEFAULTS
from .input_source import DdsRecordingSource

log = logging.getLogger(__name__)

# Thread pool for blocking ROS service calls (backup_params takes ~15 s) and
# ULog parsing. Single worker: these are long and must not pile up.
_ros_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="pid_ros_svc")

# Separate pool for analysis / CSV / Parquet export. Kept apart from _ros_executor
# so a running ULog upload (which holds the single _ros_executor slot for
# seconds) cannot block a stop-recording analysis, and vice versa. Two workers:
# one analysis + one export can overlap without starving each other.
_analysis_executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="pid_analysis")

# Accepted run-name shape. Everything reaching the browser DOM / Content-Disposition
# headers must match this — no HTML metacharacters, no header-injection bytes.
_RUN_NAME_RE = re.compile(r"^[A-Za-z0-9_.\-]{1,64}$")

# Whitelist of signals exposed by /api/runs/{run_name}/timeseries — client-supplied
# `signal` must be an actual TelemetryFrame field, never an arbitrary attribute.
_FRAME_FIELDS = frozenset(f.name for f in dataclasses.fields(TelemetryFrame))


def _sanitize_run_name(name: Optional[str]) -> str:
    """Coerce an arbitrary string into a name matching ``_RUN_NAME_RE``.

    Illegal characters become ``_``; the result is truncated to 64 chars and
    never empty. Used for uploaded filenames and defensively for header names.
    """
    safe = re.sub(r"[^A-Za-z0-9_.\-]", "_", name or "")[:64]
    return safe or "run"


def _export_basename(frames, run_name: Optional[str]) -> str:
    """Basename for exported files.

    Prefer the excitation run_tag so an analysis/export is named after
    the maneuver that produced it (maneuver↔analysis correlation); fall back to the
    recording run_name when no excitation reference was recorded.
    """
    if frames:
        counts: dict = {}
        for f in frames:
            tag = getattr(f, "exc_run_tag", None)
            if tag:
                counts[tag] = counts.get(tag, 0) + 1
        if counts:
            return _sanitize_run_name(max(counts.items(), key=lambda kv: kv[1])[0])
    return _sanitize_run_name(run_name)


def _json_safe(obj):
    """Recursively replace NaN/±Inf floats with ``None``.

    PX4 legitimately sends NaN in setpoints; a raw NaN serializes to the token
    ``NaN``, which ``JSON.parse`` in the browser rejects (or trips
    ``allow_nan=False`` and 500s). Sanitize before handing content to a
    JSONResponse.
    """
    if isinstance(obj, dict):
        return {k: _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_safe(v) for v in obj]
    if isinstance(obj, bool):
        return obj
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, np.floating):
        f = float(obj)
        return f if math.isfinite(f) else None
    return obj
from .param_parser import parse_qgc_params, extract_pid_params
from .telemetry_node import NAV_STATE_MAP

try:
    from ament_index_python.packages import get_package_share_directory
    STATIC_DIR = Path(get_package_share_directory("pid_tuning_assistant")) / "static"
except Exception:
    STATIC_DIR = Path(__file__).parent.parent / "static"


def _fetch_params_from_fcu_blocking(fcu_namespace: str) -> Dict[str, float]:
    """
    Blocking helper — runs in ThreadPoolExecutor (not the main asyncio thread).

    Creates a short-lived rclpy node, calls /fmu_control/backup_params
    (or <fcu_namespace>/backup_params), extracts PID params, destroys the node.

    Safe to run concurrently with the TelemetryNode since each node has its
    own executor and there is no shared state between them.
    """
    try:
        import rclpy
        from offboard_interfaces.srv import BackupParams
        from .param_parser import parse_from_backup_response, extract_pid_params
    except ImportError as e:
        raise RuntimeError(
            f"offboard_interfaces not available: {e}. "
            "Build offboard_interfaces first: colcon build --packages-select offboard_interfaces"
        )

    svc_name = f"{fcu_namespace.rstrip('/')}/backup_params"
    log.info(f"Calling {svc_name} (may take ~15 s) …")

    # rclpy is already initialised by the TelemetryNode thread
    node = rclpy.create_node("pid_param_fetcher_tmp")
    try:
        client = node.create_client(BackupParams, svc_name)
        if not client.wait_for_service(timeout_sec=6.0):
            raise RuntimeError(
                f"Service {svc_name} is not available. "
                "Is fmu_control running?  "
                "Check: ros2 service list | grep backup_params"
            )

        req = BackupParams.Request()
        future = client.call_async(req)

        deadline = time.time() + 90.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)

        if not future.done():
            raise TimeoutError(f"{svc_name} did not respond within 90 s")

        resp = future.result()
        if resp is None:
            raise RuntimeError(f"{svc_name} returned empty result")
        if not resp.success:
            raise RuntimeError(f"{svc_name} failed: {resp.message}")

        all_params = parse_from_backup_response(resp)
        pid = extract_pid_params(all_params)
        log.info(f"Fetched {len(all_params)} total params, {len(pid)} PID-relevant")
        return pid
    finally:
        node.destroy_node()


def _serialize_analysis(result: AnalysisResult, lang: str = "en") -> dict:
    templates = REASON_TR.get(lang, REASON_TR["en"])
    en_templates = REASON_TR["en"]

    def _fmt(tmpl_dict: dict, key: str, args: list) -> str:
        fmt = tmpl_dict.get(key) or en_templates.get(key, key)
        return fmt.format(*args) if args and "{" in fmt else fmt

    data = asdict(result)

    # translate recommendation reasons by key
    for rec in data.get("recommendations", []):
        key = rec.pop("reason_key", "")
        args = rec.pop("reason_kwargs", [])
        if key:
            rec["reason"] = _fmt(templates, key, args)

    # translate warnings by meta keys (robust — no string matching)
    meta = result.warning_meta  # List[tuple(key, args)]
    warnings = data.get("warnings", [])
    if meta and len(meta) == len(warnings):
        data["warnings"] = [_fmt(templates, k, a) for k, a in meta]

    data.pop("warning_meta", None)
    return data


def create_app(
    buffer: RecordingBuffer,
    fcu_namespace: str = "/fmu_control",
    vehicle_profile: str = "default",
    quality_cfg: Optional[dict] = None,
    max_ulog_mb: int = 256,
    auth_token: Optional[str] = None,
    vio_expected_rate_hz: float = 30.0,
    max_runs_in_memory: int = 5,
    ros_enabled: bool = True,
) -> FastAPI:
    app = FastAPI(title="PID Tuning Assistant", version="0.1.0")

    # ── authentication ────────────────────────────────────────────────────────
    # Optional bearer-token gate. This node listens on 0.0.0.0 inside a
    # privileged on-board container; without a token ANY LAN host can drive it.
    _auth_token = (auth_token or "").strip() or None
    if _auth_token is None:
        log.warning(
            "PID Tuning Assistant is running WITHOUT authentication — the web API "
            "is reachable by any host on the network. Set PID_ASSIST_TOKEN (or "
            "auth_token in params.yaml) to require a bearer token.")

    def _extract_token(request: Request) -> Optional[str]:
        auth = request.headers.get("authorization", "")
        if auth[:7].lower() == "bearer ":
            return auth[7:].strip()
        # Query param for links the browser can't add headers to (downloads).
        return request.query_params.get("token")

    @app.middleware("http")
    async def _auth_middleware(request: Request, call_next):
        # Only the REST API is gated; the HTML page and static assets stay open
        # so the operator can load the UI and enter the token. The WebSocket is
        # handled separately (own scope) in ws_stream().
        if _auth_token is not None and request.url.path.startswith("/api/"):
            provided = _extract_token(request)
            if not provided or not hmac.compare_digest(provided, _auth_token):
                return JSONResponse(
                    {"detail": "Missing or invalid API token"}, status_code=401)
        return await call_next(request)

    # Upload size caps (bytes). ULog guards SBC RAM; .params files are tiny.
    _max_ulog_bytes = int(max_ulog_mb) * 1024 * 1024
    _MAX_PARAMS_BYTES = 5 * 1024 * 1024

    # ── state shared across requests ──────────────────────────────────────────
    state: Dict[str, Any] = {
        "params": {},
        "analysis": None,       # AnalysisResult
        "last_frames": [],
        "runs": {},             # name → AnalysisResult
        "hifi": None,           # HiFiResult (ULog only)
        "plant": [],            # List[PlantEstimation] (ULog only)
        "autotune": None,       # AutotuneResult (ULog only)
        "vio": None,            # VioAnalysisResult (ULog only)
        "aid_sources": None,    # AidSourceAnalysisResult (ULog only)
        "ulog_parse": None,     # UlogParseResult (ULog only)
        "source_type": "dds",  # "dds" | "ulog"
    }

    # Cap on retained runs. Each run holds its full frame list (hundreds of
    # thousands of TelemetryFrames on a long recording), so keeping every run
    # forever leaks memory until the companion OOMs. Keep the newest N; older
    # runs are evicted (their /api/runs/*/timeseries then 404s — the UI History
    # tab degrades gracefully because dropped runs vanish from /api/runs too).
    _max_runs = max(1, int(max_runs_in_memory))

    def _register_run(name: str, entry: Dict[str, Any]) -> None:
        runs = state["runs"]
        runs[name] = entry   # dict preserves insertion order → oldest is first
        while len(runs) > _max_runs:
            oldest = next(iter(runs))
            runs.pop(oldest, None)

    # ── static files ──────────────────────────────────────────────────────────
    if STATIC_DIR.exists():
        app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

    # ── REST endpoints ────────────────────────────────────────────────────────

    @app.get("/", response_class=HTMLResponse)
    async def index():
        html_path = STATIC_DIR / "index.html"
        if html_path.exists():
            return HTMLResponse(html_path.read_text(encoding="utf-8"))
        return HTMLResponse("<h1>Static files not found. Check STATIC_DIR.</h1>", status_code=500)

    @app.get("/api/status")
    async def status():
        return {
            "recording": buffer.recording,
            "run_name": buffer.run_name,
            "frame_count": buffer.frame_count,
            "elapsed_s": round(buffer.elapsed, 1),
            "topic_status": buffer.topic_status,
            "params_loaded": bool(state["params"]),
            "nav_state_map": NAV_STATE_MAP,
        }

    @app.post("/api/start")
    async def start_recording(body: dict = None):
        # No ROS backend → no live DDS frames would ever arrive; a "recording"
        # here would silently capture nothing. Fail honestly instead.
        if not ros_enabled:
            raise HTTPException(
                503, "ROS backend disabled (--no-ros): live DDS recording "
                "unavailable. Use ULog upload for offline analysis.")
        if buffer.recording:
            raise HTTPException(400, "Already recording")
        run_name = ((body or {}).get("run_name", "") or "").strip()
        # Empty → buffer generates a safe default (run_<epoch>). A non-empty name
        # must be DOM/header-safe; reject rather than silently mangle.
        if run_name and not _RUN_NAME_RE.match(run_name):
            raise HTTPException(
                400, "Invalid run_name (allowed: A-Za-z0-9_.- , max 64 chars)")
        name = buffer.start_recording(run_name)
        return {"ok": True, "run_name": name}

    @app.post("/api/stop")
    async def stop_recording(body: dict = None):
        if not buffer.recording:
            raise HTTPException(400, "Not recording")
        lang = (body or {}).get("lang", "en")
        frames = buffer.stop_recording()
        state["last_frames"] = frames

        # Compute per-topic stats from the node (non-blocking — pure numpy on cached deques)
        topic_stats: Dict[str, TopicStats] = {}
        node = getattr(buffer, "_node_ref", None)
        if node is not None:
            try:
                topic_stats = node.compute_topic_stats(buffer._duration_s)
                buffer.topic_stats = topic_stats
            except Exception as e:
                log.warning(f"compute_topic_stats failed: {e}")

        _qcfg = quality_cfg or _QUALITY_DEFAULTS
        analyzer = PIDAnalyzer(
            state["params"],
            lang=lang,
            vehicle_profile=vehicle_profile,
            quality_cfg=_qcfg,
        )
        # analyse() is pure numpy/scipy over the full frame list — hundreds of
        # thousands of frames take seconds. Run it off the event loop so
        # /api/status, the WS stream, and other requests stay responsive.
        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(
            _analysis_executor,
            lambda: analyzer.analyse(
                frames, buffer.run_name, dict(buffer.topic_status),
                topic_stats=topic_stats,
            ),
        )
        state["analysis"] = result
        _register_run(buffer.run_name, {
            "result": result,
            "frames": frames,
            "analyzer": analyzer,
        })
        n_recs = len(result.recommendations)
        return {
            "ok": True,
            "n_frames": len(frames),
            "n_recommendations": n_recs,
            "warnings": result.warnings,
            "data_quality": {
                "achieved_rate_hz": result.data_quality.achieved_rate_hz,
                "f_nyquist_hz": result.data_quality.f_nyquist_hz,
                "overall_confidence": result.data_quality.overall_confidence,
                "spectral_ok": result.data_quality.spectral_ok,
            },
        }

    @app.post("/api/params")
    async def upload_params(file: UploadFile = File(...)):
        # Read at most 5 MB + 1 byte; more than the cap → 413.
        raw = await file.read(_MAX_PARAMS_BYTES + 1)
        if len(raw) > _MAX_PARAMS_BYTES:
            raise HTTPException(
                413, f".params file too large (limit {_MAX_PARAMS_BYTES // (1024*1024)} MB)")
        text = raw.decode("utf-8", errors="ignore")
        all_params = parse_qgc_params(text)
        pid_params = extract_pid_params(all_params)
        state["params"] = pid_params
        return {
            "ok": True,
            "source": "file",
            "loaded": len(pid_params),
            "params": pid_params,
        }

    @app.post("/api/params/live")
    async def fetch_params_live():
        """
        Fetch PID params directly from the FMU via fmu_control backup_params service.
        Runs in a thread (takes ~15 s for full parameter dump).
        fcu_namespace is configured at startup (default /fmu_control).
        READ-ONLY — never writes parameters.
        """
        # Guard the --no-ros case: rclpy.init() was never called, so
        # rclpy.create_node() inside the executor would raise deep in the ROS
        # layer and surface as an opaque 500. Report a clear 503 instead.
        try:
            import rclpy
        except ImportError:
            raise HTTPException(503, "ROS backend not available (rclpy not installed)")
        if not ros_enabled or not rclpy.ok():
            raise HTTPException(503, "ROS backend disabled (--no-ros)")

        loop = asyncio.get_event_loop()
        try:
            pid_params = await loop.run_in_executor(
                _ros_executor,
                _fetch_params_from_fcu_blocking,
                fcu_namespace,
            )
        except RuntimeError as e:
            raise HTTPException(503, str(e))
        except TimeoutError as e:
            raise HTTPException(504, str(e))
        except Exception:
            log.exception("params/live failed")
            raise HTTPException(500, "Internal error while fetching parameters")

        state["params"] = pid_params
        return {
            "ok": True,
            "source": "fmu_live",
            "loaded": len(pid_params),
            "params": pid_params,
        }

    @app.get("/api/analysis")
    async def get_analysis(lang: str = "en"):
        if state["analysis"] is None:
            raise HTTPException(404, "No analysis available. Record and stop first.")
        return JSONResponse(_json_safe(_serialize_analysis(state["analysis"], lang)))

    @app.get("/api/runs")
    async def list_runs():
        return {
            name: {
                "n_frames": len(v["frames"]),
                "timestamp": v["result"].timestamp,
                "n_recommendations": len(v["result"].recommendations),
            }
            for name, v in state["runs"].items()
        }

    @app.get("/api/runs/{run_name}/timeseries")
    async def run_timeseries(run_name: str, signal: str = "roll_rate", max_pts: int = 1000):
        """
        Return downsampled timeseries for a single signal from a stored run.
        Used by the History comparison chart.
        signal: any TelemetryFrame field name, e.g. roll_rate, roll_rate_sp, pitch, roll_sp ...
        """
        if run_name not in state["runs"]:
            raise HTTPException(404, f"Run '{run_name}' not found")
        # `signal` is client-supplied — only expose real TelemetryFrame fields,
        # never arbitrary attributes reachable through getattr.
        if signal not in _FRAME_FIELDS:
            raise HTTPException(400, f"Unknown signal '{signal}'")
        frames: list = state["runs"][run_name]["frames"]
        step = max(1, len(frames) // max_pts)
        sampled = frames[::step]
        t_vals = [_r(f.t, 3) for f in sampled]
        y_vals = [_r(getattr(f, signal, None)) for f in sampled]
        return {"run": run_name, "signal": signal, "t": t_vals, "y": y_vals}

    @app.get("/api/download/json")
    async def download_json():
        if state["analysis"] is None:
            raise HTTPException(404, "No analysis")
        _name = _export_basename(state.get("last_frames"), buffer.run_name)
        return Response(
            content=json.dumps(
                _json_safe(_serialize_analysis(state["analysis"], "en")), indent=2),
            media_type="application/json",
            headers={"Content-Disposition": f'attachment; filename="{_name}_analysis.json"'},
        )

    @app.get("/api/download/csv")
    async def download_csv():
        if not state["last_frames"]:
            raise HTTPException(404, "No frames")
        analyzer = PIDAnalyzer(state["params"])
        # Serializing hundreds of thousands of frames is CPU-bound — off-loop.
        frames = state["last_frames"]
        loop = asyncio.get_event_loop()
        csv_text = await loop.run_in_executor(
            _analysis_executor, analyzer.export_csv, frames)
        _name = _export_basename(state.get("last_frames"), buffer.run_name)
        return Response(
            content=csv_text,
            media_type="text/csv",
            headers={"Content-Disposition": f'attachment; filename="{_name}_telemetry.csv"'},
        )

    @app.get("/api/download/parquet")
    async def download_parquet():
        if not state["last_frames"]:
            raise HTTPException(404, "No frames")
        try:
            analyzer = PIDAnalyzer(state["params"])
            # pandas → pyarrow encode is heavy on big logs — run off the loop.
            frames = state["last_frames"]
            loop = asyncio.get_event_loop()
            data = await loop.run_in_executor(
                _analysis_executor, analyzer.export_parquet, frames)
            _name = _export_basename(state.get("last_frames"), buffer.run_name)
            return Response(
                content=data,
                media_type="application/octet-stream",
                headers={"Content-Disposition": f'attachment; filename="{_name}_telemetry.parquet"'},
            )
        except ImportError:
            raise HTTPException(500, "pyarrow not installed; install with: pip install pyarrow pandas")

    @app.get("/api/download/prompt")
    async def download_prompt():
        if state["analysis"] is None:
            raise HTTPException(404, "No analysis")
        analyzer = PIDAnalyzer(state["params"])
        text = analyzer.export_llm_prompt(state["analysis"], autotune=state["autotune"])
        _name = _export_basename(state.get("last_frames"), buffer.run_name)
        return Response(
            content=text,
            media_type="text/plain",
            headers={"Content-Disposition": f'attachment; filename="{_name}_llm_prompt.txt"'},
        )

    # ── ULog endpoints ────────────────────────────────────────────────────────

    @app.get("/api/ulog/available")
    async def ulog_available():
        from . import ulog_adapter
        return {"available": ulog_adapter.is_available(),
                "install_hint": "pip install pyulog"}

    @app.post("/api/ulog/upload")
    async def upload_ulog(file: UploadFile = File(...), lang: str = "en"):
        """
        Parse a .ulg file, run standard analysis + HiFi analysis.
        DDS recording is NOT affected — both modes coexist.
        """
        from . import ulog_adapter
        if not ulog_adapter.is_available():
            raise HTTPException(503,
                "pyulog not installed. "
                "Install with: pip install pyulog  "
                "then restart the assistant.")

        # Stream the upload to a temp file with a hard size cap. pyulog builds
        # full-size arrays in RAM, so an unbounded upload can OOM the companion
        # (1–4 GB). Count bytes as we go and abort with 413 on overflow.
        # The uploaded filename is attacker-controlled — sanitize it once, up
        # front, into a DOM/header-safe run name (illegal chars → '_').
        _upload_name = _sanitize_run_name(file.filename or "upload")

        tmp_path = None
        try:
            written = 0
            with tempfile.NamedTemporaryFile(suffix=".ulg", delete=False) as tf:
                tmp_path = tf.name
                while True:
                    chunk = await file.read(1024 * 1024)   # 1 MB
                    if not chunk:
                        break
                    written += len(chunk)
                    if written > _max_ulog_bytes:
                        raise HTTPException(
                            413, f"ULog file too large (limit {max_ulog_mb} MB)")
                    tf.write(chunk)

            loop = asyncio.get_event_loop()

            def _parse_and_analyse():
                from .ulog_adapter import parse_ulog
                from .hifi_analyzer import HiFiAnalyzer
                from .plant_estimator import PlantEstimator
                import dataclasses

                parse_res = parse_ulog(tmp_path)
                frames   = parse_res.frames
                metadata = parse_res.metadata
                # Present the sanitized original filename, not the temp file name.
                metadata.source_name = _upload_name

                # Standard analysis (same engine as DDS)
                _qcfg = quality_cfg or _QUALITY_DEFAULTS
                analyzer = PIDAnalyzer(
                    metadata.params or state["params"],
                    lang=lang,
                    vehicle_profile=vehicle_profile,
                    quality_cfg=_qcfg,
                )
                result = analyzer.analyse(
                    frames, metadata.source_name,
                    dict(metadata.topic_stats),
                    topic_stats=metadata.topic_stats,
                )

                # HiFi analysis (high-rate algorithms)
                hifi_result = HiFiAnalyzer().analyse(parse_res)

                # Plant estimation (needs FRF)
                plant_ests = []
                if hifi_result.frf and metadata.params:
                    pe = PlantEstimator(target_pm_deg=50.0, target_bw_hz=15.0)
                    plant_ests = pe.estimate(
                        hifi_result.frf,
                        metadata.params,
                        autotune_params=parse_res.autotune_params,
                    )

                # Offline PX4 autotune (ARX batch ID + GMVC).
                # Needs a high-rate log + params; degrades to warnings otherwise.
                from .autotune_offline import run_offline_autotune
                autotune_result = run_offline_autotune(parse_res)

                # VIO / EV external-vision analysis. Source-agnostic:
                # any EV fused by EKF2 (ArUco/VPE, LIO, scan-match, VIO, mocap).
                # Present only when the log carries estimator/EV topics.
                from .vio_analyzer import VioAnalyzer
                vio_result = VioAnalyzer(
                    expected_rate_hz=vio_expected_rate_hz, lang=lang).analyse(parse_res)

                # EKF2 aid-source registry analysis: EV is one entry
                # among flow/rng/gps/baro/mag, all run through one metric machine.
                from .aid_source_analyzer import AidSourceAnalyzer
                aid_result = AidSourceAnalyzer(lang=lang).analyse(parse_res)

                return (parse_res, result, hifi_result, plant_ests,
                        autotune_result, vio_result, aid_result, analyzer)

            try:
                (parse_res, result, hifi_result, plant_ests,
                 autotune_result, vio_result, aid_result, analyzer) = \
                    await loop.run_in_executor(_ros_executor, _parse_and_analyse)
            except HTTPException:
                raise
            except Exception:
                log.exception("ULog parse/analysis failed")
                raise HTTPException(500, "ULog parse/analysis failed")
        finally:
            if tmp_path is not None:
                try:
                    os.unlink(tmp_path)
                except OSError:
                    pass

        run_name = _upload_name
        state["ulog_parse"]  = parse_res
        state["analysis"]    = result
        state["last_frames"] = parse_res.frames
        state["hifi"]        = hifi_result
        state["plant"]       = plant_ests
        state["autotune"]    = autotune_result
        state["vio"]         = vio_result
        state["aid_sources"] = aid_result
        state["source_type"] = "ulog"
        state["params"] = parse_res.params_pid or state["params"]
        _register_run(run_name, {
            "result": result,
            "frames": parse_res.frames,
            "analyzer": analyzer,
        })

        import dataclasses
        return {
            "ok": True,
            "run_name": run_name,
            "n_frames": len(parse_res.frames),
            "duration_s": parse_res.metadata.duration_s,
            "n_recommendations": len(result.recommendations),
            "warnings": result.warnings + parse_res.warnings,
            "capabilities": dataclasses.asdict(parse_res.metadata.capabilities),
            "params_extracted": len(parse_res.params_pid),
            "hifi_available": bool(hifi_result.frf),
            "plant_available": bool(plant_ests),
            "autotune_available": bool(autotune_result and autotune_result.axes),
            "autotune_accepted": [a.axis for a in (autotune_result.axes if autotune_result else [])
                                  if a.accepted],
            "vio_available": bool(vio_result and vio_result.capabilities.vio_basic_available),
            "aid_sources_available": bool(aid_result and aid_result.available),
            # Native PX4 autotune status present in the log — enables
            # the reproduced-gains column of the tuning comparison.
            "autotune_status_available": bool(
                getattr(parse_res, "autotune_series", None) is not None
                and getattr(parse_res.autotune_series, "axes", {})),
        }

    @app.get("/api/hifi")
    async def get_hifi():
        """Return HiFi analysis results (ULog only)."""
        if state["hifi"] is None:
            raise HTTPException(404, "No HiFi analysis. Upload a .ulg file first.")
        return JSONResponse(_json_safe(state["hifi"].to_dict()))

    @app.get("/api/plant")
    async def get_plant():
        """Return plant estimation + synthesized gains (ULog only)."""
        if not state["plant"]:
            raise HTTPException(404, "No plant estimation. "
                "Need ULog with FRF-quality excitation (chirp sweep).")
        import dataclasses
        return JSONResponse(_json_safe([dataclasses.asdict(p) for p in state["plant"]]))

    @app.get("/api/autotune")
    async def get_autotune():
        """
        Return the offline PX4-autotune result (ULog only):
        per-axis ARX coeffs, VAF, GMVC-synthesized params, gate verdict.
        Read-only — gains are advisory, applied manually or via QGC.
        """
        if state["autotune"] is None:
            raise HTTPException(404, "No autotune result. Upload a high-rate "
                ".ulg file (SDLOG_PROFILE=17) with params first.")
        return JSONResponse(_json_safe(state["autotune"].to_dict()))

    @app.get("/api/vio")
    async def get_vio(lang: str = "en"):
        """
        Return the VIO / EV external-vision analysis (ULog only):
        test-ratio stats, innovation-bias classification (delay vs extrinsic),
        fusion dropouts, NEES consistency, EV rate/latency, EV_DELAY estimate,
        and EKF2_EV_* recommendations.  Source-agnostic (ArUco/VPE, LIO,
        scan-match, VIO, mocap).  READ-ONLY.
        """
        if state["vio"] is None:
            raise HTTPException(404, "No VIO/EV analysis. Upload a .ulg with "
                "estimator/EV topics first.")
        from .vio_analyzer import translate as _vio_tr
        data = state["vio"].to_dict()
        # Re-translate recommendation reasons by key so a language switch in the
        # UI re-fetches without re-parsing the log (matches /api/analysis).
        for rec in data.get("recommendations", []):
            key = rec.pop("reason_key", "")
            args = rec.pop("reason_kwargs", [])
            if key:
                rec["reason"] = _vio_tr(key, lang, args)
        return JSONResponse(_json_safe(data))

    @app.get("/api/aid_sources")
    async def get_aid_sources(lang: str = "en"):
        """
        Return the registry-driven EKF2 aid-source analysis (ULog only, prompt
        62): per-source (ev/optical_flow/rng_hgt/gps/baro/mag) test-ratio stats,
        innovation bias, fusion dropouts, NEES, rate, delay estimate and
        per-source EKF2 recommendations, plus a top-level coasting/fake_pos
        alarm.  EV is one source among many.  READ-ONLY.

        Reasons are re-translated by key so a UI language switch re-fetches
        without re-parsing the log (matches /api/analysis and /api/vio).
        """
        if state["aid_sources"] is None:
            raise HTTPException(404, "No aid-source analysis. Upload a .ulg with "
                "estimator topics first.")
        from .aid_source_analyzer import translate as _aid_tr, get_source
        data = state["aid_sources"].to_dict()
        for report in data.get("sources", []):
            src = get_source(report.get("key", ""))
            name = src.display_name if src else report.get("display_name", "")
            for rec in report.get("recommendations", []):
                key = rec.pop("reason_key", "")
                args = rec.pop("reason_kwargs", [])
                if key:
                    rec["reason"] = _aid_tr(key, lang, args, src=name,
                                            src_key=report.get("key", ""))
        return JSONResponse(_json_safe(data))

    def _vio_live_snapshot() -> Optional[dict]:
        """Non-destructive live VIO/EV snapshot from the ROS node, or None.

        Returns None when there is no ROS node or no VIO topic has been seen —
        the live panel then stays hidden.  Cheap enough to poll on the WS tick.
        """
        node = getattr(buffer, "_node_ref", None)
        if node is None or not hasattr(node, "vio_snapshot"):
            return None
        try:
            st = node.vio_snapshot(vio_expected_rate_hz)
        except Exception:
            log.debug("vio_snapshot failed", exc_info=True)
            return None
        if st is None or not st.available:
            return None
        return st.to_dict()

    @app.get("/api/vio/live")
    async def get_vio_live():
        """
        Live VIO/EV DDS watchdog snapshot (DDS mode): fusion flags,
        EV rate/latency, coasting alarm, EV-input vs EKF-output divergence, and
        an OK/WARN/CRIT verdict.  Lightweight polling alternative to the ``vio``
        field on /ws/stream.  READ-ONLY.

        NOTE: numeric innovation consistency and the EKF2_EV_DELAY estimate are
        NOT available live over DDS — see the offline ULog VIO/EV tab.
        """
        snap = _vio_live_snapshot()
        if snap is None:
            return JSONResponse({"available": False})
        return JSONResponse(_json_safe(snap))

    @app.get("/api/compare")
    async def get_compare():
        """
        Side-by-side tuning comparison: per axis, current gains
        vs PX4 native-autotune gains (reproduced from the logged ARX coeffs) vs the
        analyzer's model-based synthesis, plus each method's identified model and
        goodness metric.

        Sources are whatever the last ULog analysis produced — the view degrades
        to two columns (current vs analyzer) when the log carried no
        autotune_attitude_control_status topic.  READ-ONLY; never writes gains.
        """
        from .tuning_compare import build_tuning_comparison

        params = dict(state.get("params") or {})
        parse_res = state.get("ulog_parse")
        autotune_series = getattr(parse_res, "autotune_series", None) if parse_res else None
        plant_ests = state.get("plant") or []

        if not params and autotune_series is None and not plant_ests:
            raise HTTPException(404, "Nothing to compare. Upload a .ulg "
                "(ideally with an autotune run) and load params first.")

        # Desired rise time drives the reproduced PX4 gains; MC_AT_RISE_TIME sets
        # roll/pitch, yaw is hardcoded 0.20 s in the firmware.
        rise = params.get("MC_AT_RISE_TIME")
        sigma_map = None
        if rise and rise > 0:
            sigma_map = {"roll": float(rise), "pitch": float(rise), "yaw": 0.20}

        report = build_tuning_comparison(
            params=params,
            autotune_series=autotune_series,
            plant_estimations=plant_ests,
            sigma_map=sigma_map,
        )
        return JSONResponse(_json_safe(report))

    @app.get("/api/source")
    async def get_source():
        """Return current active source type and metadata."""
        meta = None
        if state["ulog_parse"] and state["source_type"] == "ulog":
            import dataclasses
            meta = dataclasses.asdict(state["ulog_parse"].metadata)
        return JSONResponse(_json_safe({
            "source_type": state["source_type"],
            "metadata": meta,
        }))

    # ── WebSocket stream ──────────────────────────────────────────────────────

    @app.websocket("/ws/stream")
    async def ws_stream(ws: WebSocket):
        # WebSocket auth: the browser cannot set an Authorization header on a WS
        # handshake, so the token travels as ?token=. Validate before accept().
        if _auth_token is not None:
            provided = ws.query_params.get("token")
            if not provided or not hmac.compare_digest(provided, _auth_token):
                await ws.close(code=1008)   # policy violation
                return
        await ws.accept()
        # Own per-client ring: two open tabs each get every frame instead of
        # racing over one shared ring (pop_live was destructive).
        consumer = buffer.attach_live_consumer()
        try:
            while True:
                frames = buffer.drain_live_consumer(consumer)
                payload: Dict[str, Any] = {}
                if frames:
                    # Downsample to last frame for live display
                    f = frames[-1]
                    payload.update({
                        "t": _r(f.t, 3),
                        "nav_state": f.nav_state,
                        "nav_state_name": NAV_STATE_MAP.get(f.nav_state, "?") if f.nav_state is not None else "?",
                        "roll_rate": _r(f.roll_rate), "roll_rate_sp": _r(f.roll_rate_sp),
                        "pitch_rate": _r(f.pitch_rate), "pitch_rate_sp": _r(f.pitch_rate_sp),
                        "yaw_rate": _r(f.yaw_rate), "yaw_rate_sp": _r(f.yaw_rate_sp),
                        "roll": _r(f.roll), "roll_sp": _r(f.roll_sp),
                        "pitch": _r(f.pitch), "pitch_sp": _r(f.pitch_sp),
                        "torque_roll": _r(f.torque_roll),
                        "torque_pitch": _r(f.torque_pitch),
                        "torque_yaw": _r(f.torque_yaw),
                        "m0": _r(f.m0), "m1": _r(f.m1), "m2": _r(f.m2), "m3": _r(f.m3),
                        "v_bat": _r(f.v_bat), "bat_remaining": _r(f.bat_remaining),
                    })
                # VIO/EV live watchdog — sent even when the rate loop is idle
                # (EV runs at its own cadence, unarmed there may be no ang_vel).
                vio = _vio_live_snapshot()
                if vio is not None:
                    payload["vio"] = vio
                if payload:
                    payload.setdefault("recording", buffer.recording)
                    payload.setdefault("frame_count", buffer.frame_count)
                    await ws.send_json(_json_safe(payload))
                await asyncio.sleep(0.1)   # ~10 Hz to browser
        except WebSocketDisconnect:
            pass
        finally:
            buffer.detach_live_consumer(consumer)

    return app


def _r(v, decimals: int = 4):
    if v is None:
        return None
    try:
        fv = float(v)
    except Exception:
        return None
    # NaN/±Inf serialize to the non-standard tokens NaN/Infinity, which break
    # JSON.parse in the browser. PX4 legitimately sends NaN in setpoints.
    if not math.isfinite(fv):
        return None
    return round(fv, decimals)
