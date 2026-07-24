"""
Abstract input-source layer.

                    ┌─────────────────┐
                    │  InputSource    │  ← ABC
                    └────────┬────────┘
                             │
               ┌─────────────┴──────────────┐
               │                            │
    ┌──────────▼──────────┐      ┌──────────▼──────────┐
    │  DdsRecordingSource  │      │   UlogFileSource     │
    │  (existing buffer)   │      │   (pyulog, optional) │
    └─────────────────────┘      └─────────────────────┘

Both produce the same TelemetryFrame list and SourceMetadata so the
downstream analyzer.py / hifi_analyzer.py are source-agnostic.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any

from .data_buffer import TelemetryFrame, TopicStats


@dataclass
class SourceCapabilities:
    """What algorithms are available given this source's data quality."""
    # Rate tier
    high_rate_available: bool = False       # primary topic ≥ 200 Hz
    freq_domain_available: bool = False     # ≥ 50 Hz  (FRF possible)
    # Sensor tier
    vibration_available: bool = False       # raw sensor_gyro present
    raw_sensor_available: bool = False      # pre-filter gyro available
    esc_available: bool = False             # ESC RPM data present
    actuator_data_available: bool = False   # actuator motor output present
    # Parameter tier
    params_available: bool = False          # param snapshot present
    sysid_hifi_available: bool = False      # high_rate + params → plant recovery
    # VIO / EV (external-vision) tier — see vio_analyzer.py.
    # Source-agnostic: any EV fused by EKF2 (ArUco/VPE, LIO, 2D scan-match,
    # camera VIO, mocap) lands in the same estimator topics.
    vio_basic_available: bool = False       # estimator_status_flags + estimator_innovations
    vio_aidsrc_available: bool = False      # estimator_aid_src_ev_* present (extended)
    vio_delay_estimable: bool = False       # high-rate EV source → EV_DELAY estimate
    # Excitation characterisation
    excitation_type: str = "unknown"        # "step" | "sweep" | "mixed" | "unknown"
    excitation_quality: str = "unknown"     # "good" | "marginal" | "poor" | "unknown"


@dataclass
class SourceMetadata:
    """Normalised metadata produced by every InputSource."""
    source_type: str                         # "dds" | "ulog"
    source_name: str                         # run_name or filename
    duration_s: float
    n_frames: int
    topic_stats: Dict[str, TopicStats]
    capabilities: SourceCapabilities
    params: Dict[str, float]                 # PID params for analysis engine
    params_all: Dict[str, Any] = field(default_factory=dict)   # full snapshot
    firmware_version: str = ""
    hw_system_id: str = ""
    ulog_path: str = ""                      # set only for ulog source


class InputSource(ABC):
    @abstractmethod
    def get_frames(self) -> List[TelemetryFrame]: ...

    @abstractmethod
    def get_metadata(self) -> SourceMetadata: ...


class DdsRecordingSource(InputSource):
    """
    Thin wrapper that turns an already-recorded RecordingBuffer session into
    the standard InputSource interface.  No new logic — the heavy lifting is
    still in TelemetryNode + RecordingBuffer.
    """

    def __init__(self, frames: List[TelemetryFrame], metadata: SourceMetadata):
        self._frames = frames
        self._metadata = metadata

    def get_frames(self) -> List[TelemetryFrame]:
        return self._frames

    def get_metadata(self) -> SourceMetadata:
        return self._metadata

    @staticmethod
    def build_metadata(
        frames: List[TelemetryFrame],
        topic_stats: Dict[str, TopicStats],
        params: Dict[str, float],
        run_name: str,
        duration_s: float,
    ) -> SourceMetadata:
        """Factory for building DDS metadata after stop_recording()."""
        prim = topic_stats.get("VehicleAngularVelocity")
        rate = prim.achieved_rate_hz if prim else 0.0
        caps = SourceCapabilities(
            high_rate_available=rate >= 200.0,
            freq_domain_available=rate >= 50.0,
            vibration_available=False,       # raw sensor not available over DDS
            raw_sensor_available=False,
            esc_available=False,
            actuator_data_available=bool(
                topic_stats.get("ActuatorMotors", TopicStats("ActuatorMotors")).msg_count > 0
            ),
            params_available=bool(params),
            sysid_hifi_available=False,      # never enough rate over UART DDS
            excitation_type="unknown",
            excitation_quality="unknown",
        )
        return SourceMetadata(
            source_type="dds",
            source_name=run_name,
            duration_s=duration_s,
            n_frames=len(frames),
            topic_stats=topic_stats,
            capabilities=caps,
            params=params,
        )
