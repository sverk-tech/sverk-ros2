"""
QGC *.params file parser.

Format (tab-separated):
    <sysid>\t<compid>\t<PARAM_NAME>\t<value>\t<type>
Lines starting with # are comments / header.

type codes: 6=INT32, 9=FLOAT
"""
import re
from typing import Dict, Optional, Tuple

# Parameters we care about for PID analysis
RELEVANT_PARAMS = {
    # Rate gains — multicopter
    "MC_ROLLRATE_K",  "MC_ROLLRATE_P",  "MC_ROLLRATE_I",  "MC_ROLLRATE_D",
    "MC_PITCHRATE_K", "MC_PITCHRATE_P", "MC_PITCHRATE_I", "MC_PITCHRATE_D",
    "MC_YAWRATE_K",   "MC_YAWRATE_P",   "MC_YAWRATE_I",   "MC_YAWRATE_D",
    # Attitude gains
    "MC_ROLL_P",  "MC_PITCH_P",  "MC_YAW_P",
    # Integrator limits
    "MC_RR_INT_LIM", "MC_PR_INT_LIM", "MC_YR_INT_LIM",
    # Filters
    "MC_ROLLRATE_FF", "MC_PITCHRATE_FF",
    "IMU_GYRO_CUTOFF", "IMU_DGYRO_CUTOFF",
    # Position controller (context)
    "MPC_XY_P", "MPC_Z_P", "MPC_XY_VEL_P_ACC", "MPC_XY_VEL_I_ACC",
}


def parse_qgc_params(text: str) -> Dict[str, float]:
    """
    Parse QGC .params file text.
    Returns {PARAM_NAME: float_value} for all recognized numeric parameters.
    """
    params: Dict[str, float] = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = re.split(r"\t+|\s{2,}", line)
        if len(parts) < 4:
            continue
        try:
            name  = parts[2].strip()
            value = float(parts[3].strip())
            params[name] = value
        except (IndexError, ValueError):
            continue
    return params


def extract_pid_params(params: Dict[str, float]) -> Dict[str, float]:
    """Return only the PID-relevant subset."""
    return {k: v for k, v in params.items() if k in RELEVANT_PARAMS}


def parse_from_backup_response(resp) -> Dict[str, float]:
    """
    Parse params from offboard_interfaces/BackupParams.Response.

    resp.params is a list of offboard_interfaces/msg/Param with fields:
        .name       str
        .param_type int  (0=INT32, 1=FLOAT)
        .value      float64

    Returns {PARAM_NAME: float_value} for all parameters in the response.
    """
    params: Dict[str, float] = {}
    for p in resp.params:
        try:
            params[p.name] = float(p.value)
        except (AttributeError, ValueError, TypeError):
            continue
    return params


def current_value(params: Dict[str, float], name: str) -> Optional[float]:
    return params.get(name)


def describe_params(params: Dict[str, float]) -> str:
    lines = []
    axes = ["ROLL", "PITCH", "YAW"]
    for ax in axes:
        k  = params.get(f"MC_{ax}RATE_K",  "?")
        p  = params.get(f"MC_{ax}RATE_P",  "?")
        i  = params.get(f"MC_{ax}RATE_I",  "?")
        d  = params.get(f"MC_{ax}RATE_D",  "?")
        ap = params.get(f"MC_{ax}_P", params.get(f"MC_{ax[0]}P", "?"))
        lines.append(
            f"{ax:5s} rate: K={k}  P={p}  I={i}  D={d}  | att_P={ap}"
        )
    return "\n".join(lines)
