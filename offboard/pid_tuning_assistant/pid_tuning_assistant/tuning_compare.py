"""
Side-by-side tuning comparison — current vs PX4 native autotune vs
the analyzer's model-based synthesis.

The whole point of the PX4-autotune integration is to let an operator judge the
firmware's result instead of trusting it blindly.  This module assembles, per
axis, up to three sets of rate/attitude gains together with the model each method
identified and its own goodness metric, so the numbers can be compared on equal
footing:

  1. **current**       — the gains already on the vehicle (loaded params/log);
  2. **px4_autotune**  — the gains PX4's ``mc_autotune_attitude_control`` would
                         apply, reproduced from the converged ARX(2,2,1)
                         coefficients it logged (GMVC, filtered-step excitation);
  3. **analyzer**      — the gains ``plant_estimator`` synthesizes from its own
                         2nd-order+delay / integrator+pole fit of the recorded
                         flight (loop-shaping to a PM/BW target).

READ-ONLY: this builds a report only.  It never writes gains — applying is the
qgc path's job.

Everything is duck-typed (``getattr``) so the builder does not pull in scipy /
the plant estimator at import time and stays unit-testable with plain stand-ins.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Any

# Per-axis PX4 parameter tag + the canonical gain-param order shown in a column.
_TAGS = {"roll": "ROLL", "pitch": "PITCH", "yaw": "YAW"}


def _gain_param_names(tag: str) -> List[str]:
    return [f"MC_{tag}RATE_K", f"MC_{tag}RATE_P", f"MC_{tag}RATE_I",
            f"MC_{tag}RATE_D", f"MC_{tag}_P"]


def _round(v: Optional[float], n: int = 5) -> Optional[float]:
    if v is None:
        return None
    try:
        fv = float(v)
    except (TypeError, ValueError):
        return None
    return round(fv, n)


def _current_gains(tag: str, params: Dict[str, float]) -> Optional[Dict[str, Optional[float]]]:
    names = _gain_param_names(tag)
    out = {n: _round(params.get(n)) for n in names}
    return out if any(v is not None for v in out.values()) else None


def _px4_autotune_gains(axis: str, tag: str, ident, params: Dict[str, float],
                        sigma: Optional[float]) -> Optional[Dict[str, Optional[float]]]:
    """Gains PX4 would apply, reproduced from the logged converged ARX coeffs."""
    coeff = getattr(ident, "coeff", None)
    model_dt = getattr(ident, "model_dt_s", None)
    if coeff is None:
        return None
    # Lazy import: keeps scipy out of the import path until a real comparison runs.
    from .autotune_offline import gains_from_autotune_coeffs
    res = gains_from_autotune_coeffs(axis, coeff, model_dt, sigma=sigma, current=params)
    if res is None:
        return None
    names = _gain_param_names(tag)
    mapped = res.get("params", {})
    out: Dict[str, Optional[float]] = {}
    for n in names:
        entry = mapped.get(n)
        out[n] = _round(entry["suggested"]) if entry else None
    out["_gate_pass"] = res.get("gate_pass")
    out["_gate_reason"] = res.get("gate_reason")
    return out


def _analyzer_gains(tag: str, synth) -> Optional[Dict[str, Optional[float]]]:
    """Rate gains from plant_estimator's synthesis (attitude P is not synthesized)."""
    if synth is None:
        return None
    return {
        f"MC_{tag}RATE_K": _round(getattr(synth, "K", None)),
        f"MC_{tag}RATE_P": _round(getattr(synth, "Kp", None)),
        f"MC_{tag}RATE_I": _round(getattr(synth, "Ki", None)),
        f"MC_{tag}RATE_D": _round(getattr(synth, "Kd", None)),
        f"MC_{tag}_P": None,   # analyzer does not synthesize the attitude loop
    }


def _autotune_model(ident) -> Optional[Dict[str, Any]]:
    model = getattr(ident, "model", None)
    if model is None:
        return None
    return {
        "type": "ARX(2,2,1)",
        "num": getattr(model, "num", None),
        "den": getattr(model, "den", None),
        "model_dt_s": getattr(model, "model_dt_s", None),
        "wn_hz": getattr(model, "wn_hz", None),
        "zeta": getattr(model, "zeta", None),
        "excitation": "filtered step (MC_AT_SYSID_AMP)",
    }


def _analyzer_model(plant_est) -> Optional[Dict[str, Any]]:
    pm = getattr(plant_est, "plant_model", None)
    if pm is None:
        return None
    wn = getattr(pm, "omega_n", None)
    wn_hz = (float(wn) / (2.0 * 3.141592653589793)) if wn else None
    return {
        "type": getattr(pm, "order", None),
        "wn_hz": _round(wn_hz, 3),
        "zeta": _round(getattr(pm, "zeta", None), 4),
        "delay_ms": _round((getattr(pm, "delay_s", 0.0) or 0.0) * 1000.0, 2),
        "fit_vaf": _round(getattr(pm, "fit_vaf", None), 1),
        "excitation": "recorded flight",
    }


def build_tuning_comparison(
    params: Optional[Dict[str, float]] = None,
    autotune_series=None,
    plant_estimations: Optional[List] = None,
    sigma_map: Optional[Dict[str, float]] = None,
) -> Dict[str, Any]:
    """Assemble the per-axis 3-column comparison report.

    Parameters
    ----------
    params            current gains (loaded .params or ULog snapshot).
    autotune_series   AutotuneStatusSeries or None.
    plant_estimations list of PlantEstimation (analyzer synthesis) or None.
    sigma_map         optional per-axis desired rise time (s) for reproducing the
                      PX4 gains; defaults to the firmware defaults inside
                      gains_from_autotune_coeffs when absent.

    The result always contains all three axes.  Each axis lists which columns are
    populated (``available``) so the UI degrades gracefully: with no autotune data
    it renders a 2-column current-vs-analyzer table instead of failing.
    """
    params = params or {}
    sigma_map = sigma_map or {}
    at_axes = getattr(autotune_series, "axes", {}) if autotune_series is not None else {}

    # Index analyzer estimations by axis.
    synth_by_axis: Dict[str, Any] = {}
    plant_by_axis: Dict[str, Any] = {}
    for pe in (plant_estimations or []):
        ax = getattr(pe, "axis", None)
        if ax:
            synth_by_axis[ax] = getattr(pe, "synthesized", None)
            plant_by_axis[ax] = pe

    axes_out: List[Dict[str, Any]] = []
    for axis, tag in _TAGS.items():
        ident = at_axes.get(axis)
        synth = synth_by_axis.get(axis)
        plant_est = plant_by_axis.get(axis)

        gains = {
            "current": _current_gains(tag, params),
            "px4_autotune": (_px4_autotune_gains(axis, tag, ident, params,
                                                 sigma_map.get(axis))
                             if ident is not None else None),
            "analyzer": _analyzer_gains(tag, synth),
        }
        models = {
            "px4_autotune": _autotune_model(ident) if ident is not None else None,
            "analyzer": _analyzer_model(plant_est) if plant_est is not None else None,
        }
        quality = {
            "px4_fitness": (getattr(ident, "fitness", None)
                            if ident is not None else None),
            "analyzer_vaf": (models["analyzer"] or {}).get("fit_vaf"),
        }
        available = [k for k, v in gains.items() if v is not None]
        axes_out.append({
            "axis": axis,
            "param_order": _gain_param_names(tag),
            "gains": gains,
            "models": models,
            "quality": quality,
            "available": available,
        })

    return {
        "axes": axes_out,
        "sources": {
            "current": bool(params),
            "px4_autotune": bool(at_axes),
            "analyzer": bool(synth_by_axis),
        },
        "disclaimer": (
            "Gains are advisory. PX4 autotune uses a filtered-step excitation and "
            "an ARX(2,2,1) SISO model; the analyzer fits a 2nd-order+delay / "
            "integrator+pole model to whatever motion the recording contained. "
            "Change ONE parameter at a time and verify in SITL before flying. "
            "This view is read-only — apply via the qgc PX4 Autotune page."
        ),
    }
