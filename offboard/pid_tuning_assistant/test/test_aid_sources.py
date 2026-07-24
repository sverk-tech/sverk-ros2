"""
Unit tests for aid_source_analyzer — registry-driven EKF2 aid-source analysis
.  Registry-driven synthetic data, no real.ulg / pyulog needed.

    python -m pytest test/test_aid_sources.py -v

Covers: registry field-name integrity (no typos / nonexistent fields), the
gnss/gps prefix trap, flow reject dropouts, range test-ratio stats, inactive
sources producing no recommendations, top-level fake_pos coasting, and the
guarantee that no recommendation names a v1.15.4-nonexistent parameter.
"""
import os
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import pytest

from pid_tuning_assistant.aid_source_analyzer import (
    AID_SOURCES, AID_SOURCE_BY_KEY, NONEXISTENT_PARAMS, get_source,
    AidSourceSeries, SharedContext, AidSourceBundle, AidSourceAnalyzer,
    axis_test_ratio_stats, fusion_dropouts, _top_level_alarm,
)


# Verified PX4 v1.15.4 field/flag names (from msg/EstimatorInnovations.msg and
# EstimatorStatusFlags.msg — the reference in).  A registry entry
# naming anything outside these sets is a bug (typo or nonexistent field).
VALID_INNOV_FIELDS = {
    "gps_hvel", "gps_vvel", "gps_hpos", "gps_vpos",
    "ev_hvel", "ev_vvel", "ev_hpos", "ev_vpos",
    "rng_vpos", "baro_vpos", "aux_hvel", "flow", "terr_flow",
    "heading", "mag_field", "gravity", "drag",
    "airspeed", "beta", "hagl", "hagl_rate",
}
VALID_CS_FLAGS = {
    "cs_ev_pos", "cs_ev_vel", "cs_ev_yaw", "cs_ev_hgt",
    "cs_opt_flow", "cs_rng_hgt", "cs_rng_stuck", "cs_rng_fault",
    "cs_rng_kin_consistent", "cs_gps", "cs_gps_hgt", "cs_gps_yaw",
    "cs_gps_yaw_fault", "cs_baro_hgt", "cs_gnd_effect",
    "cs_mag", "cs_mag_hdg", "cs_mag_3d", "cs_mag_dec", "cs_mag_fault",
    "cs_mag_field_disturbed", "cs_mag_aligned_in_flight",
    "cs_mag_heading_consistent", "cs_fake_pos", "cs_fake_hgt",
    "cs_inertial_dead_reckoning", "cs_wind_dead_reckoning",
}
VALID_REJECT_FLAGS = {
    "reject_hor_vel", "reject_ver_vel", "reject_hor_pos", "reject_ver_pos",
    "reject_yaw", "reject_airspeed", "reject_sideslip", "reject_hagl",
    "reject_optflow_x", "reject_optflow_y",
}


# ── (1) registry integrity ────────────────────────────────────────────────────

def test_registry_field_names_exist():
    """Every innov field, cs flag and reject flag in the registry is a real
    PX4 v1.15.4 field name (guards against typos / hallucinated fields)."""
    seen_keys = set()
    for src in AID_SOURCES:
        assert src.key not in seen_keys, f"duplicate key {src.key}"
        seen_keys.add(src.key)
        for _role, base, _dim in src.innov_fields:
            assert base in VALID_INNOV_FIELDS, f"{src.key}: bad innov field {base}"
        for flag in src.cs_flags:
            assert flag in VALID_CS_FLAGS, f"{src.key}: bad cs flag {flag}"
        assert src.primary_cs in src.cs_flags
        for rej in src.reject_fields:
            assert rej in VALID_REJECT_FLAGS, f"{src.key}: bad reject flag {rej}"
        # No registry param is one of the known-nonexistent names.
        for pname in src.params:
            assert pname not in NONEXISTENT_PARAMS, f"{src.key}: {pname} does not exist"
        for pname in (*src.gate_params, *src.noise_params, *src.pos_offset_params):
            assert pname not in NONEXISTENT_PARAMS


def test_expected_sources_present():
    """EV + optical_flow + rng_hgt are first-class; gps/baro/mag registered."""
    for key in ("ev", "optical_flow", "rng_hgt", "gps", "baro", "mag"):
        assert get_source(key) is not None


# ── (2) gnss / gps prefix trap ────────────────────────────────────────────────

def test_gnss_prefix_trap():
    """GPS reads innovations from gps_*, but its aid_src topics are gnss_* —
    estimator_aid_src_gps_* does NOT exist and must never appear."""
    gps = get_source("gps")
    innov_bases = {base for _r, base, _d in gps.innov_fields}
    assert innov_bases == {"gps_hpos", "gps_vpos", "gps_hvel", "gps_vvel"}
    aid_names = {name for _r, name, _d in gps.aid_src_topics}
    assert all(n.startswith("estimator_aid_src_gnss_") for n in aid_names)
    assert not any("estimator_aid_src_gps_" in n for n in aid_names)


# ── (3) flow reject dropout ───────────────────────────────────────────────────

def test_flow_reject_dropout():
    """reject_optflow_x/y true runs → correct flow dropout intervals."""
    flow = get_source("optical_flow")
    t_flags = np.array([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    #  reject_optflow_x:  F  F  T  T  F  F  F   → one 2.0 s interval
    rx = np.array([0, 0, 1, 1, 0, 0, 0], dtype=bool)
    ry = np.zeros(7, dtype=bool)
    series = AidSourceSeries(
        key="optical_flow", t_flags=t_flags,
        cs={"cs_opt_flow": np.ones(7, dtype=bool)},
        reject={"reject_optflow_x": rx, "reject_optflow_y": ry},
    )
    rep = fusion_dropouts(flow, series, SharedContext())
    rej = [iv for iv in rep.intervals if iv.cause == "reject_optflow_x"]
    assert len(rej) == 1
    assert rej[0].duration_s == pytest.approx(2.0)
    assert rep.max_duration_s == pytest.approx(2.0)


# ── (4) range test-ratio stats ────────────────────────────────────────────────

def test_rng_test_ratio_stats():
    """A known fraction of rng_vpos test_ratio > 1 → correct %>1."""
    rng = get_source("rng_hgt")
    n = 50
    tr = np.full(n, 0.4)
    tr[:15] = 1.7                  # 30 % rejected
    series = AidSourceSeries(
        key="rng_hgt",
        t_innov=np.arange(n, dtype=float) * 0.5,
        innov={"vpos": np.zeros(n)},
        innov_tr={"vpos": tr},
    )
    stats = axis_test_ratio_stats(rng, series)
    vpos = next(st for st in stats if st.axis == "rng_hgt_vpos")
    assert vpos.pct_over_1 == pytest.approx(30.0)
    assert vpos.n == n


# ── helper: fake parse result for the orchestrator ────────────────────────────

def _fake_parse(bundle: AidSourceBundle, params: dict = None):
    meta = types.SimpleNamespace(
        source_name="synthetic.ulg", duration_s=bundle.shared.duration_s or 20.0,
        params=params or {}, params_all=params or {},
    )
    return types.SimpleNamespace(metadata=meta, aid_source_bundle=bundle)


# ── (5) inactive source → no recommendations ──────────────────────────────────

def test_inactive_source_no_recs():
    """A source with CTRL=0 and no fusion → 'inactive', zero real recs."""
    bundle = AidSourceBundle()
    bundle.shared.duration_s = 20.0
    # gps: CTRL 0, no cs true, no innovation data.
    bundle.sources["gps"] = AidSourceSeries(
        key="gps", cs={"cs_gps": np.zeros(10, dtype=bool)})
    res = AidSourceAnalyzer().analyse(_fake_parse(bundle, {"EKF2_GPS_CTRL": 0}))
    gps = next(r for r in res.sources if r.key == "gps")
    assert gps.active is False
    # Only NOTE-type entries (inactive marker + gps hint), no tunable params.
    assert all(r.param == "NOTE" for r in gps.recommendations)
    assert any(r.reason_key == "rec_inactive" for r in gps.recommendations)


# ── (6) top-level fake_pos coasting ───────────────────────────────────────────

def test_fake_pos_coasting_flagged():
    """cs_fake_pos with sustained runs → top-level coasting alarm."""
    t_flags = np.arange(0.0, 10.0, 1.0)
    fake = np.zeros(len(t_flags), dtype=bool)
    fake[3:7] = True              # 3..7 s coasting
    ctx = SharedContext(
        duration_s=10.0, t_flags=t_flags, cs_fake_pos=fake,
        cs_inertial_dead_reckoning=np.zeros(len(t_flags), dtype=bool),
    )
    alarm = _top_level_alarm(ctx)
    assert alarm.coasting is True
    assert len(alarm.fake_pos_intervals) == 1
    assert alarm.fake_pos_intervals[0].duration_s == pytest.approx(4.0)
    assert alarm.coasting_fraction_pct > 0


# ── (7) no nonexistent params ever suggested ──────────────────────────────────

def test_no_nonexistent_params_suggested():
    """Across a rich synthetic bundle, no recommendation names a param that does
    not exist on v1.15.4 (EKF2_EV_GATE / EVA_GATE / REQ_FIX / MAG_DECL /
    MAG_YAWLIM)."""
    n = 40
    t_innov = np.arange(n, dtype=float) * 0.5
    t_flags = np.linspace(0, 20, n)
    bundle = AidSourceBundle()
    bundle.shared = SharedContext(
        duration_s=20.0, t_flags=t_flags,
        cs_fake_pos=np.zeros(n, dtype=bool),
        cs_inertial_dead_reckoning=np.zeros(n, dtype=bool),
        t_lpos=t_innov, lpos_vel=np.column_stack(
            [np.sin(t_innov), np.cos(t_innov), np.zeros(n)]),
    )
    # EV with rejections + a rich aid_src (drives gate + NEES + delay recs).
    ev_tr = np.full(n, 0.6)
    ev_tr[:10] = 1.4
    aid = {
        "t": t_innov,
        "observation": np.column_stack([np.cumsum(np.sin(t_innov)) * 0.5,
                                        np.cumsum(np.cos(t_innov)) * 0.5]),
        "innovation": np.column_stack([np.full(n, 0.3), np.full(n, 0.3)]),
        "innovation_variance": np.column_stack([np.full(n, 0.01), np.full(n, 0.01)]),
        "innovation_rejected": np.zeros(n, dtype=bool),
        "fused": np.ones(n, dtype=bool),
    }
    bundle.sources["ev"] = AidSourceSeries(
        key="ev", t_innov=t_innov, t_flags=t_flags,
        innov={"hpos": np.column_stack([np.full(n, 0.12), np.zeros(n)]),
               "vpos": np.zeros(n)},
        innov_tr={"hpos": np.column_stack([ev_tr, ev_tr])},
        cs={"cs_ev_pos": np.ones(n, dtype=bool)},
        aid_src=aid, aid_src_dim=2, aid_src_name="estimator_aid_src_ev_pos",
    )
    res = AidSourceAnalyzer().analyse(_fake_parse(
        bundle, {"EKF2_EV_CTRL": 15, "EKF2_EV_DELAY": 0}))
    for report in res.sources:
        for rec in report.recommendations:
            assert rec.param not in NONEXISTENT_PARAMS, rec.param
            for bad in NONEXISTENT_PARAMS:
                assert bad not in (rec.reason or ""), f"{bad} leaked into reason"
    # EV was active and produced at least one tunable recommendation.
    ev = next(r for r in res.sources if r.key == "ev")
    assert ev.active is True
    assert any(r.param != "NOTE" for r in ev.recommendations)
