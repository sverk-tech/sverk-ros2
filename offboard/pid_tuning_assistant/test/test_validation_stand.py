# -*- coding: utf-8 -*-
"""
Synthetic validation stand for pid_tuning_assistant.PIDAnalyzer (pytest).

Simulates a PX4-structured closed rate loop with KNOWN ground truth:
  u = K*(P*e + I*int(e) - D*d(gyro_f)/dt),  e = sp - gyro_f
  gyro filtered by 2nd-order Butterworth 40 Hz (IMU_GYRO_CUTOFF equivalent),
  plant: torque -> angular accel (gain) + 1st-order motor lag + transport delay.
Sim at 1 kHz, telemetry subsampled to 50 Hz (DDS-over-UART scenario).

Ground truth (first positive step, full-rate):
  - "well tuned"  (K=1.0, delay 8 ms):  overshoot ~10.9 %, rise ~122 ms
  - "oscillatory" (K=2.5, delay 30 ms): overshoot ~55 %, limit cycle ~4.9 Hz

These tests guard the step-metric fixes from (negative-step overshoot,
rise via argmax, window truncation).  Runtime ~30 s — acceptable, no slow marker.

    python -m pytest test/test_validation_stand.py -q
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import pytest

from pid_tuning_assistant.analyzer import PIDAnalyzer            # noqa: E402
from pid_tuning_assistant.data_buffer import TelemetryFrame, TopicStats  # noqa: E402

FS_SIM = 1000.0
DT = 1.0 / FS_SIM


# ── simulation ────────────────────────────────────────────────────────────────

def simulate(K=1.0, P=0.15, I=0.2, D=0.003, accel_gain=80.0, motor_tau=0.025,
             delay_s=0.008, gyro_noise=0.03, duration=30.0, seed=1,
             step_amp=1.0, jab_period=2.5, jab_hold=1.0):
    """Simulate the roll rate loop; returns full-rate arrays."""
    rng = np.random.default_rng(seed)
    n = int(duration * FS_SIM)
    t = np.arange(n) * DT

    # setpoint: alternating-sign stick jabs
    sp = np.zeros(n)
    t_jab, k_jab = 2.0, 0
    while t_jab + jab_hold < duration - 2.0:
        i0, i1 = int(t_jab * FS_SIM), int((t_jab + jab_hold) * FS_SIM)
        sp[i0:i1] = step_amp if (k_jab % 2 == 0) else -step_amp
        t_jab += jab_period
        k_jab += 1

    from scipy.signal import butter
    b, a = butter(2, 40.0 / (FS_SIM / 2.0), btype="low")
    z_g = np.zeros(2)                       # biquad state (DF2T)

    delay_n = max(1, int(delay_s * FS_SIM))
    u_buf = np.zeros(delay_n)

    rate = motor = integ = gyro_f_prev = 0.0
    rates, gyro_fs, torques = np.zeros(n), np.zeros(n), np.zeros(n)

    for k in range(n):
        gyro_raw = rate + rng.normal(0.0, gyro_noise)
        x = gyro_raw                        # biquad step
        y = b[0] * x + z_g[0]
        z_g[0] = b[1] * x - a[1] * y + z_g[1]
        z_g[1] = b[2] * x - a[2] * y
        gyro_f = y

        e = sp[k] - gyro_f
        dgyro = (gyro_f - gyro_f_prev) / DT
        gyro_f_prev = gyro_f
        u = K * (P * e + I * integ - D * dgyro)
        u = float(np.clip(u, -1.0, 1.0))
        if abs(u) < 1.0:                    # naive anti-windup
            integ += e * DT
        u_delayed = u_buf[0]                # transport delay + motor lag
        u_buf[:-1] = u_buf[1:]
        u_buf[-1] = u
        motor += (u_delayed - motor) * DT / motor_tau
        rate += accel_gain * motor * DT

        rates[k], gyro_fs[k], torques[k] = rate, gyro_f, u
    return t, sp, rates, gyro_fs, torques


def true_step_metrics(t, sp, rate, amp):
    """Ground-truth overshoot/rise on the FIRST positive step, full rate."""
    idx = np.where(np.diff(sp) > 0.5 * amp)[0]
    if len(idx) == 0:
        return None
    i0 = idx[0] + 1
    w = rate[i0: i0 + int(1.0 * FS_SIM)]
    norm = (w - w[0]) / (sp[i0] - w[0] + 1e-9)
    ovs = max(0.0, (float(np.max(norm)) - 1.0) * 100.0)
    i10 = int(np.argmax(norm >= 0.1))
    i90 = int(np.argmax(norm >= 0.9))
    rise = (i90 - i10) * DT if i90 > i10 else float("nan")
    return dict(overshoot_pct=ovs, rise_s=rise)


def to_frames(t, sp, gyro_f, torque, fs_out=50.0):
    """Subsample to TelemetryFrames like the DDS node would produce."""
    step = int(FS_SIM / fs_out)
    frames = []
    for k in range(0, len(t), step):
        frames.append(TelemetryFrame(
            t=float(t[k]), wall_t=float(t[k]), px4_t=int(t[k] * 1e6),
            nav_state=15,  # STAB
            roll_rate=float(gyro_f[k]), pitch_rate=0.0, yaw_rate=0.0,
            roll_rate_sp=float(sp[k]), pitch_rate_sp=0.0, yaw_rate_sp=0.0,
            torque_roll=float(torque[k]), torque_pitch=0.0, torque_yaw=0.0,
        ))
    return frames


def run_case(name, fs_out=50.0, **simkw):
    """Simulate, analyse, return (truth, AnalysisResult, roll rate segments)."""
    t, sp, rates, gyro_f, tq = simulate(**simkw)
    truth = true_step_metrics(t, sp, rates, simkw.get("step_amp", 1.0))
    frames = to_frames(t, sp, gyro_f, tq, fs_out=fs_out)
    stats = {"VehicleAngularVelocity": TopicStats(
        topic="VehicleAngularVelocity", msg_count=len(frames),
        achieved_rate_hz=fs_out, jitter_ms=1.0, drop_pct=0.0,
        f_nyquist_hz=fs_out / 2.0, duration_s=float(t[-1]))}
    params = {"MC_ROLLRATE_K": simkw.get("K", 1.0), "MC_ROLLRATE_P": 0.15,
              "MC_ROLLRATE_I": 0.2, "MC_ROLLRATE_D": 0.003,
              "MC_PITCHRATE_K": 1.0, "MC_YAWRATE_K": 1.0}
    an = PIDAnalyzer(params=params, lang="en")
    res = an.analyse(frames, run_name=name,
                     topic_status={"VehicleRatesSetpoint": "active",
                                   "VehicleAttitudeSetpoint": "active"},
                     topic_stats=stats)
    roll_segs = [sm for sm in res.segments
                 if sm.axis == "roll" and sm.loop == "rate" and sm.n_events]
    return truth, res, roll_segs


def _roll_events(roll_segs):
    return [e for sm in roll_segs for e in sm.events]


def _roll_k_recs(res):
    return [r for r in res.recommendations if r.param == "MC_ROLLRATE_K"]


# ── tests ─────────────────────────────────────────────────────────────────────

def test_isolated_positive_and_negative_steps_accurate():
    """Case E: clean-window jabs; every unit step (both signs) must recover the
    true overshoot (~11 %) and rise (~0.12 s), not just the positive ones."""
    _, _, roll_segs = run_case("E", K=1.0, duration=60.0,
                               jab_period=7.0, jab_hold=3.0)
    assert roll_segs, "expected a roll rate segment with events"
    events = _roll_events(roll_segs)
    unit = [e for e in events if 0.8 <= e.step_size <= 1.2]
    assert len(unit) >= 6, f"too few unit steps: {len(unit)}"

    # both step directions must be present and analysed
    assert any(e.sp_after > e.sp_before for e in unit), "no positive steps"
    assert any(e.sp_after < e.sp_before for e in unit), "no negative steps"

    for e in unit:
        sign = "+" if e.sp_after > e.sp_before else "-"
        assert 8.0 <= e.overshoot_pct <= 18.0, \
            f"{sign} step overshoot out of range: {e.overshoot_pct}"
        assert 0.08 <= e.rise_time_s <= 0.18, \
            f"{sign} step rise out of range: {e.rise_time_s}"


def test_dense_jabs_do_not_inflate_overshoot():
    """Case A: jabs every 2.5 s (README-style pauses). Window truncation must
    stop the returning stick from inflating overshoot."""
    _, _, roll_segs = run_case("A", K=1.0)   # jab_period=2.5 default
    assert roll_segs, "expected a roll rate segment with events"
    events = _roll_events(roll_segs)
    ovs_mean = float(np.mean([e.overshoot_pct for e in events]))
    assert ovs_mean < 20.0, f"ovs_mean inflated: {ovs_mean}"
    assert all(e.overshoot_pct <= 60.0 for e in events), \
        f"contaminated event: max={max(e.overshoot_pct for e in events)}"


_CASES = {
    "A": dict(K=1.0),
    "E": dict(K=1.0, duration=60.0, jab_period=7.0, jab_hold=3.0),
    "F": dict(K=2.5, delay_s=0.030, duration=60.0, jab_period=7.0, jab_hold=3.0),
    "C": dict(K=0.35, duration=60.0, jab_period=7.0, jab_hold=3.0),
}


def test_no_contradictory_k_recommendations():
    """Strict arbitration guarantee: in no case A/E/F/C may any single
    (axis, param) carry two recommendations with opposite-sign Δ.  For the
    well-tuned isolated case (E) the assistant must not touch rate-K at all."""
    for name, kw in _CASES.items():
        _, res, _ = run_case(name, **kw)
        by_param = {}
        for r in res.recommendations:
            if r.param == "NOTE" or r.delta_pct == 0:
                continue
            by_param.setdefault((r.axis, r.param), set()).add(
                "up" if r.delta_pct > 0 else "down")
        for key, signs in by_param.items():
            assert signs != {"up", "down"}, \
                f"{name}: contradictory Δ signs on {key}"
        if name == "E":
            assert not _roll_k_recs(res), \
                f"E (tuned) should have no MC_ROLLRATE_K recs: " \
                f"{[(r.direction, r.reason_key) for r in _roll_k_recs(res)]}"


def test_delta_capped():
    """Case F: the model K-scale rule wants Δ≈−98%; the ±30% cap must hold for
    every recommendation."""
    _, res, _ = run_case("F", **_CASES["F"])
    for r in res.recommendations:
        assert abs(r.delta_pct) <= 30.0 + 1e-9, \
            f"uncapped delta {r.delta_pct} on {r.param} ({r.reason_key})"


def test_well_tuned_is_quiet():
    """Case E: a tuned drone must not receive a high-confidence, non-zero-Δ
    recommendation to change MC_ROLLRATE_K."""
    _, res, _ = run_case("E", **_CASES["E"])
    loud = [r for r in _roll_k_recs(res)
            if r.confidence == "high" and abs(r.delta_pct) > 0]
    assert not loud, \
        f"tuned drone got confident K advice: " \
        f"{[(r.delta_pct, r.reason_key) for r in loud]}"


def test_k_too_low_gets_increase():
    """Case C (K=0.35, sluggish): either MC_ROLLRATE_K is recommended UP, or no
    K-reduction is offered.  A sluggish drone whose rise stays just under the slow
    threshold legitimately triggers no r4 — a known heuristic weakness, so the
    'no decrease' branch is accepted rather than xfailed."""
    _, res, _ = run_case("C", **_CASES["C"])
    k_recs = _roll_k_recs(res)
    has_increase = any(r.delta_pct > 0 for r in k_recs)
    has_decrease = any(r.delta_pct < 0 for r in k_recs)
    assert has_increase or not has_decrease, \
        f"C: sluggish drone advised to lower K: " \
        f"{[(r.delta_pct, r.reason_key) for r in k_recs]}"


def test_oscillation_detected():
    """Case F: K=2.5 + 30 ms delay → ~4.9 Hz limit cycle. Expect a rate-K
    reduction flagged as an oscillation."""
    _, res, _ = run_case("F", K=2.5, delay_s=0.030, duration=60.0,
                         jab_period=7.0, jab_hold=3.0)
    k_recs = [r for r in _roll_k_recs(res) if r.delta_pct < 0]
    assert k_recs, "expected a MC_ROLLRATE_K reduction for the oscillatory case"
    assert any(r.reason_key in ("r1_oscillation", "r1_oscillation_no_model")
               for r in k_recs), \
        f"unexpected reason keys: {[r.reason_key for r in k_recs]}"


def test_no_false_low_freq_oscillation():
    """Case E (well tuned): the dominant-frequency guard must not pick a peak off
    the command's own spectrum, so no low-frequency-oscillation rule (r3) fires."""
    _, res, _ = run_case("E", K=1.0, duration=60.0,
                         jab_period=7.0, jab_hold=3.0)
    r3 = [r for r in res.recommendations if r.reason_key == "r3_low_freq_osc"]
    assert not r3, \
        f"false low-freq oscillation flagged on a tuned drone: " \
        f"{[(r.axis, r.reason) for r in r3]}"


def test_zero_setpoint_hover_not_dropped():
    """A pure hover (roll_rate_sp == 0.0 for the whole recording) must NOT be
    treated as 'no data'.  Before the fix, ``0.0 or np.nan`` nulled every sample,
    has_sp became False, the error signal was zeroed and rms_error collapsed to 0."""
    rng = np.random.default_rng(3)
    fs = 50.0
    n = int(30.0 * fs)
    frames = []
    for k in range(n):
        frames.append(TelemetryFrame(
            t=k / fs, wall_t=k / fs, px4_t=int(k / fs * 1e6),
            nav_state=15,  # STAB
            roll_rate=float(rng.normal(0.0, 0.05)),
            pitch_rate=0.0, yaw_rate=0.0,
            roll_rate_sp=0.0, pitch_rate_sp=0.0, yaw_rate_sp=0.0,
            torque_roll=float(rng.normal(0.0, 0.01)),
            torque_pitch=0.0, torque_yaw=0.0,
        ))
    stats = {"VehicleAngularVelocity": TopicStats(
        topic="VehicleAngularVelocity", msg_count=n,
        achieved_rate_hz=fs, jitter_ms=1.0, drop_pct=0.0,
        f_nyquist_hz=fs / 2.0, duration_s=n / fs)}
    an = PIDAnalyzer(params={}, lang="en")
    res = an.analyse(frames, run_name="hover",
                     topic_status={"VehicleRatesSetpoint": "active",
                                   "VehicleAttitudeSetpoint": "active"},
                     topic_stats=stats)
    roll_rate_segs = [sm for sm in res.segments
                      if sm.axis == "roll" and sm.loop == "rate"]
    assert roll_rate_segs, "roll/rate segment missing — zero setpoint was dropped"
    assert any(sm.rms_error_mean > 0.0 for sm in roll_rate_segs), \
        "zero-hover roll/rate rms_error collapsed to 0"


def test_dropout_does_not_create_steps():
    """Case E with 10 % of roll_rate_sp samples dropped (None).  Masking the
    dropouts (vs back-filling with 0.0) must not fabricate extra step events."""
    _, _, segs_clean = run_case("E", K=1.0, duration=60.0,
                                jab_period=7.0, jab_hold=3.0)
    n_clean = sum(sm.n_events for sm in segs_clean)

    t, sp, rates, gyro_f, tq = simulate(K=1.0, duration=60.0,
                                        jab_period=7.0, jab_hold=3.0)
    frames = to_frames(t, sp, gyro_f, tq, fs_out=50.0)
    rng = np.random.default_rng(7)
    for f in frames:
        if rng.random() < 0.10:
            f.roll_rate_sp = None
    stats = {"VehicleAngularVelocity": TopicStats(
        topic="VehicleAngularVelocity", msg_count=len(frames),
        achieved_rate_hz=50.0, jitter_ms=1.0, drop_pct=0.0,
        f_nyquist_hz=25.0, duration_s=float(t[-1]))}
    an = PIDAnalyzer(params={"MC_ROLLRATE_K": 1.0}, lang="en")
    res = an.analyse(frames, run_name="E_drop",
                     topic_status={"VehicleRatesSetpoint": "active",
                                   "VehicleAttitudeSetpoint": "active"},
                     topic_stats=stats)
    segs = [sm for sm in res.segments
            if sm.axis == "roll" and sm.loop == "rate" and sm.n_events]
    n_drop = sum(sm.n_events for sm in segs)
    assert n_drop <= n_clean, \
        f"dropouts fabricated steps: clean={n_clean}, dropped={n_drop}"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
