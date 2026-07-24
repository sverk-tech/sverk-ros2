"""Reference-based (unbiased) closed-loop identification.

Synthetic ground truth: a known 2nd-order plant G in closed loop with a PI
controller, driven by an external chirp reference r, with output noise. Because
the injected r is uncorrelated with the noise, Ĝ = Φ_ry/Φ_ru must recover the
true plant more accurately than the setpoint-based H1 = Suy/Suu (which is biased
when the internal setpoint correlates with the noise via feedback).
"""
import numpy as np
import pytest
from scipy import signal as sig

from pid_tuning_assistant.hifi_analyzer import (
    compute_frf, compute_frf_reference,
)


# ── synthetic closed-loop plant ────────────────────────────────────────────────

WN = 20.0          # plant natural freq (rad/s)  ~3.2 Hz
ZETA = 0.5
KP, KI = 0.8, 4.0  # PI controller
FS = 400.0         # Hz
DT = 1.0 / FS
T_END = 40.0


def _true_plant(f_hz):
    """True plant G(j2πf) = wn^2 / ((jw)^2 + 2ζwn(jw) + wn^2)."""
    w = 2.0 * np.pi * np.asarray(f_hz)
    s = 1j * w
    return WN**2 / (s * s + 2.0 * ZETA * WN * s + WN**2)


def _simulate(noise_std, seed=0):
    """Closed loop: e=r-y, u=PI(e), y=G(u)+n. Returns t, r, u, y (all 1-D)."""
    rng = np.random.default_rng(seed)
    n = int(T_END * FS)
    t = np.arange(n) * DT

    # External reference: exponential chirp 0.3 → 12 Hz, unit amplitude.
    r = 1.0 * sig.chirp(t, f0=0.3, f1=12.0, t1=T_END, method="logarithmic")

    # Discretize plant (Tustin) → difference equation. cont2discrete → (num, den, dt).
    bd, ad, _ = sig.cont2discrete(
        ([WN**2], [1.0, 2.0 * ZETA * WN, WN**2]), DT, method="bilinear")
    bd = bd.ravel()
    # Controller PI C(s)=Kp+Ki/s = (Kp·s + Ki)/s → num=[Kp, Ki], den=[1, 0], Tustin.
    bc, ac, _ = sig.cont2discrete(([KP, KI], [1.0, 0.0]), DT, method="bilinear")
    bc = bc.ravel(); ac = ac.ravel()

    # Manual per-sample closed-loop integration (feedback forbids batch lfilter).
    na = len(ad); nb = len(bd)
    ug = np.zeros(max(na, nb) + 2)   # plant input history
    yg = np.zeros(na + 2)            # plant output history
    ec = np.zeros(len(ac) + 2)       # controller input history
    uc = np.zeros(len(ac) + 2)       # controller output history

    u = np.zeros(n)
    y = np.zeros(n)
    noise = rng.normal(0.0, noise_std, n)
    y_prev = 0.0

    for k in range(n):
        e = r[k] - y_prev
        # controller difference eq: ac[0]*uc = sum(bc*ec) - sum(ac[1:]*uc_hist)
        ec = np.roll(ec, 1); ec[0] = e
        acc = np.dot(bc, ec[:len(bc)]) - np.dot(ac[1:], uc[:len(ac) - 1])
        uk = acc / ac[0]
        uc = np.roll(uc, 1); uc[0] = uk

        # plant difference eq
        ug = np.roll(ug, 1); ug[0] = uk
        acc_y = np.dot(bd, ug[:len(bd)]) - np.dot(ad[1:], yg[:len(ad) - 1])
        yk = acc_y / ad[0]
        yg = np.roll(yg, 1); yg[0] = yk

        yk_meas = yk + noise[k]
        u[k] = uk
        y[k] = yk_meas
        y_prev = yk_meas

    return t, r, u, y


def _mag_error_over_band(frf, f_lo=1.0, f_hi=10.0):
    """Mean |dB error| of estimated |G| vs true plant, over valid coherent bins."""
    errs = []
    for p in frf.bode:
        if not p.valid or not (f_lo <= p.freq_hz <= f_hi):
            continue
        true_db = 20.0 * np.log10(np.abs(_true_plant(p.freq_hz)) + 1e-12)
        errs.append(abs(p.mag_db - true_db))
    return float(np.mean(errs)) if errs else float("inf")


# ── tests ──────────────────────────────────────────────────────────────────────

def test_reference_beats_setpoint_under_noise():
    """Ĝ=Φ_ry/Φ_ru recovers |G| more accurately than H1=Suy/Suu with noise.

    Noise 0.4 makes the closed-loop bias of the setpoint estimate clearly visible
    (robust 8/8 across seeds, ≥2.5 dB margin) while the reference estimate stays
    accurate; at very low noise the two converge (both approach the true plant).
    """
    t, r, u, y = _simulate(noise_std=0.4, seed=1)
    nperseg = 2048

    frf_ref = compute_frf_reference(t, r, u, y, FS, "roll", "rate",
                                    f_nyquist_hz=FS / 2, nperseg=nperseg)
    frf_sp = compute_frf(t, u, y, FS, "roll", "rate",
                         f_nyquist_hz=FS / 2, nperseg=nperseg)

    assert frf_ref is not None and frf_sp is not None
    assert frf_ref.method == "reference_indirect"
    assert frf_sp.method == "setpoint_h1"

    err_ref = _mag_error_over_band(frf_ref)
    err_sp = _mag_error_over_band(frf_sp)
    # Both should produce coherent bins in-band.
    assert np.isfinite(err_ref) and np.isfinite(err_sp)
    # Reference-based is unbiased → strictly smaller magnitude error.
    assert err_ref < err_sp, f"ref err {err_ref:.2f} dB !< setpoint err {err_sp:.2f} dB"
    # And it should be genuinely accurate, not merely less-bad.
    assert err_ref < 2.0, f"reference-based error {err_ref:.2f} dB too large"


def test_reference_recovers_dc_gain():
    """At low frequency |Ĝ| ≈ plant DC gain (0 dB here)."""
    t, r, u, y = _simulate(noise_std=0.1, seed=2)
    frf_ref = compute_frf_reference(t, r, u, y, FS, "roll", "rate",
                                    f_nyquist_hz=FS / 2, nperseg=2048)
    assert frf_ref is not None
    lows = [p.mag_db for p in frf_ref.bode if p.valid and 1.0 <= p.freq_hz <= 3.0]
    assert lows, "no valid low-frequency bins"
    assert abs(np.median(lows)) < 3.0  # DC gain ~0 dB


def _phase_error_over_band(frf, f_lo=2.0, f_hi=10.0):
    errs = []
    for p in frf.bode:
        if not p.valid or not (f_lo <= p.freq_hz <= f_hi):
            continue
        true_ph = np.degrees(np.angle(_true_plant(p.freq_hz)))
        d = abs(((p.phase_deg - true_ph) + 180.0) % 360.0 - 180.0)
        errs.append(d)
    return float(np.mean(errs)) if errs else float("inf")


def test_timestamp_desync_degrades_estimate():
    """The r/u/y shared-clock contract matters: putting y on a different clock
    (a fixed sample offset) injects a spurious phase and wrecks the estimate."""
    t, r, u, y = _simulate(noise_std=0.15, seed=5)
    y_desync = np.roll(y, 8)  # ~20 ms at 400 Hz: y on a clock offset from r/u

    frf_ok = compute_frf_reference(t, r, u, y, FS, "roll", "rate", FS / 2, nperseg=2048)
    frf_ds = compute_frf_reference(t, r, u, y_desync, FS, "roll", "rate", FS / 2, nperseg=2048)
    assert frf_ok is not None and frf_ds is not None

    ph_ok = _phase_error_over_band(frf_ok)
    ph_ds = _phase_error_over_band(frf_ds)
    # Desync adds tens of degrees of phase error the aligned estimate does not have.
    assert ph_ds > ph_ok + 20.0, f"desync phase err {ph_ds:.1f}° not >> aligned {ph_ok:.1f}°"


def test_richness_gate_flags_short_window():
    """A too-short inject window (few cycles of the lowest coherent freq) must not
    be reported as high confidence."""
    t, r, u, y = _simulate(noise_std=0.1, seed=6)
    n = int(1.2 * FS)  # ~1.2 s window
    frf = compute_frf_reference(t[:n], r[:n], u[:n], y[:n], FS, "roll", "rate",
                                FS / 2, nperseg=512)
    # Either rejected outright, or accepted but flagged low (not high).
    assert frf is None or frf.confidence in ("low", "medium")
    if frf is not None and frf.confidence == "low":
        assert "short" in frf.note or "bins" in frf.note


def test_segmentation_picks_dominant_run():
    """Two runs on the same axis are never merged — the largest is chosen."""
    from types import SimpleNamespace
    from pid_tuning_assistant.hifi_analyzer import _dominant_exc_run

    def mk(tag, axis="roll", phase="inject", ref=0.1):
        return SimpleNamespace(exc_axis=axis, exc_phase=phase, exc_ref=ref, exc_run_tag=tag)

    frames = [mk("runA") for _ in range(30)]
    frames += [mk("runB") for _ in range(10)]
    frames += [mk("runA", phase="settle_out") for _ in range(9)]  # not inject → ignored

    tag, n_runs = _dominant_exc_run(frames, "roll")
    assert tag == "runA" and n_runs == 2
    assert _dominant_exc_run(frames, "pitch") == (None, 0)


def test_ulog_excitation_extraction():
    """Offline path: reconstruct exc_* from a (merged) ULog excitation_sample, and
    warn gracefully when it is absent."""
    from pid_tuning_assistant.ulog_adapter import (
        _resolve_string_field, _ffill_str, _extract_excitation,
    )
    n = 10
    ts = (np.arange(n) * 1000).astype(np.int64)
    data = {"timestamp": ts, "ref": np.linspace(0.0, 0.1, n),
            "baseline": np.zeros(n), "loop": np.zeros(n)}
    for base, s in (("axis", "roll"), ("phase", "inject"), ("run_tag", "roll_doublet_1")):
        for j, ch in enumerate(s):
            data[f"{base}[{j}]"] = np.full(n, ord(ch))
        data[f"{base}[{len(s)}]"] = np.zeros(n)   # null terminator

    assert _resolve_string_field(data, "axis") == ["roll"] * n
    assert _ffill_str(np.array([500.0, 1500.0]), ts.astype(float),
                      ["roll"] * n) == ["roll", "roll"]

    class _DS:
        def __init__(self, name, d):
            self.name = name; self.data = d; self.multi_id = 0

    warns = []
    num, strc = _extract_excitation([_DS("excitation_sample", data)], ts, warns)
    assert "exc_ref" in num and len(num["exc_ref"]) == n
    assert strc["exc_axis"] == ["roll"] * n
    assert strc["exc_phase"] == ["inject"] * n
    assert strc["exc_run_tag"] == ["roll_doublet_1"] * n

    warns_absent = []
    num0, strc0 = _extract_excitation([], ts, warns_absent)
    assert num0 == {} and strc0 == {}
    assert any("No excitation_sample" in w for w in warns_absent)


def test_graceful_when_no_reference():
    """No reference samples (all NaN) → reference path returns None; caller falls
    back to the setpoint-based estimate (with its biased marker)."""
    t, r, u, y = _simulate(noise_std=0.1, seed=3)
    r_missing = np.full_like(r, np.nan)
    frf_ref = compute_frf_reference(t, r_missing, u, y, FS, "roll", "rate",
                                    f_nyquist_hz=FS / 2, nperseg=2048)
    assert frf_ref is None
    frf_sp = compute_frf(t, u, y, FS, "roll", "rate", f_nyquist_hz=FS / 2, nperseg=2048)
    assert frf_sp is not None
    assert frf_sp.method == "setpoint_h1"


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v", "-s"]))
