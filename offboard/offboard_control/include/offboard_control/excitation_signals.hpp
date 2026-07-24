// Header-only excitation-signal library for closed-loop system ID.
//
// Generates windowed, (mostly) zero-mean disturbance waveforms sampled
// value-by-value in the flight-path timer (>= 100-200 Hz). NO ROS / PX4 / Eigen
// dependencies on purpose: pure <cmath>/<vector> so it is unit-testable with
// gtest and cheap on the real-time path. Formulas follow the excitation context
// (docs/dev/pid_tuning_prompts/70_excitation_context.md, section "Формулы").
//
// The caller decides units: `amplitude` is rad/s for the rate loop (Pattern B)
// or rad for the attitude loop (Pattern A). Every waveform is wrapped in a Tukey
// window so it ramps in/out through zero, and no waveform ever exceeds
// |amplitude| after windowing/scaling (hard safety cap, verified by gtest).

#ifndef OFFBOARD_CONTROL__EXCITATION_SIGNALS_HPP_
#define OFFBOARD_CONTROL__EXCITATION_SIGNALS_HPP_

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <vector>

namespace excitation
{

enum class Waveform { Step, Doublet, ThreeTwoOneOne, Chirp, Multisine, Prbs };

struct ExcitationParams
{
  Waveform waveform = Waveform::Doublet;
  double amplitude = 0.0;    // rad/s (rate loop) or rad (attitude loop)
  double duration = 0.0;     // s; 0 = auto from the waveform
  double f_min = 0.5;        // Hz, low band edge  (chirp/multisine/prbs)
  double f_max = 10.0;       // Hz, high band edge (chirp/multisine/prbs)
  double base_period = 0.0;  // s, dt for 3211/doublet; 0 = 1/(2*f_max)
  uint32_t seed = 1;         // prbs register seed / multisine reproducibility
  double taper = 0.15;       // Tukey window fraction per end, [0..0.5]
};

// First-order washout (high-pass) for use in the node timer loop: subtracts a
// slowly tracking low-pass state so any residual DC does not accumulate tilt.
// `state` must persist across calls; `dt`/`tau` in seconds.
inline double washout(double raw, double & state, double dt, double tau = 0.2)
{
  if (tau <= 0.0) {
    return raw;
  }
  const double alpha = dt / (tau + dt);
  state += alpha * (raw - state);
  return raw - state;
}

class ExcitationSignal
{
public:
  explicit ExcitationSignal(const ExcitationParams & p)
  : p_(p)
  {
    taper_ = std::min(std::max(p_.taper, 0.0), 0.5);
    resolveDuration();          // sets duration_ (and prbs dims tclk_/n_/N_)
    if (p_.waveform == Waveform::Multisine) {
      buildMultisine();         // needs duration_
      computeScale();           // needs window() + multisineRaw()
    }
    if (p_.waveform == Waveform::Prbs) {
      buildPrbs();              // needs n_/N_
    }
  }

  // Effective window length in seconds.
  double duration() const { return duration_; }

  // True once t is past the window; value() returns 0 there.
  bool finished(double t) const { return t > duration_; }

  // Instantaneous sample at time t (t measured from window start).
  double value(double t) const
  {
    if (t < 0.0 || t > duration_) {
      return 0.0;
    }
    double core = 0.0;
    switch (p_.waveform) {
      case Waveform::Step:           core = p_.amplitude;      break;  // shape from window
      case Waveform::Doublet:        core = doublet(t);        break;
      case Waveform::ThreeTwoOneOne: core = threeTwoOneOne(t); break;
      case Waveform::Chirp:          core = chirp(t);          break;
      case Waveform::Multisine:      core = ms_scale_ * multisineRaw(t); break;
      case Waveform::Prbs:           core = prbs(t);           break;
    }
    return core * window(t);
  }

  // --- introspection (for the analyzer/tests; not on the hot path) ---
  double clockPeriod() const { return tclk_; }                 // prbs T_clk
  std::size_t sequenceLength() const { return seq_.size(); }   // prbs N
  const std::vector<double> & sequence() const { return seq_; }

private:
  static constexpr double kPi = 3.14159265358979323846;

  // --- shape parameters -----------------------------------------------------

  double dtBase() const
  {
    if (p_.base_period > 0.0) {
      return p_.base_period;
    }
    if (p_.f_max > 0.0) {
      return 1.0 / (2.0 * p_.f_max);
    }
    return 0.05;
  }

  void resolveDuration()
  {
    switch (p_.waveform) {
      case Waveform::Doublet:
        duration_ = 2.0 * dtBase();
        break;
      case Waveform::ThreeTwoOneOne:
        duration_ = 7.0 * dtBase();
        break;
      case Waveform::Prbs:
        resolvePrbsDims();
        break;
      case Waveform::Step:
      case Waveform::Chirp:
      case Waveform::Multisine:
      default:
        duration_ = (p_.duration > 0.0) ? p_.duration : 2.0;
        break;
    }
  }

  // --- Tukey window ---------------------------------------------------------
  // Cosine taper of length taper_*duration_ at each end, flat (==1) in between.
  double window(double t) const
  {
    if (taper_ <= 0.0) {
      return 1.0;
    }
    const double r = taper_ * duration_;
    if (r <= 0.0) {
      return 1.0;
    }
    if (t < r) {
      return 0.5 * (1.0 + std::cos(kPi * (t / r - 1.0)));
    }
    if (t > duration_ - r) {
      const double s = (t - (duration_ - r)) / r;
      return 0.5 * (1.0 + std::cos(kPi * s));
    }
    return 1.0;
  }

  // --- waveforms ------------------------------------------------------------

  double doublet(double t) const
  {
    const double dt = dtBase();
    if (t < dt) {
      return p_.amplitude;
    }
    if (t < 2.0 * dt) {
      return -p_.amplitude;
    }
    return 0.0;
  }

  // 3-2-1-1: switch signs at 3dt/5dt/6dt, total 7dt. Standard bang-bang +-A has
  // a +A*dt DC bias (durations +:4dt, -:3dt). To be zero-mean by construction
  // AND respect the peak cap, the positive level is scaled to 0.75*A so that
  // 4*0.75A == 3*A; the negative level stays at the cap (-A → peak == A).
  double threeTwoOneOne(double t) const
  {
    const double dt = dtBase();
    const double pos = 0.75 * p_.amplitude;
    const double neg = -p_.amplitude;
    if (t < 3.0 * dt) { return pos; }
    if (t < 5.0 * dt) { return neg; }
    if (t < 6.0 * dt) { return pos; }
    if (t < 7.0 * dt) { return neg; }
    return 0.0;
  }

  // CIFER exponential chirp: theta(t) integrated in closed form.
  // omega(t)=w_min + C2*(exp(C1*t/T)-1)*(w_max-w_min); C2 chosen so the sweep
  // reaches ~f_max at t=T. theta = w_min*t + C2*(w_max-w_min)*((T/C1)*(e-1)-t).
  double chirp(double t) const
  {
    const double T = duration_;
    const double wmin = 2.0 * kPi * p_.f_min;
    const double wmax = 2.0 * kPi * p_.f_max;
    const double C1 = 4.0;
    const double C2 = 0.0187;
    const double e = std::exp(C1 * t / T) - 1.0;
    const double theta = wmin * t + C2 * (wmax - wmin) * ((T / C1) * e - t);
    return p_.amplitude * std::sin(theta);
  }

  // --- multisine (Schroeder-phased) ----------------------------------------

  void buildMultisine()
  {
    const double T = duration_;
    ms_T_ = (T > 0.0) ? T : 1.0;
    int kmin = static_cast<int>(std::ceil(p_.f_min * ms_T_));
    int kmax = static_cast<int>(std::floor(p_.f_max * ms_T_));
    if (kmin < 1) {
      kmin = 1;
    }
    ms_k_.clear();
    ms_phase_.clear();
    for (int k = kmin; k <= kmax; ++k) {
      ms_k_.push_back(k);
    }
    const double F = static_cast<double>(ms_k_.size());
    for (int k : ms_k_) {
      // Schroeder phase: phi_k = -pi*k*(k-1)/F  → low crest factor.
      const double phi = -kPi * static_cast<double>(k) *
        static_cast<double>(k - 1) / (F > 0.0 ? F : 1.0);
      ms_phase_.push_back(phi);
    }
  }

  double multisineRaw(double t) const
  {
    double s = 0.0;
    for (std::size_t i = 0; i < ms_k_.size(); ++i) {
      const double f = static_cast<double>(ms_k_[i]) / ms_T_;
      s += std::cos(2.0 * kPi * f * t + ms_phase_[i]);
    }
    return s;
  }

  // Scale the whole multisine so the peak of the *windowed* signal == amplitude.
  void computeScale()
  {
    double peak = 0.0;
    const int Ns = 8192;
    for (int i = 0; i <= Ns; ++i) {
      const double t = duration_ * static_cast<double>(i) / static_cast<double>(Ns);
      const double v = std::fabs(multisineRaw(t) * window(t));
      if (v > peak) {
        peak = v;
      }
    }
    ms_scale_ = (peak > 1e-12) ? (p_.amplitude / peak) : 0.0;
  }

  // --- PRBS (maximum-length Galois LFSR) -----------------------------------

  // Galois feedback masks (right-shift, XOR mask when the shifted-out bit is 1)
  // for primitive polynomials of degree n → maximal length 2^n-1. Verified: each
  // mask returns the register to its seed after exactly 2^n-1 steps, the output
  // is balanced (#+1 - #-1 == 1) and its periodic autocorrelation is -1/N.
  static uint32_t galoisMask(int n)
  {
    switch (n) {
      case 2:  return 0x0003u;
      case 3:  return 0x0005u;
      case 4:  return 0x0009u;
      case 5:  return 0x0012u;
      case 6:  return 0x0021u;
      case 7:  return 0x0041u;
      case 8:  return 0x008Eu;
      case 9:  return 0x0108u;
      case 10: return 0x0204u;
      case 11: return 0x0402u;
      case 12: return 0x0829u;
      case 13: return 0x100Du;
      case 14: return 0x2015u;
      case 15: return 0x4001u;
      case 16: return 0x8016u;
      default: return 0x0041u;
    }
  }

  void resolvePrbsDims()
  {
    const double fmax = (p_.f_max > 0.0) ? p_.f_max : 10.0;
    tclk_ = 1.0 / (3.0 * fmax);
    if (p_.duration > 0.0) {
      int n = 2;
      while (n < 16 &&
        static_cast<double>((1u << n) - 1u) * tclk_ < p_.duration)
      {
        ++n;
      }
      n_ = n;
    } else {
      n_ = 7;
    }
    n_ = std::min(std::max(n_, 2), 16);
    N_ = (1u << n_) - 1u;
    duration_ = (p_.duration > 0.0) ? p_.duration
                                    : static_cast<double>(N_) * tclk_;
  }

  void buildPrbs()
  {
    const uint32_t mask = galoisMask(n_);
    uint32_t reg = p_.seed & N_;
    if (reg == 0u) {
      reg = 1u;
    }
    seq_.resize(N_);
    for (uint32_t i = 0; i < N_; ++i) {
      seq_[i] = (reg & 1u) ? 1.0 : -1.0;   // output the outgoing LSB, mapped to +-1
      const uint32_t lsb = reg & 1u;
      reg >>= 1;
      if (lsb) {
        reg ^= mask;
      }
      reg &= N_;
    }
  }

  double prbs(double t) const
  {
    if (seq_.empty()) {
      return 0.0;
    }
    long idx = static_cast<long>(std::floor(t / tclk_));
    if (idx < 0) {
      idx = 0;
    }
    const uint32_t k = static_cast<uint32_t>(idx % static_cast<long>(N_));
    return p_.amplitude * seq_[k];
  }

  // --- state ----------------------------------------------------------------
  ExcitationParams p_;
  double taper_ = 0.15;
  double duration_ = 0.0;

  // prbs
  double tclk_ = 0.0;
  int n_ = 7;
  uint32_t N_ = 127u;
  std::vector<double> seq_;

  // multisine
  double ms_T_ = 1.0;
  std::vector<int> ms_k_;
  std::vector<double> ms_phase_;
  double ms_scale_ = 1.0;
};

}  // namespace excitation

#endif  // OFFBOARD_CONTROL__EXCITATION_SIGNALS_HPP_
