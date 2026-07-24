// Unit tests for the header-only excitation-signal library.
// Pure gtest, no ROS/PX4: validates each waveform against checkable ground
// truth (switch times, zero-mean, instantaneous frequency, spectrum, crest
// factor, PRBS maximal-length property) and the hard amplitude cap.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "offboard_control/excitation_signals.hpp"

using excitation::ExcitationParams;
using excitation::ExcitationSignal;
using excitation::Waveform;

namespace
{
constexpr double kPi = 3.14159265358979323846;

// Dense uniform samples of value(t) over [0, duration] (inclusive of ends).
std::vector<double> sample(const ExcitationSignal & s, int n)
{
  std::vector<double> v(n + 1);
  for (int i = 0; i <= n; ++i) {
    const double t = s.duration() * static_cast<double>(i) / static_cast<double>(n);
    v[i] = s.value(t);
  }
  return v;
}

// Trapezoidal integral of value(t) over [0, duration].
double integrate(const ExcitationSignal & s, int n)
{
  const double dt = s.duration() / static_cast<double>(n);
  double acc = 0.0;
  double prev = s.value(0.0);
  for (int i = 1; i <= n; ++i) {
    const double cur = s.value(dt * i);
    acc += 0.5 * (prev + cur) * dt;
    prev = cur;
  }
  return acc;
}

double maxAbs(const std::vector<double> & v)
{
  double m = 0.0;
  for (double x : v) {
    m = std::max(m, std::fabs(x));
  }
  return m;
}

// Times where value(t) changes sign (ignoring zeros), from a fine sweep.
std::vector<double> signChangeTimes(const ExcitationSignal & s, int n)
{
  std::vector<double> times;
  const double dt = s.duration() / static_cast<double>(n);
  double prev = 0.0;
  double prevT = 0.0;
  for (int i = 0; i <= n; ++i) {
    const double t = dt * i;
    const double v = s.value(t);
    if (v != 0.0) {
      if (prev != 0.0 && (v > 0.0) != (prev > 0.0)) {
        times.push_back(0.5 * (t + prevT));
      }
      prev = v;
      prevT = t;
    }
  }
  return times;
}

// One-sided DFT-bin magnitude (1/N normalized) at integer harmonic k over a
// signal sampled at N points across exactly one period.
double dftMag(const std::vector<double> & x, int k)
{
  const double N = static_cast<double>(x.size());
  double re = 0.0, im = 0.0;
  for (std::size_t n = 0; n < x.size(); ++n) {
    const double a = 2.0 * kPi * k * static_cast<double>(n) / N;
    re += x[n] * std::cos(a);
    im -= x[n] * std::sin(a);
  }
  return std::sqrt(re * re + im * im) / N;
}
}  // namespace

TEST(ExcitationSignals, StepTapered)
{
  ExcitationParams p;
  p.waveform = Waveform::Step;
  p.amplitude = 0.1;
  p.duration = 2.0;
  p.taper = 0.15;
  ExcitationSignal s(p);

  EXPECT_NEAR(s.value(0.0), 0.0, 1e-6);              // ramps in from zero
  EXPECT_NEAR(s.value(s.duration()), 0.0, 1e-6);     // ramps out to zero
  EXPECT_NEAR(s.value(s.duration() / 2.0), 0.1, 1e-9);  // flat middle == A
}

TEST(ExcitationSignals, DoubletZeroMean)
{
  ExcitationParams p;
  p.waveform = Waveform::Doublet;
  p.amplitude = 0.1;
  p.f_max = 5.0;      // dt = 0.1 s, duration = 0.2 s
  ExcitationSignal s(p);

  EXPECT_NEAR(s.duration(), 0.2, 1e-9);
  EXPECT_NEAR(integrate(s, 20000), 0.0, 1e-4);       // area-balanced (exact)
  EXPECT_EQ(signChangeTimes(s, 20000).size(), 1u);   // one sign flip
}

TEST(ExcitationSignals, ThreeTwoOneOneSwitchTimes)
{
  ExcitationParams p;
  p.waveform = Waveform::ThreeTwoOneOne;
  p.amplitude = 0.1;
  p.f_max = 3.0;                 // dt = 1/6 s
  ExcitationSignal s(p);

  const double dt = 1.0 / 6.0;
  EXPECT_NEAR(s.duration(), 7.0 * dt, 1e-9);

  const auto flips = signChangeTimes(s, 70000);
  ASSERT_EQ(flips.size(), 3u);
  EXPECT_NEAR(flips[0], 3.0 * dt, 2e-3);
  EXPECT_NEAR(flips[1], 5.0 * dt, 2e-3);
  EXPECT_NEAR(flips[2], 6.0 * dt, 2e-3);

  // Zero-mean by construction (Tukey adds only a small residual).
  EXPECT_NEAR(integrate(s, 70000), 0.0, 0.05 * p.amplitude * s.duration());
}

TEST(ExcitationSignals, ChirpInstantaneousFreq)
{
  ExcitationParams p;
  p.waveform = Waveform::Chirp;
  p.amplitude = 1.0;
  p.f_min = 0.5;
  p.f_max = 8.0;
  p.duration = 20.0;
  p.taper = 0.0;                 // clean zero crossings for detection
  ExcitationSignal s(p);

  // Local frequency between consecutive zero crossings = 1/(2*half-period).
  const auto zc = signChangeTimes(s, 400000);
  ASSERT_GE(zc.size(), 10u);

  std::vector<double> freqs;
  for (std::size_t i = 1; i < zc.size(); ++i) {
    freqs.push_back(0.5 / (zc[i] - zc[i - 1]));
  }
  // Starts near f_min, ends near f_max, increases overall.
  EXPECT_NEAR(freqs.front(), p.f_min, 0.35);
  EXPECT_GT(freqs.back(), 5.0);
  EXPECT_LT(freqs.back(), 10.0);

  const double firstThird = freqs[freqs.size() / 6];
  const double lastThird = freqs[freqs.size() * 5 / 6];
  EXPECT_GT(lastThird, firstThird * 3.0);
}

TEST(ExcitationSignals, MultisineSpectrum)
{
  ExcitationParams p;
  p.waveform = Waveform::Multisine;
  p.amplitude = 1.0;
  p.f_min = 1.0;
  p.f_max = 5.0;
  p.duration = 4.0;              // harmonics k = 4..20
  p.taper = 0.0;                 // exactly one period → clean bins
  ExcitationSignal s(p);

  const int Ns = 512;            // > 2*kmax (Nyquist)
  std::vector<double> x(Ns);
  for (int n = 0; n < Ns; ++n) {
    x[n] = s.value(p.duration * n / Ns);
  }

  double peak = 0.0;
  for (int k = 0; k <= 25; ++k) {
    peak = std::max(peak, dftMag(x, k));
  }
  ASSERT_GT(peak, 1e-6);

  // In-band harmonics carry energy; out-of-band bins are ~0.
  for (int k = 4; k <= 20; ++k) {
    EXPECT_GT(dftMag(x, k), 0.3 * peak) << "missing in-band harmonic k=" << k;
  }
  for (int k : {0, 1, 2, 3, 21, 22, 23, 24, 25}) {
    EXPECT_LT(dftMag(x, k), 0.05 * peak) << "leakage at out-of-band k=" << k;
  }

  // Amplitude cap: peak of the signal equals the requested amplitude.
  EXPECT_NEAR(maxAbs(sample(s, 20000)), p.amplitude, 0.02 * p.amplitude);
}

TEST(ExcitationSignals, MultisineCrestFactor)
{
  ExcitationParams p;
  p.waveform = Waveform::Multisine;
  p.amplitude = 1.0;
  p.f_min = 0.5;
  p.f_max = 10.0;
  p.duration = 8.0;
  p.taper = 0.0;
  ExcitationSignal s(p);

  const int Ns = 4096;
  double peak = 0.0, sumSq = 0.0;
  for (int n = 0; n < Ns; ++n) {
    const double v = s.value(p.duration * n / Ns);
    peak = std::max(peak, std::fabs(v));
    sumSq += v * v;
  }
  const double rms = std::sqrt(sumSq / Ns);
  const double crest = peak / rms;
  EXPECT_GT(crest, 1.4);
  EXPECT_LT(crest, 2.6);         // Schroeder phasing → low crest
}

TEST(ExcitationSignals, PrbsLevelsAndPeriod)
{
  ExcitationParams p;
  p.waveform = Waveform::Prbs;
  p.amplitude = 0.1;
  p.f_max = 10.0;                // T_clk = 1/30 s
  p.duration = 3.0;
  p.seed = 0xACE1u;
  p.taper = 0.0;                 // levels are exactly +-A without a window
  ExcitationSignal s(p);

  // Levels: sampling each clock centre yields exactly +A or -A.
  const double tclk = s.clockPeriod();
  for (double t = 0.5 * tclk; t < s.duration(); t += tclk) {
    const double v = s.value(t);
    EXPECT_TRUE(std::fabs(std::fabs(v) - p.amplitude) < 1e-12)
      << "off-level PRBS sample " << v << " at t=" << t;
  }

  // Maximal-length property: length N = 2^n-1, near-balanced, and the
  // periodic autocorrelation at any nonzero shift equals -1/N (flat spectrum).
  const auto & seq = s.sequence();
  const std::size_t N = seq.size();
  ASSERT_GT(N, 0u);

  int ones = 0;
  for (double b : seq) {
    if (b > 0.0) {
      ++ones;
    }
  }
  // 2^(n-1) plus vs 2^(n-1)-1 minus  →  #plus - #minus == 1.
  EXPECT_EQ(2 * ones - static_cast<int>(N), 1);

  double corr = 0.0;
  for (std::size_t i = 0; i < N; ++i) {
    corr += seq[i] * seq[(i + 1) % N];
  }
  corr /= static_cast<double>(N);
  EXPECT_NEAR(corr, -1.0 / static_cast<double>(N), 1e-9);
}

TEST(ExcitationSignals, AmplitudeCaps)
{
  const double A = 0.1;
  std::vector<ExcitationParams> cases;

  auto mk = [&](Waveform w, double dur, double fmax) {
    ExcitationParams p;
    p.waveform = w;
    p.amplitude = A;
    p.duration = dur;
    p.f_min = 0.5;
    p.f_max = fmax;
    p.taper = 0.15;
    return p;
  };
  cases.push_back(mk(Waveform::Step, 2.0, 10.0));
  cases.push_back(mk(Waveform::Doublet, 0.0, 5.0));
  cases.push_back(mk(Waveform::ThreeTwoOneOne, 0.0, 3.0));
  cases.push_back(mk(Waveform::Chirp, 15.0, 10.0));
  cases.push_back(mk(Waveform::Multisine, 8.0, 10.0));
  cases.push_back(mk(Waveform::Prbs, 5.0, 10.0));

  for (const auto & p : cases) {
    ExcitationSignal s(p);
    const double m = maxAbs(sample(s, 40000));
    EXPECT_LE(m, A * 1.001)
      << "waveform " << static_cast<int>(p.waveform) << " exceeds amplitude cap";
  }
}

TEST(ExcitationSignals, Finished)
{
  ExcitationParams p;
  p.waveform = Waveform::Chirp;
  p.amplitude = 1.0;
  p.duration = 5.0;
  ExcitationSignal s(p);

  EXPECT_FALSE(s.finished(s.duration() * 0.5));
  EXPECT_TRUE(s.finished(s.duration() + 0.01));
  EXPECT_EQ(s.value(s.duration() + 1.0), 0.0);
  EXPECT_EQ(s.value(-0.5), 0.0);
}
