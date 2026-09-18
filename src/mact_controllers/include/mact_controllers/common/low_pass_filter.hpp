// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <cmath>

#include <Eigen/Core>

namespace mact_controllers {

/**
 * @brief First-order (single-pole) IIR low-pass filter for a fixed-size vector.
 *
 * The recursion is the exponential moving average
 *
 *     y[k] = y[k-1] + alpha * (u[k] - y[k-1]),
 *
 * with the smoothing factor derived from a cut-off frequency so that the
 * configuration file stays physical:
 *
 *     tau   = 1 / (2*pi*f_c),      alpha = dt / (tau + dt).
 *
 * alpha is recomputed whenever the sample time changes, which keeps the
 * bandwidth correct if the controller period jitters. A non-positive cut-off
 * frequency disables the filter (alpha = 1, i.e. pass-through), which is handy
 * to compare against the unfiltered signal without touching the code.
 *
 * Everything is fixed-size and allocation-free, so it is safe to call from the
 * real-time update() of a controller.
 */
template <int N>
class LowPassFilter {
 public:
  using Vector = Eigen::Matrix<double, N, 1>;

  LowPassFilter() { reset(); }

  /// @param cutoff_hz Cut-off frequency [Hz]; <= 0 disables the filter.
  void setCutoff(double cutoff_hz) {
    cutoff_hz_ = cutoff_hz;
    // Force a recomputation of alpha on the next update().
    last_dt_ = -1.0;
  }

  double cutoff() const { return cutoff_hz_; }

  /// Clear the internal state; the next sample is passed through unchanged.
  void reset() {
    value_.setZero();
    initialised_ = false;
    last_dt_ = -1.0;
    alpha_ = 1.0;
  }

  /// Restart the filter from a known value (e.g. on activation).
  void reset(const Vector& value) {
    value_ = value;
    initialised_ = true;
    last_dt_ = -1.0;
    alpha_ = 1.0;
  }

  /**
   * @brief Filter one sample.
   * @param input Raw measurement.
   * @param dt    Time since the previous call [s]; must be > 0.
   * @return The filtered value (also available through value()).
   */
  const Vector& update(const Vector& input, double dt) {
    // The very first sample defines the state: starting from zero would
    // otherwise inject a step of the size of the signal itself.
    if (!initialised_) {
      value_ = input;
      initialised_ = true;
      return value_;
    }

    if (dt != last_dt_) {
      alpha_ = computeAlpha(cutoff_hz_, dt);
      last_dt_ = dt;
    }

    value_ += alpha_ * (input - value_);
    return value_;
  }

  const Vector& value() const { return value_; }
  bool initialised() const { return initialised_; }

 private:
  static double computeAlpha(double cutoff_hz, double dt) {
    if (!(cutoff_hz > 0.0) || !(dt > 0.0)) {
      return 1.0;  // disabled: pass-through
    }
    const double tau = 1.0 / (2.0 * M_PI * cutoff_hz);
    return dt / (tau + dt);
  }

  Vector value_;
  double cutoff_hz_{0.0};
  double alpha_{1.0};
  double last_dt_{-1.0};
  bool initialised_{false};
};

}  // namespace mact_controllers
