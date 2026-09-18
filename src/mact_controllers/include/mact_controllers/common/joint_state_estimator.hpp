// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>

#include "mact_controllers/common/low_pass_filter.hpp"

namespace mact_controllers {

/**
 * @brief Velocity and acceleration estimator for a 7-DoF arm.
 *
 * The hardware exposes joint position and velocity but never acceleration,
 * while the regressors need q, dq and ddq. This class turns the raw velocity
 * of the state interface into the pair (dq, ddq) used by the control law:
 *
 *     dq  = LPF_dq(dq_raw)
 *     ddq = LPF_ddq( (dq_raw[k] - dq_raw[k-1]) / dt )
 *
 * The acceleration is differentiated from the *raw* velocity and filtered
 * afterwards, rather than differentiating the already filtered signal: the two
 * are equivalent for a linear filter, but this way the two cut-off frequencies
 * are independent, and ddq (which needs the heavier filtering) does not inherit
 * the lag of the dq filter on top of its own.
 *
 * Fixed-size and allocation-free: safe to call from a real-time loop.
 */
class JointStateEstimator {
 public:
  static constexpr int kNumJoints = 7;
  using Vector7d = Eigen::Matrix<double, kNumJoints, 1>;

  /**
   * @param dq_cutoff_hz  Cut-off of the velocity filter [Hz]; <= 0 disables it.
   * @param ddq_cutoff_hz Cut-off of the acceleration filter [Hz].
   */
  void configure(double dq_cutoff_hz, double ddq_cutoff_hz);

  /// Drop all history. Call on activation, before the first update().
  void reset();

  /**
   * @brief Consume one raw velocity sample.
   * @param dq_raw Velocity as read from the state interface.
   * @param dt     Control period [s].
   *
   * The first call after a reset() only primes the filters: dq is set to
   * dq_raw and ddq is left at zero, since no finite difference is available
   * yet.
   */
  void update(const Vector7d& dq_raw, double dt);

  /// Filtered joint velocity.
  const Vector7d& dq() const { return dq_; }
  /// Filtered joint acceleration.
  const Vector7d& ddq() const { return ddq_; }

 private:
  LowPassFilter<kNumJoints> dq_filter_;
  LowPassFilter<kNumJoints> ddq_filter_;

  Vector7d dq_{Vector7d::Zero()};
  Vector7d ddq_{Vector7d::Zero()};
  Vector7d previous_dq_raw_{Vector7d::Zero()};

  bool has_previous_sample_{false};
};

}  // namespace mact_controllers
