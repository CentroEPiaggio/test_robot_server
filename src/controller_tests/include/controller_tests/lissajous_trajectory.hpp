// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <array>

#include <Eigen/Core>

namespace controller_tests {

constexpr int kNumJoints = 7;
using Vector7d = Eigen::Matrix<double, kNumJoints, 1>;

/// One sample of a twice-differentiable joint trajectory.
struct TrajectoryPoint
{
  Vector7d position{Vector7d::Zero()};
  Vector7d velocity{Vector7d::Zero()};
  Vector7d acceleration{Vector7d::Zero()};
};

/**
 * @brief Joint-space Lissajous test trajectory with a smooth approach.
 *
 * The motion is made of three phases, each continuous in position, velocity
 * and acceleration so that nothing is excited that the trajectory did not ask
 * for:
 *
 *  1. **approach**, over `approach_duration`: a quintic polynomial from the
 *     pose the robot happens to be in to the fixed centre `centre`, starting
 *     and ending at rest. This is what makes two runs comparable: whatever the
 *     robot was doing before, the Lissajous always starts from the same pose.
 *
 *  2. **Lissajous**, over `run_duration`:
 *
 *         q_d,i(t) = centre_i + s(t) * A_i * sin(2*pi*f_i*t + phi_i)
 *
 *     with an envelope s(t) that rises from 0 to 1 over `ramp_duration` and
 *     falls back to 0 over the last `ramp_duration`, again as a quintic with
 *     zero first and second derivative at both ends. Because s(0) = s'(0) =
 *     s''(0) = 0, the phase offsets phi_i can be chosen freely without
 *     producing a jump at the junction.
 *
 *  3. **hold**: the centre pose at rest, indefinitely.
 *
 * The class is a pure function of time: it holds no ROS dependency and no
 * state beyond its configuration, which makes it directly testable.
 */
class LissajousTrajectory
{
public:
  struct Configuration
  {
    Vector7d centre{Vector7d::Zero()};
    Vector7d amplitude{Vector7d::Zero()};   ///< [rad]
    Vector7d frequency{Vector7d::Zero()};   ///< [Hz]
    Vector7d phase{Vector7d::Zero()};       ///< [rad]
    double approach_duration{5.0};          ///< [s]
    double ramp_duration{2.0};              ///< [s]
    double run_duration{30.0};              ///< [s]
  };

  enum class Phase { kApproach, kLissajous, kHold };

  /// @param start The pose the approach starts from.
  void configure(const Configuration & configuration, const Vector7d & start);

  /// @param t Time since the start of the motion [s].
  TrajectoryPoint sample(double t) const;

  /// Which phase @p t falls into.
  Phase phaseAt(double t) const;

  /// Total duration of approach plus Lissajous [s].
  double totalDuration() const
  {
    return configuration_.approach_duration + configuration_.run_duration;
  }

private:
  /// Quintic interpolation from 0 to 1 with zero velocity and acceleration at
  /// both ends, and its first two time derivatives.
  static void quinticScaling(double t, double duration, double & s, double & ds, double & dds);

  Configuration configuration_;
  Vector7d start_{Vector7d::Zero()};
};

}  // namespace controller_tests
