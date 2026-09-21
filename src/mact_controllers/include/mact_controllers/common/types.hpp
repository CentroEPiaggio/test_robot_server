// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <rclcpp/time.hpp>

namespace mact_controllers {

constexpr int kNumJoints = 7;

using Vector7d = Eigen::Matrix<double, kNumJoints, 1>;
/// A regressor: 7 rows, one column per dynamic parameter (100 for the Franka).
using RegressorMatrix = Eigen::Matrix<double, kNumJoints, Eigen::Dynamic>;

/**
 * @brief One desired-trajectory sample, as received from the reference topic.
 *
 * The arrival time is kept alongside the values because
 * trajectory_msgs/JointTrajectoryPoint carries no stamp, and the controller
 * needs to know how old the reference is to detect a stalled generator.
 */
struct TrajectorySample
{
  Vector7d q_d{Vector7d::Zero()};
  Vector7d dq_d{Vector7d::Zero()};
  Vector7d ddq_d{Vector7d::Zero()};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  /// The generator's own motion clock, from time_from_start [s].
  double time_from_start{0.0};
  bool valid{false};
};

/**
 * @brief The motion on which the regressors have to be evaluated.
 *
 * Y_r is evaluated on (q, dq, dq_d, ddq_d) and Y on (q, dq, ddq), following
 * eq. (2) of the extended abstract.
 */
struct MotionSample
{
  Vector7d q{Vector7d::Zero()};
  Vector7d dq{Vector7d::Zero()};
  Vector7d ddq{Vector7d::Zero()};
  Vector7d q_d{Vector7d::Zero()};
  Vector7d dq_d{Vector7d::Zero()};
  Vector7d ddq_d{Vector7d::Zero()};
};

/**
 * @brief Model quantities for one control cycle.
 *
 * Filled by the concrete controller: from a linked Thunder instance, or from
 * the matrices most recently published by the Robot Server. Everything is
 * preallocated by resize() so that the update loop never allocates.
 */
struct ModelTerms
{
  RegressorMatrix regressor_r;  ///< Y_r(q, dq, dq_d, ddq_d), Slotine-Li
  RegressorMatrix regressor;    ///< Y(q, dq, ddq), standard
  RegressorMatrix regressor_g;  ///< reg_G(q), gravity only

  bool has_regressor_r{false};
  bool has_regressor{false};
  bool has_regressor_g{false};

  /// Age of the freshest model data [ms]. Always 0 when the model is linked.
  double age_ms{0.0};

  void resize(int num_parameters)
  {
    regressor_r.setZero(kNumJoints, num_parameters);
    regressor.setZero(kNumJoints, num_parameters);
    regressor_g.setZero(kNumJoints, num_parameters);
  }

  void invalidate()
  {
    has_regressor_r = false;
    has_regressor = false;
    has_regressor_g = false;
    age_ms = 0.0;
  }
};

}  // namespace mact_controllers
