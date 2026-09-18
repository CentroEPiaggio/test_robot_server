// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

namespace mact_controllers {

/**
 * @brief Parameter update law of the Modified Adaptive Computed Torque (MACT).
 *
 * Implements eq. (2) of "Thunder Dynamics: A Robot Server for ROS":
 *
 *     pi_hat_dot = R_p * ( Y_r^T * e_dot + gamma * Y^T * R_t * (tau - Y*pi_hat) )
 *
 * where Y_r = Y_r(q, dq, dq_d, ddq_d) is the Slotine-Li regressor evaluated on
 * the desired motion and Y = Y(q, dq, ddq) the standard regressor evaluated on
 * the actual one. The second term is a prediction error on the applied torque
 * and is skipped entirely when gamma == 0 (which also saves the cost of Y).
 *
 * R_p is diagonal and built from the structure of the Thunder regressor
 * parameter vector par_REG, which is a stack of one 10-element block per joint:
 *
 *     [ m, m*c_x, m*c_y, m*c_z, I_xx, I_xy, I_xz, I_yy, I_yz, I_zz ]
 *
 * so the gain of entry (link l, parameter k) is link_gains[l] * param_gains[k].
 * Mass, first moments and inertias differ by orders of magnitude, so a single
 * scalar gain is impractical; this factorisation keeps the configuration to
 * (number of links + 10) numbers while still letting whole links be frozen by
 * setting their link gain to zero.
 *
 * The integration is explicit Euler at the control period. Everything is
 * preallocated in configure(): update() performs no allocation.
 */
class AdaptationLaw {
 public:
  static constexpr int kNumJoints = 7;
  /// Number of parameters in one Thunder link block of par_REG.
  static constexpr int kParametersPerLink = 10;

  using Vector7d = Eigen::Matrix<double, kNumJoints, 1>;
  using RegressorMatrix = Eigen::Matrix<double, kNumJoints, Eigen::Dynamic>;

  /**
   * @brief Allocate and set the gains.
   *
   * @param num_parameters Size of par_REG (100 for the Franka model).
   * @param link_gains     One gain per link block; its size times
   *                       kParametersPerLink must equal num_parameters.
   * @param param_gains    One gain per parameter within a block (10 entries).
   * @param gamma          Weight of the prediction error term; 0 disables it.
   * @param r_t            Diagonal of R_t, one entry per joint.
   * @param error_message  Filled with the reason when the call returns false.
   * @return false if the sizes are inconsistent or a gain is negative.
   */
  bool configure(
    int num_parameters,
    const std::vector<double> & link_gains,
    const std::vector<double> & param_gains,
    double gamma,
    const std::vector<double> & r_t,
    std::string & error_message);

  /// Set the current estimate, e.g. from the value held by the server.
  void setEstimate(const Eigen::VectorXd & estimate);

  /**
   * @brief Integrate the update law by one step.
   *
   * @param regressor_r  Y_r, 7 x num_parameters.
   * @param error_rate   e_dot = dq_d - dq.
   * @param regressor    Y, 7 x num_parameters; ignored when gamma == 0.
   * @param torque       tau actually applied by the robot; ignored when
   *                     gamma == 0.
   * @param has_regressor True when @p regressor and @p torque are meaningful.
   * @param dt           Integration step [s].
   */
  void update(
    const RegressorMatrix & regressor_r,
    const Vector7d & error_rate,
    const RegressorMatrix & regressor,
    const Vector7d & torque,
    bool has_regressor,
    double dt);

  const Eigen::VectorXd & estimate() const { return estimate_; }
  /// Diagonal of R_p, exposed for logging and for tests.
  const Eigen::VectorXd & gains() const { return gain_diagonal_; }
  /// True when the prediction error term of eq. (2) is active.
  bool usesPredictionError() const { return gamma_ != 0.0; }
  int numParameters() const { return static_cast<int>(estimate_.size()); }

 private:
  Eigen::VectorXd estimate_;        ///< pi_hat
  Eigen::VectorXd gain_diagonal_;   ///< diag(R_p)
  Vector7d r_t_{Vector7d::Ones()};  ///< diag(R_t)
  double gamma_{0.0};

  // Scratch space, sized in configure() so that update() never allocates.
  Eigen::VectorXd rate_;            ///< pi_hat_dot
  Vector7d prediction_error_;       ///< tau - Y*pi_hat
};

}  // namespace mact_controllers
