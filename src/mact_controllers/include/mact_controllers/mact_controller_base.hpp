// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <mact_msgs/msg/mact_state.hpp>

#include "mact_controllers/common/adaptation_law.hpp"
#include "mact_controllers/common/gravity_source.hpp"
#include "mact_controllers/common/joint_state_estimator.hpp"
#include "mact_controllers/common/types.hpp"

namespace mact_controllers {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief Common skeleton of the two MACT controllers.
 *
 * Implements everything that must be *identical* between the in-process and
 * the server-based variant, so that a comparison between them measures the
 * architecture and nothing else: interface claiming, parameter handling, the
 * reference trajectory buffer, velocity/acceleration estimation, the control
 * law, the parameter update law, saturation, and diagnostics.
 *
 * The only thing left to the derived classes is *where the model comes from*,
 * through updateModel(); and what to do with the estimate afterwards, through
 * onCycleEnd().
 *
 * ### Control law
 *
 * With e = q_d - q, and using eq. (3) to assemble the model term from the
 * Slotine-Li regressor and the local copy of the estimate:
 *
 *     tau_model = Y_r(q, dq, dq_d, ddq_d) * pi_hat + k_v * e_dot + k_p * e
 *
 * `tau_model` is the torque the robot actually applies. What is written to the
 * command interface is
 *
 *     tau_cmd = tau_model                (simulation: Gazebo applies gravity)
 *     tau_cmd = tau_model - G_hat        (real robot: libfranka compensates
 *                                         gravity internally, so commanding it
 *                                         again would double it)
 *
 * with G_hat = reg_G(q) * pi_hat. The distinction matters for the update law
 * too: eq. (2) needs the torque the robot applies, i.e. `tau_model`, not the
 * gravity-free `tau_cmd`.
 *
 * ### Degraded operation
 *
 * The loop never waits for the model. When no usable regressor is available
 * (the server has not answered yet, or its samples went stale) the model term
 * is dropped and the controller commands the PD part alone, freezing the
 * parameter estimate until fresh data arrives. This is logged and reported in
 * the diagnostics message so that a degraded interval is visible in the bag.
 */
class MactControllerBase : public controller_interface::ControllerInterface
{
public:
  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // --------------------------------------------------------------- hooks --
  /// Extra parameter declarations of the concrete controller.
  virtual CallbackReturn onInitDerived() {return CallbackReturn::SUCCESS;}

  /// Extra configuration (publishers, subscriptions, model instantiation).
  virtual CallbackReturn onConfigureDerived() {return CallbackReturn::SUCCESS;}

  /// Extra activation work; the base class has already reset its own state.
  virtual CallbackReturn onActivateDerived() {return CallbackReturn::SUCCESS;}

  virtual void onDeactivateDerived() {}

  /**
   * @brief Provide the model quantities for this cycle.
   *
   * @param motion The measured and desired motion.
   * @param terms  Output; preallocated, fill in place and set the has_* flags.
   * @return false when no usable Y_r is available, which puts the controller
   *         in the PD-only fallback for this cycle.
   *
   * Called from the real-time loop: implementations must not allocate, lock or
   * block.
   */
  virtual bool updateModel(const MotionSample & motion, ModelTerms & terms) = 0;

  /**
   * @brief Called at the very end of the cycle, after the command was written.
   *
   * Used by the server-based variant to publish the state and the freshly
   * integrated estimate, so that publishing never delays the command.
   */
  virtual void onCycleEnd(
    const MotionSample & /*motion*/, const Eigen::VectorXd & /*estimate*/) {}

  /**
   * @brief Initial value of pi_hat.
   *
   * Called once from on_configure(), never from on_activate(): the controller
   * manager activates controllers from its real-time thread, so anything that
   * may block (a service call to the server) has to happen at configuration
   * time.
   *
   * @param estimate Output, already sized to numParameters().
   * @return false to abort configuration.
   */
  virtual bool fetchInitialEstimate(Eigen::VectorXd & estimate) = 0;

  // ------------------------------------------------------------ accessors --
  int numParameters() const {return num_parameters_;}
  const std::vector<std::string> & jointNames() const {return joint_names_;}
  bool needsRegressor() const {return adaptation_.usesPredictionError();}
  /// Only the 'estimate' gravity source needs reg_G; the others do not.
  bool needsGravityRegressor() const {return gravity_source_ == GravitySource::kEstimate;}

private:
  void readState();
  /// Fetch the robot description, used by the urdf_kdl gravity source.
  bool fetchRobotDescription(std::string & description);
  /// Fill gravity_torque_ from the configured source; false when unavailable.
  bool updateGravity(const MotionSample & motion);
  void writeCommand(const Vector7d & torque);
  void publishDiagnostics(const rclcpp::Time & time, const MotionSample & motion);
  void trajectoryCallback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr message);

  // ------------------------------------------------------------ interface --
  std::vector<std::string> joint_names_;
  std::string arm_id_;

  // --------------------------------------------------------------- gains --
  Vector7d k_p_{Vector7d::Zero()};
  Vector7d k_v_{Vector7d::Zero()};
  Vector7d max_torque_{Vector7d::Zero()};
  double max_tracking_error_{0.0};

  // ------------------------------------------------------------ estimation --
  JointStateEstimator estimator_;
  AdaptationLaw adaptation_;
  Eigen::VectorXd initial_estimate_;
  int num_parameters_{100};
  bool adaptation_enabled_{true};
  /// Latched by the first reference; the estimate is held until then.
  bool adaptation_started_{false};

  // ----------------------------------------------------------- references --
  realtime_tools::RealtimeBuffer<TrajectorySample> trajectory_buffer_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr
    trajectory_subscription_;
  double trajectory_timeout_s_{0.0};
  /// The generator's motion clock, as last reported by the reference.
  double trajectory_time_{0.0};

  // ------------------------------------------------------------- measured --
  Vector7d q_{Vector7d::Zero()};
  Vector7d dq_raw_{Vector7d::Zero()};
  Vector7d tau_measured_{Vector7d::Zero()};
  Vector7d q_hold_{Vector7d::Zero()};

  // -------------------------------------------------------------- torques --
  Vector7d tau_model_{Vector7d::Zero()};
  Vector7d tau_model_previous_{Vector7d::Zero()};
  Vector7d tau_command_{Vector7d::Zero()};

  // ---------------------------------------------------------------- model --
  ModelTerms terms_;

  // ------------------------------------------------------------- gravity --
  GravitySource gravity_source_{GravitySource::kNone};
  Vector7d gravity_torque_{Vector7d::Zero()};
  UrdfGravityModel urdf_gravity_;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  /// Whether the model term was actually used in the last cycle.
  bool model_valid_{false};

  // ---------------------------------------------------------- diagnostics --
  std::shared_ptr<realtime_tools::RealtimePublisher<mact_msgs::msg::MactState>>
    state_publisher_;
  int publish_every_n_cycles_{10};
  int cycles_in_window_{0};
  double duration_min_us_{0.0};
  double duration_max_us_{0.0};
  double duration_sum_us_{0.0};
  uint32_t degraded_cycles_{0};
};

}  // namespace mact_controllers
