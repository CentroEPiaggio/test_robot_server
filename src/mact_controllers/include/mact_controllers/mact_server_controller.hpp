// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <franka_server/msg/float32_array.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include "mact_controllers/mact_controller_base.hpp"

namespace mact_controllers {

/**
 * @brief MACT controller that carries no model and talks to the Robot Server.
 *
 * The controller publishes its state (q, dq, ddq) and the reference motion
 * (dqr = dq_d, ddqr = ddq_d) together with the current estimate, and consumes
 * the regressors the server publishes back. It never waits: every quantity is
 * read from a lock-free buffer holding the most recent sample, so the loop
 * closes through the middleware with a bounded, one-round-trip staleness
 * instead of a blocking request-response.
 *
 * The topic names default to the ones declared in the `ros_server_generator`
 * section of franka_conf.yaml.
 *
 * ### Parameter feedback
 *
 * par_REG is published every cycle so that the server's regressors and the
 * controller's estimate never drift apart. par_DYN is published as well, so
 * that the server's M, C, G stay meaningful too; its value is not recomputed
 * here but taken from the server's own reg2dyn topic, which is the conversion
 * the server itself publishes. par_DYN is therefore one round trip behind
 * par_REG, which is harmless because nothing in the control law uses it.
 *
 * @note reg2dyn divides the first moments by the mass, so it returns NaN for
 *       every zero-mass block (with the shipped parameters: `base` and `EE`).
 *       The `EE` block falls inside the 80 values that make up par_DYN, so the
 *       NaNs are filtered out here before publishing; see sanitise().
 */
class MactServerController : public MactControllerBase
{
protected:
  CallbackReturn onInitDerived() override;
  CallbackReturn onConfigureDerived() override;
  CallbackReturn onActivateDerived() override;

  bool updateModel(const MotionSample & motion, ModelTerms & terms) override;
  void onCycleEnd(const MotionSample & motion, const Eigen::VectorXd & estimate) override;
  bool fetchInitialEstimate(Eigen::VectorXd & estimate) override;

private:
  using FloatArray = franka_server::msg::Float32Array;

  /// A matrix as most recently received, with the time it arrived.
  struct TimedArray
  {
    std::vector<float> data;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  using ArrayBuffer = realtime_tools::RealtimeBuffer<TimedArray>;
  using ArrayPublisher = realtime_tools::RealtimePublisher<FloatArray>;

  /// Create a keep-last-one subscription filling @p buffer.
  rclcpp::Subscription<FloatArray>::SharedPtr makeSubscription(
    const std::string & topic, ArrayBuffer & buffer);

  /// Create a keep-last-one publisher with @p size values preallocated.
  std::shared_ptr<ArrayPublisher> makePublisher(const std::string & topic, std::size_t size);

  /// Publish an Eigen vector through a real-time publisher, dropping the
  /// sample if the publisher is busy.
  static void publish(
    const std::shared_ptr<ArrayPublisher> & publisher, const Eigen::VectorXd & values);
  static void publish(
    const std::shared_ptr<ArrayPublisher> & publisher, const Vector7d & values);

  /**
   * @brief Copy a row-major flattened 7 x n matrix into @p destination.
   * @return false if the message does not have the expected size.
   */
  bool toMatrix(const TimedArray & source, RegressorMatrix & destination) const;

  /// Age of a sample in milliseconds; a large value when it was never filled.
  double ageMs(const TimedArray & source, const rclcpp::Time & now) const;

  /// Replace every non-finite entry by zero. Returns the number replaced.
  static int sanitise(Eigen::VectorXd & values);

  // ------------------------------------------------------------- incoming --
  ArrayBuffer regressor_r_buffer_;
  ArrayBuffer regressor_buffer_;
  ArrayBuffer regressor_g_buffer_;
  ArrayBuffer reg2dyn_buffer_;

  rclcpp::Subscription<FloatArray>::SharedPtr regressor_r_subscription_;
  rclcpp::Subscription<FloatArray>::SharedPtr regressor_subscription_;
  rclcpp::Subscription<FloatArray>::SharedPtr regressor_g_subscription_;
  rclcpp::Subscription<FloatArray>::SharedPtr reg2dyn_subscription_;

  // ------------------------------------------------------------- outgoing --
  std::shared_ptr<ArrayPublisher> q_publisher_;
  std::shared_ptr<ArrayPublisher> dq_publisher_;
  std::shared_ptr<ArrayPublisher> ddq_publisher_;
  std::shared_ptr<ArrayPublisher> dqr_publisher_;
  std::shared_ptr<ArrayPublisher> ddqr_publisher_;
  std::shared_ptr<ArrayPublisher> par_reg_publisher_;
  std::shared_ptr<ArrayPublisher> par_dyn_publisher_;

  // ------------------------------------------------------------- settings --
  double server_timeout_s_{0.0};
  int num_dynamic_parameters_{80};
  bool warned_about_nan_{false};

  // Scratch space, preallocated in on_configure().
  Eigen::VectorXd par_dyn_;
};

}  // namespace mact_controllers
