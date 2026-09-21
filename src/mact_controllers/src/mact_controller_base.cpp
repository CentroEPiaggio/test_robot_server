// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#include "mact_controllers/mact_controller_base.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <array>

namespace mact_controllers {

namespace {

/// Copy an Eigen 7-vector into a fixed-size message array.
template<typename Array>
void toMessage(const Vector7d & source, Array & destination)
{
  for (int joint = 0; joint < kNumJoints; ++joint) {
    destination[joint] = source(joint);
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// Interface configuration
// ---------------------------------------------------------------------------

controller_interface::InterfaceConfiguration
MactControllerBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    configuration.names.push_back(joint + "/effort");
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
MactControllerBase::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // The order fixed here is the one readState() relies on: three interfaces
  // per joint, position first.
  for (const auto & joint : joint_names_) {
    configuration.names.push_back(joint + "/position");
    configuration.names.push_back(joint + "/velocity");
    configuration.names.push_back(joint + "/effort");
  }
  // The robot's own model is exposed as two extra state interfaces by
  // franka_hardware. They are claimed only when the gravity actually comes
  // from there, so that the controller still runs in Gazebo, where they do not
  // exist. readState() indexes the joint interfaces from the front, so
  // appending these at the end changes nothing else.
  if (franka_robot_model_) {
    for (const auto & name : franka_robot_model_->get_state_interface_names()) {
      configuration.names.push_back(name);
    }
  }
  return configuration;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

CallbackReturn MactControllerBase::on_init()
{
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::vector<std::string>>("joints", {});

    auto_declare<std::string>("trajectory_topic", "/mact/trajectory");
    auto_declare<std::string>("state_topic", "/mact/state");

    auto_declare<std::vector<double>>("gains.k_p", {});
    auto_declare<std::vector<double>>("gains.k_v", {});

    auto_declare<int>("num_parameters", 100);
    auto_declare<bool>("adaptation.enabled", true);
    auto_declare<double>("adaptation.gamma", 0.0);
    auto_declare<std::vector<double>>("adaptation.R_t", {});
    auto_declare<std::vector<double>>("adaptation.R_p_link", {});
    auto_declare<std::vector<double>>("adaptation.R_p_param", {});
    auto_declare<double>("adaptation.initial_scale", 1.0);

    auto_declare<double>("filters.dq_cutoff_hz", 100.0);
    auto_declare<double>("filters.ddq_cutoff_hz", 30.0);

    auto_declare<std::vector<double>>("safety.max_torque", {});
    auto_declare<double>("safety.max_tracking_error", 0.5);
    auto_declare<double>("safety.trajectory_timeout_ms", 20.0);

    // Where the gravity torque removed from the command comes from:
    // none | estimate | urdf_kdl | franka_model.
    auto_declare<std::string>("gravity.source", "estimate");
    auto_declare<std::vector<double>>("gravity.vector", {0.0, 0.0, -9.8});
    auto_declare<std::string>("gravity.description_node", "robot_state_publisher");
    auto_declare<std::string>("gravity.chain_root", "");
    auto_declare<std::string>("gravity.chain_tip", "");
    auto_declare<double>("gravity.description_timeout_s", 10.0);

    auto_declare<int>("diagnostics.publish_every_n_cycles", 10);
  } catch (const std::exception & exception) {
    fprintf(stderr, "Exception thrown during init stage: %s\n", exception.what());
    return CallbackReturn::ERROR;
  }

  return onInitDerived();
}

CallbackReturn MactControllerBase::on_configure(const rclcpp_lifecycle::State & /*previous*/)
{
  const auto logger = get_node()->get_logger();

  // ------------------------------------------------------------- joints --
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    // Franka convention: <arm_id>_joint1 .. <arm_id>_joint7.
    for (int joint = 1; joint <= kNumJoints; ++joint) {
      joint_names_.push_back(arm_id_ + "_joint" + std::to_string(joint));
    }
  }
  if (static_cast<int>(joint_names_.size()) != kNumJoints) {
    RCLCPP_FATAL(
      logger, "'joints' must list %d names but lists %zu", kNumJoints, joint_names_.size());
    return CallbackReturn::FAILURE;
  }

  // -------------------------------------------------------------- gains --
  const auto read_vector7 = [&](const std::string & name, Vector7d & target) {
      const auto values = get_node()->get_parameter(name).as_double_array();
      if (static_cast<int>(values.size()) != kNumJoints) {
        RCLCPP_FATAL(
          logger, "'%s' must have %d entries but has %zu", name.c_str(), kNumJoints,
          values.size());
        return false;
      }
      for (int joint = 0; joint < kNumJoints; ++joint) {
        target(joint) = values[joint];
      }
      return true;
    };

  if (!read_vector7("gains.k_p", k_p_) ||
    !read_vector7("gains.k_v", k_v_) ||
    !read_vector7("safety.max_torque", max_torque_))
  {
    return CallbackReturn::FAILURE;
  }
  if ((max_torque_.array() <= 0.0).any()) {
    RCLCPP_FATAL(logger, "'safety.max_torque' entries must be strictly positive");
    return CallbackReturn::FAILURE;
  }

  max_tracking_error_ = get_node()->get_parameter("safety.max_tracking_error").as_double();
  trajectory_timeout_s_ =
    get_node()->get_parameter("safety.trajectory_timeout_ms").as_double() * 1e-3;

  // ---------------------------------------------------------- estimation --
  estimator_.configure(
    get_node()->get_parameter("filters.dq_cutoff_hz").as_double(),
    get_node()->get_parameter("filters.ddq_cutoff_hz").as_double());

  num_parameters_ = static_cast<int>(get_node()->get_parameter("num_parameters").as_int());
  adaptation_enabled_ = get_node()->get_parameter("adaptation.enabled").as_bool();

  std::string adaptation_error;
  if (!adaptation_.configure(
      num_parameters_,
      get_node()->get_parameter("adaptation.R_p_link").as_double_array(),
      get_node()->get_parameter("adaptation.R_p_param").as_double_array(),
      get_node()->get_parameter("adaptation.gamma").as_double(),
      get_node()->get_parameter("adaptation.R_t").as_double_array(),
      adaptation_error))
  {
    RCLCPP_FATAL(logger, "Invalid adaptation configuration: %s", adaptation_error.c_str());
    return CallbackReturn::FAILURE;
  }

  terms_.resize(num_parameters_);
  initial_estimate_.setZero(num_parameters_);

  // ------------------------------------------------------------- gravity --
  const auto gravity_name = get_node()->get_parameter("gravity.source").as_string();
  if (!gravitySourceFromString(gravity_name, gravity_source_)) {
    RCLCPP_FATAL(
      logger,
      "'gravity.source' is '%s'; expected one of none, estimate, urdf_kdl, franka_model",
      gravity_name.c_str());
    return CallbackReturn::FAILURE;
  }

  if (gravity_source_ == GravitySource::kUrdfKdl) {
    const auto vector = get_node()->get_parameter("gravity.vector").as_double_array();
    if (vector.size() != 3) {
      RCLCPP_FATAL(logger, "'gravity.vector' must have 3 entries but has %zu", vector.size());
      return CallbackReturn::FAILURE;
    }
    std::string description;
    if (!fetchRobotDescription(description)) {
      return CallbackReturn::FAILURE;
    }
    std::string error;
    if (!urdf_gravity_.configure(
        description, {vector[0], vector[1], vector[2]},
        get_node()->get_parameter("gravity.chain_root").as_string(),
        get_node()->get_parameter("gravity.chain_tip").as_string(), error))
    {
      RCLCPP_FATAL(logger, "Could not build the gravity model from the URDF: %s", error.c_str());
      return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(
      logger, "Gravity from the URDF chain '%s' -> '%s' with g = [%.3f, %.3f, %.3f]",
      urdf_gravity_.rootLink().c_str(), urdf_gravity_.tipLink().c_str(),
      vector[0], vector[1], vector[2]);
  }

  if (gravity_source_ == GravitySource::kFrankaModel) {
    // Only franka_hardware exposes these; in Gazebo the activation will fail
    // on the missing interfaces, which is the intended, loud failure.
    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      arm_id_ + "/robot_model", arm_id_ + "/robot_state");
    RCLCPP_INFO(logger, "Gravity from the robot's own model ('%s/robot_model')", arm_id_.c_str());
  }

  // ----------------------------------------------------------- reference --
  const auto trajectory_topic = get_node()->get_parameter("trajectory_topic").as_string();
  // Keep-last-one, so that a late sample is superseded rather than queued: the
  // loop always consumes the most recent reference and never falls behind.
  trajectory_subscription_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
    trajectory_topic, rclcpp::QoS(1),
    [this](const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr message) {
      trajectoryCallback(message);
    });

  // --------------------------------------------------------- diagnostics --
  publish_every_n_cycles_ =
    static_cast<int>(get_node()->get_parameter("diagnostics.publish_every_n_cycles").as_int());
  publish_every_n_cycles_ = std::max(1, publish_every_n_cycles_);

  const auto state_topic = get_node()->get_parameter("state_topic").as_string();
  state_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<mact_msgs::msg::MactState>>(
    get_node()->create_publisher<mact_msgs::msg::MactState>(state_topic, rclcpp::QoS(10)));
  // Size the variable-length field once, here, so that update() never
  // allocates while holding the real-time publisher.
  state_publisher_->msg_.pi_hat.assign(num_parameters_, 0.0);

  const auto derived = onConfigureDerived();
  if (derived != CallbackReturn::SUCCESS) {
    return derived;
  }

  // The initial estimate is fetched last, because the derived class may need
  // its own configuration (e.g. a service client) to obtain it.
  if (!fetchInitialEstimate(initial_estimate_)) {
    RCLCPP_FATAL(logger, "Could not obtain the initial parameter estimate");
    return CallbackReturn::FAILURE;
  }
  const double scale = get_node()->get_parameter("adaptation.initial_scale").as_double();
  initial_estimate_ *= scale;
  RCLCPP_INFO(
    logger,
    "MACT configured: %d parameters, adaptation %s, prediction error term %s, "
    "gravity source '%s', initial estimate scaled by %.3f",
    num_parameters_, adaptation_enabled_ ? "on" : "off",
    adaptation_.usesPredictionError() ? "on" : "off",
    toString(gravity_source_), scale);

  return CallbackReturn::SUCCESS;
}

CallbackReturn MactControllerBase::on_activate(const rclcpp_lifecycle::State & /*previous*/)
{
  if (franka_robot_model_) {
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  }
  gravity_torque_.setZero();

  readState();

  estimator_.reset();
  adaptation_.setEstimate(initial_estimate_);

  // Until a reference arrives, hold the pose the robot was activated in.
  q_hold_ = q_;
  trajectory_buffer_.writeFromNonRT(TrajectorySample{});

  tau_model_.setZero();
  tau_model_previous_.setZero();
  tau_command_.setZero();
  terms_.invalidate();
  model_valid_ = false;

  degraded_cycles_ = 0;
  cycles_in_window_ = 0;
  duration_sum_us_ = 0.0;
  duration_min_us_ = std::numeric_limits<double>::max();
  duration_max_us_ = 0.0;

  return onActivateDerived();
}

CallbackReturn MactControllerBase::on_deactivate(const rclcpp_lifecycle::State & /*previous*/)
{
  // Leave the joints without torque rather than with the last command.
  writeCommand(Vector7d::Zero());
  if (franka_robot_model_) {
    franka_robot_model_->release_interfaces();
  }
  onDeactivateDerived();
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------

controller_interface::return_type MactControllerBase::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const auto cycle_start = std::chrono::steady_clock::now();
  const double dt = period.seconds();

  // ------------------------------------------------------ measured state --
  readState();
  estimator_.update(dq_raw_, dt);

  // --------------------------------------------------- desired trajectory --
  MotionSample motion;
  motion.q = q_;
  motion.dq = estimator_.dq();
  motion.ddq = estimator_.ddq();

  const TrajectorySample & reference = *trajectory_buffer_.readFromRT();
  const bool reference_fresh =
    reference.valid && trajectory_timeout_s_ > 0.0 &&
    (time - reference.stamp).seconds() <= trajectory_timeout_s_;

  if (reference_fresh) {
    motion.q_d = reference.q_d;
    motion.dq_d = reference.dq_d;
    motion.ddq_d = reference.ddq_d;
    q_hold_ = reference.q_d;
  } else {
    // No reference, or the generator stalled: hold the last commanded pose at
    // rest instead of tracking a stale velocity.
    motion.q_d = q_hold_;
    motion.dq_d.setZero();
    motion.ddq_d.setZero();
    if (reference.valid) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Desired trajectory is stale; holding position");
    }
  }

  const Vector7d error = motion.q_d - motion.q;
  const Vector7d error_rate = motion.dq_d - motion.dq;

  if (max_tracking_error_ > 0.0 && error.cwiseAbs().maxCoeff() > max_tracking_error_) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Tracking error %.3f rad exceeds the limit of %.3f rad; deactivating",
      error.cwiseAbs().maxCoeff(), max_tracking_error_);
    writeCommand(Vector7d::Zero());
    return controller_interface::return_type::ERROR;
  }

  // ---------------------------------------------------------------- model --
  terms_.invalidate();
  model_valid_ = updateModel(motion, terms_) && terms_.has_regressor_r;

  // The model term and the gravity subtraction go together. Commanding
  // Y_r*pi_hat without being able to remove the gravity part of it would apply
  // gravity twice, since both libfranka and franka_ign_ros2_control add their
  // own gravity compensation on top of the commanded torque. Falling back to
  // the PD term is safe in that case: the robot stays gravity-compensated by
  // the hardware and merely loses the feed-forward.
  if (!updateGravity(motion)) {
    model_valid_ = false;
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "No gravity torque available from source '%s'; commanding the PD term "
      "alone rather than gravity twice", toString(gravity_source_));
  }

  // ----------------------------------------------------------- control law --
  // PD part, always present.
  tau_model_ = k_v_.cwiseProduct(error_rate) + k_p_.cwiseProduct(error);

  if (model_valid_) {
    // Model term assembled from the Slotine-Li regressor and the local copy of
    // the estimate, i.e. eq. (3) of the extended abstract.
    tau_model_.noalias() += terms_.regressor_r * adaptation_.estimate();
  } else {
    ++degraded_cycles_;
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "No usable regressor; commanding the PD term alone");
  }

  // ------------------------------------------------------------ adaptation --
  // Uses the torque applied in the *previous* cycle, which is the one the
  // prediction error of eq. (2) refers to.
  if (adaptation_enabled_ && model_valid_) {
    adaptation_.update(
      terms_.regressor_r, error_rate, terms_.regressor, tau_model_previous_,
      terms_.has_regressor, dt);
  }

  // -------------------------------------------------------------- command --
  tau_command_ = tau_model_;
  if (model_valid_) {
    // Only when the model term is actually commanded: with the PD term alone
    // there is no gravity in the command to remove, and subtracting it would
    // make the arm drop.
    tau_command_ -= gravity_torque_;
  }
  tau_command_ = tau_command_.cwiseMax(-max_torque_).cwiseMin(max_torque_);

  writeCommand(tau_command_);
  tau_model_previous_ = tau_model_;

  // Publishing happens after the command is written, so that it can never
  // delay the actuation.
  onCycleEnd(motion, adaptation_.estimate());

  // ------------------------------------------------------------- timing ---
  const double duration_us =
    std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - cycle_start)
    .count();
  duration_sum_us_ += duration_us;
  duration_min_us_ = std::min(duration_min_us_, duration_us);
  duration_max_us_ = std::max(duration_max_us_, duration_us);
  ++cycles_in_window_;

  if (cycles_in_window_ >= publish_every_n_cycles_) {
    publishDiagnostics(time, motion);
  }

  return controller_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

bool MactControllerBase::updateGravity(const MotionSample & motion)
{
  switch (gravity_source_) {
    case GravitySource::kNone:
      gravity_torque_.setZero();
      return true;

    case GravitySource::kEstimate:
      // reg_G(q) * pi_hat. The only source that depends on the model being
      // available this cycle.
      if (!terms_.has_regressor_g) {
        return false;
      }
      gravity_torque_.noalias() = terms_.regressor_g * adaptation_.estimate();
      return true;

    case GravitySource::kUrdfKdl:
      gravity_torque_ = urdf_gravity_.compute(motion.q);
      return true;

    case GravitySource::kFrankaModel: {
      if (!franka_robot_model_) {
        return false;
      }
      const std::array<double, kNumJoints> gravity =
        franka_robot_model_->getGravityForceVector();
      for (int joint = 0; joint < kNumJoints; ++joint) {
        gravity_torque_(joint) = gravity[joint];
      }
      return true;
    }
  }
  return false;
}

bool MactControllerBase::fetchRobotDescription(std::string & description)
{
  const auto logger = get_node()->get_logger();
  const auto source = get_node()->get_parameter("gravity.description_node").as_string();
  const auto timeout = std::chrono::duration<double>(
    get_node()->get_parameter("gravity.description_timeout_s").as_double());
  const auto timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(timeout);

  // As with the initial estimate: a throw-away node with its own executor, so
  // that waiting here cannot deadlock the controller manager's executor.
  auto client_node = std::make_shared<rclcpp::Node>(
    std::string(get_node()->get_name()) + "_description_client");
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(client_node, source);

  if (!client->wait_for_service(timeout_ns)) {
    RCLCPP_FATAL(
      logger, "Node '%s' did not appear within %.1f s; cannot read 'robot_description'",
      source.c_str(), timeout.count());
    return false;
  }

  auto future = client->get_parameters({"robot_description"});
  if (rclcpp::spin_until_future_complete(client_node, future, timeout_ns) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_FATAL(logger, "Could not read 'robot_description' from '%s'", source.c_str());
    return false;
  }

  const auto values = future.get();
  if (values.empty() || values[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
    values[0].as_string().empty())
  {
    RCLCPP_FATAL(logger, "'%s' has no usable 'robot_description'", source.c_str());
    return false;
  }
  description = values[0].as_string();
  return true;
}

void MactControllerBase::readState()
{
  for (int joint = 0; joint < kNumJoints; ++joint) {
    q_(joint) = state_interfaces_[3 * joint + 0].get_value();
    dq_raw_(joint) = state_interfaces_[3 * joint + 1].get_value();
    tau_measured_(joint) = state_interfaces_[3 * joint + 2].get_value();
  }
}

void MactControllerBase::writeCommand(const Vector7d & torque)
{
  for (int joint = 0; joint < kNumJoints; ++joint) {
    command_interfaces_[joint].set_value(torque(joint));
  }
}

void MactControllerBase::trajectoryCallback(
  const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr message)
{
  if (message->positions.size() != static_cast<std::size_t>(kNumJoints) ||
    message->velocities.size() != static_cast<std::size_t>(kNumJoints) ||
    message->accelerations.size() != static_cast<std::size_t>(kNumJoints))
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Ignoring a trajectory point that does not carry %d positions, velocities "
      "and accelerations", kNumJoints);
    return;
  }

  TrajectorySample sample;
  for (int joint = 0; joint < kNumJoints; ++joint) {
    sample.q_d(joint) = message->positions[joint];
    sample.dq_d(joint) = message->velocities[joint];
    sample.ddq_d(joint) = message->accelerations[joint];
  }
  // JointTrajectoryPoint has no header, so the arrival time is what the
  // staleness check has to work with.
  sample.stamp = get_node()->now();
  sample.valid = true;

  trajectory_buffer_.writeFromNonRT(sample);
}

void MactControllerBase::publishDiagnostics(const rclcpp::Time & time, const MotionSample & motion)
{
  if (state_publisher_ && state_publisher_->trylock()) {
    auto & message = state_publisher_->msg_;
    message.header.stamp = time;

    toMessage(motion.q, message.q);
    toMessage(motion.dq, message.dq);
    toMessage(motion.ddq, message.ddq);
    toMessage(motion.q_d, message.q_d);
    toMessage(motion.dq_d, message.dq_d);
    toMessage(motion.ddq_d, message.ddq_d);
    toMessage(motion.q_d - motion.q, message.e);
    toMessage(motion.dq_d - motion.dq, message.de);
    toMessage(tau_model_, message.tau_model);
    toMessage(tau_command_, message.tau_cmd);
    toMessage(tau_measured_, message.tau_meas);
    toMessage(gravity_torque_, message.tau_gravity);

    const auto & estimate = adaptation_.estimate();
    for (int parameter = 0; parameter < num_parameters_; ++parameter) {
      message.pi_hat[parameter] = estimate(parameter);
    }

    message.update_duration_min_us = duration_min_us_;
    message.update_duration_max_us = duration_max_us_;
    message.update_duration_mean_us =
      cycles_in_window_ > 0 ? duration_sum_us_ / cycles_in_window_ : 0.0;
    message.cycles_in_window = static_cast<uint32_t>(cycles_in_window_);

    message.model_valid = model_valid_;
    message.model_age_ms = terms_.age_ms;
    message.degraded_cycles = degraded_cycles_;
    message.trajectory_valid = trajectory_buffer_.readFromRT()->valid;

    state_publisher_->unlockAndPublish();
  }

  // The window is restarted whether or not the message could be published, so
  // that a missed publication does not distort the next one.
  cycles_in_window_ = 0;
  duration_sum_us_ = 0.0;
  duration_min_us_ = std::numeric_limits<double>::max();
  duration_max_us_ = 0.0;
}

}  // namespace mact_controllers
