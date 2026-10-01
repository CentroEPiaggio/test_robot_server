// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#include "mact_controllers/mact_controller_base.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <array>
#include <thread>
#include <tuple>

namespace mact_controllers {

namespace {

using mact_msgs::msg::MactState;

// The per-stage arrays of the message are indexed by Stage.
static_assert(
  std::tuple_size<decltype(MactState::stage_duration_mean_us)>::value == kNumStages &&
  std::tuple_size<decltype(MactState::stage_duration_max_us)>::value == kNumStages,
  "MactState's per-stage arrays must have one entry per Stage");
static_assert(
  MactState::STAGE_STATE == static_cast<std::size_t>(Stage::kState) &&
  MactState::STAGE_MODEL == static_cast<std::size_t>(Stage::kModel) &&
  MactState::STAGE_REGRESSOR_R == static_cast<std::size_t>(Stage::kRegressorR) &&
  MactState::STAGE_REGRESSOR == static_cast<std::size_t>(Stage::kRegressor) &&
  MactState::STAGE_REGRESSOR_G == static_cast<std::size_t>(Stage::kRegressorG) &&
  MactState::STAGE_GRAVITY == static_cast<std::size_t>(Stage::kGravity) &&
  MactState::STAGE_CONTROL == static_cast<std::size_t>(Stage::kControl) &&
  MactState::STAGE_ADAPTATION == static_cast<std::size_t>(Stage::kAdaptation) &&
  MactState::STAGE_CYCLE_END == static_cast<std::size_t>(Stage::kCycleEnd) &&
  MactState::STAGE_SNAPSHOT == static_cast<std::size_t>(Stage::kSnapshot),
  "MactState's STAGE_* constants and Stage must list the stages in the same order");

/// How many times the timer tries to take the snapshot before leaving it to
/// the next tick. The loop holds the mutex for a copy of a few microseconds.
constexpr int kSnapshotAttempts = 10;

DurationStats & stage(TimingWindow & timing, Stage which)
{
  return timing.stages[static_cast<std::size_t>(which)];
}

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

CallbackReturn MactControllerBase::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::vector<std::string>>("joints", {});

    auto_declare<std::string>("trajectory_topic", "/mact/trajectory");
    auto_declare<std::string>("state_topic", "/mact/state");

    auto_declare<std::vector<double>>("gains.k_p", {});
    auto_declare<std::vector<double>>("gains.k_v", {});

    auto_declare<int>("num_parameters", 100);
    auto_declare<bool>("adaptation.enabled", true);
    // Whether the update law integrates is decided from the outside, by
    // whoever owns the motion; this is only the value it starts from.
    auto_declare<std::string>("adaptation.service", "~/set_adaptation");
    // Which torque the prediction error compares against: measured | commanded.
    auto_declare<std::string>("adaptation.torque_source", "measured");
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

    // Rate of the non-real-time timer that publishes mact_msgs/MactState;
    // <= 0 disables the diagnostics altogether.
    auto_declare<double>("diagnostics.publish_rate_hz", 100.0);
  } catch (const std::exception & exception) {
    fprintf(stderr, "Exception thrown during init stage: %s\n", exception.what());
    return CallbackReturn::ERROR;
  }

  return onInitDerived();
}

CallbackReturn MactControllerBase::on_configure(const rclcpp_lifecycle::State & /*previous*/) {
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
  adaptation_enabled_default_ = get_node()->get_parameter("adaptation.enabled").as_bool();
  adaptation_enabled_.store(adaptation_enabled_default_, std::memory_order_relaxed);

  const auto torque_name = get_node()->get_parameter("adaptation.torque_source").as_string();
  if (!adaptationTorqueSourceFromString(torque_name, adaptation_torque_source_)) {
    RCLCPP_FATAL(
      logger, "'adaptation.torque_source' is '%s'; expected 'measured' or 'commanded'",
      torque_name.c_str());
    return CallbackReturn::FAILURE;
  }

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

  // ---------------------------------------------------- adaptation switch --
  // The controller does not decide when estimating is meaningful; it is told.
  const auto adaptation_service = get_node()->get_parameter("adaptation.service").as_string();
  // on_configure() can run again after a cleanup, and the name is still taken
  // until the old service is destroyed: release it before asking for it again.
  adaptation_service_.reset();
  adaptation_service_ = get_node()->create_service<std_srvs::srv::SetBool>(
    adaptation_service,
    [this](
      const std_srvs::srv::SetBool::Request::SharedPtr request,
      std_srvs::srv::SetBool::Response::SharedPtr response) {
      setAdaptationCallback(request, response);
    });

  // --------------------------------------------------------- diagnostics --
  // Stop the timer of a previous configuration before touching what it reads.
  if (diagnostics_timer_) {
    diagnostics_timer_->cancel();
  }
  diagnostics_timer_.reset();
  state_publisher_.reset();

  // Size the estimate once, here, so that neither the loop nor the timer
  // allocates when they copy it around.
  snapshot_ = DiagnosticsSnapshot{};
  snapshot_.pi_hat.setZero(num_parameters_);
  published_snapshot_ = snapshot_;
  state_message_ = MactState{};
  state_message_.pi_hat.assign(num_parameters_, 0.0);

  const double publish_rate_hz =
    get_node()->get_parameter("diagnostics.publish_rate_hz").as_double();
  diagnostics_enabled_ = publish_rate_hz > 0.0;
  if (diagnostics_enabled_) {
    const auto state_topic = get_node()->get_parameter("state_topic").as_string();
    // Best effort, keep-last-one: this is monitoring, and a sample that did
    // not make it is superseded by the next one. A subscriber has to be best
    // effort as well, since a reliable one does not match this publisher.
    state_publisher_ =
      get_node()->create_publisher<MactState>(state_topic, rclcpp::QoS(1).best_effort());
    // A wall timer on the controller manager's executor: it runs outside the
    // real-time loop, which only ever hands it a snapshot.
    diagnostics_timer_ = get_node()->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz)),
      [this]() {publishDiagnostics();});
    RCLCPP_INFO(
      logger, "Diagnostics on '%s' at %.1f Hz", state_publisher_->get_topic_name(),
      publish_rate_hz);
  } else {
    RCLCPP_WARN(
      logger, "Diagnostics disabled ('diagnostics.publish_rate_hz' <= 0): nothing is "
      "published on the state topic, and the loop's warnings are not logged either");
  }

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
    "MACT configured: %d parameters, adaptation starts %s (toggle on '%s'), "
    "prediction error term %s against the %s torque, gravity source '%s', "
    "initial estimate scaled by %.3f",
    num_parameters_, adaptation_enabled_default_ ? "on" : "off",
    adaptation_service_->get_service_name(),
    adaptation_.usesPredictionError() ? "on" : "off",
    toString(adaptation_torque_source_), toString(gravity_source_), scale);

  return CallbackReturn::SUCCESS;
}

CallbackReturn MactControllerBase::on_activate(const rclcpp_lifecycle::State & /*previous*/) {
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
  tau_applied_.setZero();
  tau_command_.setZero();
  reference_fresh_ = false;
  terms_.invalidate();
  model_valid_ = false;

  // Every activation starts from the configured value, so that a run is not
  // silently affected by a service call made during the previous one.
  adaptation_enabled_.store(adaptation_enabled_default_, std::memory_order_relaxed);
  trajectory_time_ = 0.0;
  degraded_cycles_ = 0;
  stale_reference_cycles_ = 0;
  gravity_unavailable_cycles_ = 0;
  timing_.reset();
  snapshot_duration_us_ = -1.0;
  tripped_tracking_error_.store(-1.0, std::memory_order_relaxed);
  // Whatever the timer has not taken yet belongs to the previous activation.
  // If it is taking it right now, the leftover timing merges into the first
  // window of this one, which is harmless.
  {
    std::unique_lock<std::mutex> lock(snapshot_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      snapshot_.timing.reset();
      snapshot_.fresh = false;
    }
  }

  return onActivateDerived();
}

CallbackReturn MactControllerBase::on_deactivate(const rclcpp_lifecycle::State & /*previous*/)
{
  // Leave the joints without torque rather than with the last command: a full
  // torque equal to G is a zero command.
  writeCommand(gravity_torque_);
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
  const rclcpp::Time & time, const rclcpp::Duration & period) {
  // Nothing in here logs: writing to the console and to /rosout from the loop
  // can block it. Whatever is worth reporting is counted, and the diagnostics
  // timer logs it from outside the loop.
  const auto cycle_start = SteadyClock::now();
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
  // A non-positive timeout means "never stale", which is the only reading that
  // does not turn a configuration typo into a controller that silently ignores
  // every reference it is given.
  const bool reference_fresh =
    reference.valid &&
    (trajectory_timeout_s_ <= 0.0 ||
    (time - reference.stamp).seconds() <= trajectory_timeout_s_);

  reference_fresh_ = reference_fresh;

  if (reference_fresh) {
    motion.q_d = reference.q_d;
    motion.dq_d = reference.dq_d;
    motion.ddq_d = reference.ddq_d;
    trajectory_time_ = reference.time_from_start;
    q_hold_ = reference.q_d;
  } else {
    // No reference, or the generator stalled: hold the last commanded pose at
    // rest instead of tracking a stale velocity.
    motion.q_d = q_hold_;
    motion.dq_d.setZero();
    motion.ddq_d.setZero();
    if (reference.valid) {
      ++stale_reference_cycles_;
    }
  }

  const Vector7d error = motion.q_d - motion.q;
  const Vector7d error_rate = motion.dq_d - motion.dq;

  if (max_tracking_error_ > 0.0 && error.cwiseAbs().maxCoeff() > max_tracking_error_) {
    tripped_tracking_error_.store(error.cwiseAbs().maxCoeff(), std::memory_order_relaxed);
    writeCommand(gravity_torque_);  // a zero command
    return controller_interface::return_type::ERROR;
  }
  const auto state_done = SteadyClock::now();

  // ---------------------------------------------------------------- model --
  terms_.invalidate();
  model_valid_ = updateModel(motion, terms_) && terms_.has_regressor_r;
  const auto model_done = SteadyClock::now();

  // G is the gravity the hardware adds back on top of the command (libfranka,
  // franka_ign_ros2_control). Without it the model term cannot be commanded,
  // since its own gravity would then be applied twice: fall back to PD.
  if (!updateGravity(motion)) {
    model_valid_ = false;
    ++gravity_unavailable_cycles_;
  }
  const auto gravity_done = SteadyClock::now();

  // ----------------------------------------------------------- control law --
  // The full joint torque, gravity included. The model term carries the
  // estimated gravity (eq. (3)); without it the hardware's own G stands in, so
  // that what is sent below is the PD term alone.
  tau_model_ = k_v_.cwiseProduct(error_rate) + k_p_.cwiseProduct(error);
  if (model_valid_) {
    tau_model_.noalias() += terms_.regressor_r * adaptation_.estimate();
  } else {
    tau_model_ += gravity_torque_;
    ++degraded_cycles_;
  }
  const auto adaptation_start = SteadyClock::now();

  // ------------------------------------------------------------ adaptation --
  // Switched from outside, by the SetBool service. The prediction error needs
  // the torque applied over [k-1, k], the interval ddq_k was differentiated
  // over: the sensor's tau_meas_k, or tau_applied_, the full torque the robot
  // got in the previous cycle (written below, after this update).
  const Vector7d & adaptation_torque =
    adaptation_torque_source_ == AdaptationTorqueSource::kMeasured ?
    tau_measured_ : tau_applied_;
  if (adaptation_enabled_.load(std::memory_order_relaxed) && model_valid_) {
    adaptation_.update(
      terms_.regressor_r, error_rate, terms_.regressor, adaptation_torque,
      terms_.has_regressor, dt);
  }
  const auto adaptation_done = SteadyClock::now();

  // -------------------------------------------------------------- command --
  writeCommand(tau_model_);
  const auto command_done = SteadyClock::now();

  // Publishing happens after the command is written, so that it can never
  // delay the actuation.
  onCycleEnd(motion, adaptation_.estimate());
  const auto cycle_end_done = SteadyClock::now();

  // ---------------------------------------------------------- diagnostics --
  if (diagnostics_enabled_) {
    timing_.total.add(elapsedUs(cycle_start, cycle_end_done));
    stage(timing_, Stage::kState).add(elapsedUs(cycle_start, state_done));
    stage(timing_, Stage::kModel).add(elapsedUs(state_done, model_done));
    stage(timing_, Stage::kRegressorR).add(terms_.regressor_r_us);
    stage(timing_, Stage::kRegressor).add(terms_.regressor_us);
    stage(timing_, Stage::kRegressorG).add(terms_.regressor_g_us);
    stage(timing_, Stage::kGravity).add(elapsedUs(model_done, gravity_done));
    stage(timing_, Stage::kControl).add(
      elapsedUs(gravity_done, adaptation_start) + elapsedUs(adaptation_done, command_done));
    stage(timing_, Stage::kAdaptation).add(elapsedUs(adaptation_start, adaptation_done));
    stage(timing_, Stage::kCycleEnd).add(elapsedUs(command_done, cycle_end_done));
    // The hand-over of the previous cycle; there is none on the first one.
    if (snapshot_duration_us_ >= 0.0) {
      stage(timing_, Stage::kSnapshot).add(snapshot_duration_us_);
    }
    ++timing_.cycles;

    writeSnapshot(time, motion);
    snapshot_duration_us_ = elapsedUs(cycle_end_done, SteadyClock::now());
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
        gravity_torque_.setZero();
        return false;
      }
      gravity_torque_.noalias() = terms_.regressor_g * adaptation_.estimate();
      return true;

    case GravitySource::kUrdfKdl:
      gravity_torque_ = urdf_gravity_.compute(motion.q);
      return true;

    case GravitySource::kFrankaModel: {
      if (!franka_robot_model_) {
        gravity_torque_.setZero();
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
  // `torque` is the full joint torque. The hardware adds G back on top of the
  // command, so G is removed here, and the Franka limits (magnitude and rate)
  // apply to that gravity-free command, tau_d in the FCI documentation.
  for (int joint = 0; joint < kNumJoints; ++joint) {
    const double command = std::clamp(
      torque(joint) - gravity_torque_(joint), -max_torque_(joint), max_torque_(joint));
    tau_command_(joint) += std::clamp(
      command - tau_command_(joint), -max_tau_rate_[joint], max_tau_rate_[joint]);
    command_interfaces_[joint].set_value(tau_command_(joint));
  }
  // What the robot actually gets, for the next cycle's update law.
  tau_applied_ = tau_command_ + gravity_torque_;
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
  // staleness check has to work with. time_from_start, on the other hand, is
  // the generator's own motion clock and is exactly what analysis needs.
  sample.stamp = get_node()->now();
  sample.time_from_start = rclcpp::Duration(message->time_from_start).seconds();
  sample.valid = true;

  trajectory_buffer_.writeFromNonRT(sample);
}

void MactControllerBase::setAdaptationCallback(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  const bool previous = adaptation_enabled_.exchange(request->data, std::memory_order_relaxed);
  response->success = true;
  response->message = request->data ? "adaptation enabled" : "adaptation disabled";
  if (previous != request->data) {
    RCLCPP_INFO(
      get_node()->get_logger(), "Parameter update law %s",
      request->data ? "enabled" : "frozen");
  }
}

void MactControllerBase::writeSnapshot(const rclcpp::Time & time, const MotionSample & motion) {
  // Runs in the loop: try, never wait. If the timer is taking the previous
  // snapshot right now, this cycle's sample is skipped and its timing stays in
  // timing_, to be handed over with the next one.
  std::unique_lock<std::mutex> lock(snapshot_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return;
  }

  snapshot_.stamp = time;
  snapshot_.motion = motion;
  snapshot_.tau_model = tau_model_;
  snapshot_.tau_command = tau_command_;
  snapshot_.tau_measured = tau_measured_;
  snapshot_.tau_gravity = gravity_torque_;
  // Same size as configured, so this copies without reallocating.
  snapshot_.pi_hat = adaptation_.estimate();

  snapshot_.trajectory_time = trajectory_time_;
  snapshot_.model_age_ms = terms_.age_ms;
  snapshot_.model_valid = model_valid_;
  snapshot_.trajectory_valid = reference_fresh_;
  snapshot_.adaptation_enabled = adaptation_enabled_.load(std::memory_order_relaxed);

  snapshot_.degraded_cycles = degraded_cycles_;
  snapshot_.stale_reference_cycles = stale_reference_cycles_;
  snapshot_.gravity_unavailable_cycles = gravity_unavailable_cycles_;

  snapshot_.timing.merge(timing_);
  snapshot_.fresh = true;
  lock.unlock();

  timing_.reset();
}

void MactControllerBase::publishDiagnostics() {
  // A stop is the one thing that must be reported whether or not there is a
  // snapshot to go with it.
  const double tripped = tripped_tracking_error_.exchange(-1.0, std::memory_order_relaxed);
  if (tripped >= 0.0) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Tracking error %.3f rad exceeds the limit of %.3f rad; deactivating",
      tripped, max_tracking_error_);
  }

  // Try-lock as well, never lock(): the loop's unlock then never has a waiter
  // to wake. The copy happens under the mutex; everything else after it.
  bool taken = false;
  for (int attempt = 0; attempt < kSnapshotAttempts && !taken; ++attempt) {
    std::unique_lock<std::mutex> lock(snapshot_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      std::this_thread::yield();
      continue;
    }
    if (!snapshot_.fresh) {
      return;  // nothing new since the last tick, e.g. the controller is inactive
    }
    published_snapshot_ = snapshot_;
    snapshot_.timing.reset();
    snapshot_.fresh = false;
    taken = true;
  }
  if (!taken) {
    return;  // the timing stays in the snapshot and goes out with the next one
  }

  const DiagnosticsSnapshot & snapshot = published_snapshot_;
  logLoopEvents(snapshot);

  auto & message = state_message_;
  message.header.stamp = snapshot.stamp;

  const MotionSample & motion = snapshot.motion;
  toMessage(motion.q, message.q);
  toMessage(motion.dq, message.dq);
  toMessage(motion.ddq, message.ddq);
  toMessage(motion.q_d, message.q_d);
  toMessage(motion.dq_d, message.dq_d);
  toMessage(motion.ddq_d, message.ddq_d);
  toMessage(motion.q_d - motion.q, message.e);
  toMessage(motion.dq_d - motion.dq, message.de);
  toMessage(snapshot.tau_model, message.tau_model);
  toMessage(snapshot.tau_command, message.tau_cmd);
  toMessage(snapshot.tau_measured, message.tau_meas);
  toMessage(snapshot.tau_gravity, message.tau_gravity);

  for (int parameter = 0; parameter < num_parameters_; ++parameter) {
    message.pi_hat[parameter] = snapshot.pi_hat(parameter);
  }

  const TimingWindow & timing = snapshot.timing;
  const double cycles = static_cast<double>(timing.cycles);
  const auto mean = [cycles](const DurationStats & stats) {
      return cycles > 0.0 ? stats.sum / cycles : 0.0;
    };
  message.update_duration_min_us = timing.cycles > 0 ? timing.total.min : 0.0;
  message.update_duration_max_us = timing.total.max;
  message.update_duration_mean_us = mean(timing.total);
  message.cycles_in_window = timing.cycles;
  for (std::size_t index = 0; index < kNumStages; ++index) {
    message.stage_duration_mean_us[index] = mean(timing.stages[index]);
    message.stage_duration_max_us[index] = timing.stages[index].max;
  }

  message.model_valid = snapshot.model_valid;
  message.model_age_ms = snapshot.model_age_ms;
  message.degraded_cycles = snapshot.degraded_cycles;
  message.trajectory_valid = snapshot.trajectory_valid;
  message.trajectory_time = snapshot.trajectory_time;
  message.adaptation_enabled = snapshot.adaptation_enabled;

  state_publisher_->publish(message);
}

void MactControllerBase::logLoopEvents(const DiagnosticsSnapshot & snapshot) {
  const auto logger = get_node()->get_logger();
  auto & clock = *get_node()->get_clock();

  // The loop counts; this logs whenever a count grew since the last tick, with
  // the throttling the loop used to apply itself. The counts restart from
  // zero at every activation.
  const auto grew = [](uint32_t current, uint32_t & logged) {
      if (current < logged) {
        logged = 0;
      }
      const bool result = current > logged;
      logged = current;
      return result;
    };

  if (grew(snapshot.stale_reference_cycles, logged_stale_reference_cycles_)) {
    RCLCPP_WARN_THROTTLE(logger, clock, 1000, "Desired trajectory is stale; holding position");
  }
  if (grew(snapshot.gravity_unavailable_cycles, logged_gravity_unavailable_cycles_)) {
    RCLCPP_WARN_THROTTLE(
      logger, clock, 5000,
      "No gravity torque available from source '%s'; commanding the PD term "
      "alone rather than gravity twice", toString(gravity_source_));
  }
  if (grew(snapshot.degraded_cycles, logged_degraded_cycles_)) {
    RCLCPP_WARN_THROTTLE(logger, clock, 1000, "No usable regressor; commanding the PD term alone");
  }
}

}  // namespace mact_controllers
