// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#include "mact_controllers/mact_server_controller.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include <franka_server/srv/get_value.hpp>

using namespace std::chrono_literals;

namespace mact_controllers {

namespace {
/// Age reported for a sample that was never received.
constexpr double kNeverReceivedMs = std::numeric_limits<double>::max();
}  // namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

CallbackReturn MactServerController::onInitDerived()
{
  // Defaults mirror the `inputs`, `topics` and `services` entries of
  // franka_conf.yaml.
  auto_declare<std::string>("server.topics.q", "/controller/q");
  auto_declare<std::string>("server.topics.dq", "/controller/dq");
  auto_declare<std::string>("server.topics.ddq", "/controller/ddq");
  auto_declare<std::string>("server.topics.dqr", "/controller/dqr");
  auto_declare<std::string>("server.topics.ddqr", "/controller/ddqr");
  auto_declare<std::string>("server.topics.par_REG", "/controller/par_REG");
  auto_declare<std::string>("server.topics.par_DYN", "/controller/par_DYN");

  auto_declare<std::string>("server.topics.Yr", "franka_server/Yr");
  auto_declare<std::string>("server.topics.Y", "franka_server/Y");
  auto_declare<std::string>("server.topics.reg_G", "franka_server/reg_G");
  auto_declare<std::string>("server.topics.reg2dyn", "franka_server/reg2dyn");

  auto_declare<std::string>("server.services.get_par_REG", "franka_server/get_par_REG");

  auto_declare<double>("server.timeout_ms", 5.0);
  auto_declare<double>("server.wait_timeout_s", 10.0);
  auto_declare<int>("server.num_dynamic_parameters", 80);

  return CallbackReturn::SUCCESS;
}

CallbackReturn MactServerController::onConfigureDerived()
{
  const auto logger = get_node()->get_logger();
  const auto topic = [this](const std::string & name) {
      return get_node()->get_parameter("server.topics." + name).as_string();
    };

  server_timeout_s_ = get_node()->get_parameter("server.timeout_ms").as_double() * 1e-3;
  num_dynamic_parameters_ =
    static_cast<int>(get_node()->get_parameter("server.num_dynamic_parameters").as_int());
  if (num_dynamic_parameters_ < 0 || num_dynamic_parameters_ > numParameters()) {
    RCLCPP_FATAL(
      logger, "'server.num_dynamic_parameters' (%d) must be between 0 and num_parameters (%d)",
      num_dynamic_parameters_, numParameters());
    return CallbackReturn::FAILURE;
  }
  par_dyn_.setZero(num_dynamic_parameters_);

  // ------------------------------------------------------------- incoming --
  regressor_r_subscription_ = makeSubscription(topic("Yr"), regressor_r_buffer_);
  reg2dyn_subscription_ = makeSubscription(topic("reg2dyn"), reg2dyn_buffer_);
  if (needsRegressor()) {
    regressor_subscription_ = makeSubscription(topic("Y"), regressor_buffer_);
  }
  if (needsGravityRegressor()) {
    regressor_g_subscription_ = makeSubscription(topic("reg_G"), regressor_g_buffer_);
  }

  // ------------------------------------------------------------- outgoing --
  q_publisher_ = makePublisher(topic("q"), kNumJoints);
  dq_publisher_ = makePublisher(topic("dq"), kNumJoints);
  ddq_publisher_ = makePublisher(topic("ddq"), kNumJoints);
  dqr_publisher_ = makePublisher(topic("dqr"), kNumJoints);
  ddqr_publisher_ = makePublisher(topic("ddqr"), kNumJoints);
  par_reg_publisher_ = makePublisher(topic("par_REG"), numParameters());
  par_dyn_publisher_ = makePublisher(topic("par_DYN"), num_dynamic_parameters_);

  std::string consumed = topic("Yr");
  if (needsRegressor()) {
    consumed += ", " + topic("Y");
  }
  if (needsGravityRegressor()) {
    consumed += ", " + topic("reg_G");
  }
  RCLCPP_INFO(
    logger, "Robot Server interface ready: reading %s; staleness limit %.1f ms",
    consumed.c_str(), server_timeout_s_ * 1e3);

  return CallbackReturn::SUCCESS;
}

CallbackReturn MactServerController::onActivateDerived()
{
  // Drop anything received while the controller was inactive: those samples
  // describe a motion that is no longer current.
  regressor_r_buffer_.writeFromNonRT(TimedArray{});
  regressor_buffer_.writeFromNonRT(TimedArray{});
  regressor_g_buffer_.writeFromNonRT(TimedArray{});
  reg2dyn_buffer_.writeFromNonRT(TimedArray{});
  warned_about_nan_ = false;
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Initial estimate
// ---------------------------------------------------------------------------

bool MactServerController::fetchInitialEstimate(Eigen::VectorXd & estimate)
{
  const auto logger = get_node()->get_logger();
  const auto service = get_node()->get_parameter("server.services.get_par_REG").as_string();
  const auto wait_timeout =
    std::chrono::duration<double>(get_node()->get_parameter("server.wait_timeout_s").as_double());

  // The call is made through a throw-away node with its own executor: the
  // controller's node is spun by the controller manager, and blocking on a
  // future from inside one of its callbacks would deadlock a single-threaded
  // executor.
  auto client_node = std::make_shared<rclcpp::Node>(
    std::string(get_node()->get_name()) + "_par_client");
  auto client = client_node->create_client<franka_server::srv::GetValue>(service);

  if (!client->wait_for_service(std::chrono::duration_cast<std::chrono::nanoseconds>(wait_timeout)))
  {
    RCLCPP_FATAL(
      logger, "Service '%s' is not available after %.1f s: is franka_server running?",
      service.c_str(), wait_timeout.count());
    return false;
  }

  auto request = std::make_shared<franka_server::srv::GetValue::Request>();
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(
      client_node, future,
      std::chrono::duration_cast<std::chrono::nanoseconds>(wait_timeout)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_FATAL(logger, "Call to '%s' did not complete", service.c_str());
    return false;
  }

  // The shared_ptr has to be kept alive: async_send_request() hands back a
  // std::future, so get() moves the response out and returns it by value.
  // Binding a reference straight to future.get()->value would leave a
  // reference to a response that is already destroyed.
  const auto response = future.get();
  const auto & values = response->value;
  if (static_cast<int>(values.size()) != estimate.size()) {
    RCLCPP_FATAL(
      logger, "'%s' returned %zu values but 'num_parameters' is %ld",
      service.c_str(), values.size(), static_cast<long>(estimate.size()));
    return false;
  }
  for (int parameter = 0; parameter < estimate.size(); ++parameter) {
    estimate(parameter) = static_cast<double>(values[parameter]);
  }

  RCLCPP_INFO(
    logger, "Initial estimate obtained from '%s' (%ld parameters)",
    service.c_str(), static_cast<long>(estimate.size()));
  return true;
}

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------

bool MactServerController::updateModel(const MotionSample & /*motion*/, ModelTerms & terms)
{
  const auto now = get_node()->now();

  const TimedArray & regressor_r = *regressor_r_buffer_.readFromRT();
  terms.age_ms = ageMs(regressor_r, now);

  // Nothing yet, or the server fell behind: the base class falls back to the
  // PD term alone and freezes the estimate. The loop itself never waits.
  if (!regressor_r.valid || terms.age_ms > server_timeout_s_ * 1e3) {
    return false;
  }
  if (!toMatrix(regressor_r, terms.regressor_r)) {
    return false;
  }
  terms.has_regressor_r = true;

  if (needsRegressor()) {
    const TimedArray & regressor = *regressor_buffer_.readFromRT();
    if (regressor.valid && ageMs(regressor, now) <= server_timeout_s_ * 1e3 &&
      toMatrix(regressor, terms.regressor))
    {
      terms.has_regressor = true;
    }
  }

  if (needsGravityRegressor()) {
    const TimedArray & regressor_g = *regressor_g_buffer_.readFromRT();
    if (regressor_g.valid && ageMs(regressor_g, now) <= server_timeout_s_ * 1e3 &&
      toMatrix(regressor_g, terms.regressor_g))
    {
      terms.has_regressor_g = true;
    }
  }

  return true;
}

void MactServerController::onCycleEnd(
  const MotionSample & motion, const Eigen::VectorXd & estimate)
{
  // The state the server needs to evaluate the regressors for the next cycle.
  // dqr/ddqr carry the desired motion, which is what makes get_Yr() the
  // Slotine-Li regressor of eq. (2).
  publish(q_publisher_, motion.q);
  publish(dq_publisher_, motion.dq);
  publish(ddq_publisher_, motion.ddq);
  publish(dqr_publisher_, motion.dq_d);
  publish(ddqr_publisher_, motion.ddq_d);

  // Keep the server's copy of the parameters in step with the estimate.
  publish(par_reg_publisher_, estimate);

  // par_DYN is not recomputed here: it is the conversion the server itself
  // publishes on reg2dyn, of which only the trailing symbolic blocks make up
  // par_DYN. It is therefore one round trip behind par_REG.
  const TimedArray & reg2dyn = *reg2dyn_buffer_.readFromRT();
  if (reg2dyn.valid && static_cast<int>(reg2dyn.data.size()) == numParameters() &&
    num_dynamic_parameters_ > 0)
  {
    const int offset = numParameters() - num_dynamic_parameters_;
    for (int parameter = 0; parameter < num_dynamic_parameters_; ++parameter) {
      par_dyn_(parameter) = static_cast<double>(reg2dyn.data[offset + parameter]);
    }
    // reg2dyn divides by the mass, so a zero-mass block (the EE block with the
    // shipped parameters) comes back as NaN. Forwarding that would poison the
    // server's M, C and G.
    const int replaced = sanitise(par_dyn_);
    if (replaced > 0 && !warned_about_nan_) {
      warned_about_nan_ = true;
      RCLCPP_WARN(
        get_node()->get_logger(),
        "reg2dyn returned %d non-finite values (zero-mass block); they are published as zero. "
        "The generated reg2dyn should guard the division by the mass.",
        replaced);
    }
    publish(par_dyn_publisher_, par_dyn_);
  }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

rclcpp::Subscription<MactServerController::FloatArray>::SharedPtr
MactServerController::makeSubscription(const std::string & topic, ArrayBuffer & buffer)
{
  // Best-effort, keep-last-one: a late sample must be superseded, never
  // queued, so that the loop always reads the freshest matrix available. It is
  // compatible with the reliable publisher of the generated server.
  return get_node()->create_subscription<FloatArray>(
    topic, rclcpp::QoS(1).best_effort(),
    [this, &buffer](const FloatArray::SharedPtr message) {
      TimedArray sample;
      sample.data = message->data;
      sample.stamp = get_node()->now();
      sample.valid = true;
      buffer.writeFromNonRT(sample);
    });
}

std::shared_ptr<MactServerController::ArrayPublisher> MactServerController::makePublisher(
  const std::string & topic, std::size_t size)
{
  // Reliable keep-last-one, to match the subscriptions of the generated
  // server: a best-effort publisher would not connect to them at all.
  auto publisher = std::make_shared<ArrayPublisher>(
    get_node()->create_publisher<FloatArray>(topic, rclcpp::QoS(1)));
  // Size the payload once, so that publishing from the loop never allocates.
  publisher->msg_.data.assign(size, 0.0F);
  return publisher;
}

void MactServerController::publish(
  const std::shared_ptr<ArrayPublisher> & publisher, const Eigen::VectorXd & values)
{
  if (!publisher || !publisher->trylock()) {
    return;  // still sending the previous sample: drop this one
  }
  auto & data = publisher->msg_.data;
  const auto count = std::min<std::size_t>(data.size(), static_cast<std::size_t>(values.size()));
  for (std::size_t index = 0; index < count; ++index) {
    data[index] = static_cast<float>(values(static_cast<Eigen::Index>(index)));
  }
  publisher->unlockAndPublish();
}

void MactServerController::publish(
  const std::shared_ptr<ArrayPublisher> & publisher, const Vector7d & values)
{
  if (!publisher || !publisher->trylock()) {
    return;
  }
  auto & data = publisher->msg_.data;
  const auto count = std::min<std::size_t>(data.size(), static_cast<std::size_t>(kNumJoints));
  for (std::size_t index = 0; index < count; ++index) {
    data[index] = static_cast<float>(values(static_cast<Eigen::Index>(index)));
  }
  publisher->unlockAndPublish();
}

bool MactServerController::toMatrix(
  const TimedArray & source, RegressorMatrix & destination) const
{
  const auto expected = static_cast<std::size_t>(kNumJoints) *
    static_cast<std::size_t>(numParameters());
  if (source.data.size() != expected) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Expected a %d x %d regressor (%zu values) but received %zu",
      kNumJoints, numParameters(), expected, source.data.size());
    return false;
  }
  // The server flattens row by row.
  for (int row = 0; row < kNumJoints; ++row) {
    for (int column = 0; column < numParameters(); ++column) {
      destination(row, column) =
        static_cast<double>(source.data[row * numParameters() + column]);
    }
  }
  return true;
}

double MactServerController::ageMs(const TimedArray & source, const rclcpp::Time & now) const
{
  if (!source.valid) {
    return kNeverReceivedMs;
  }
  return (now - source.stamp).seconds() * 1e3;
}

int MactServerController::sanitise(Eigen::VectorXd & values)
{
  int replaced = 0;
  for (Eigen::Index index = 0; index < values.size(); ++index) {
    if (!std::isfinite(values(index))) {
      values(index) = 0.0;
      ++replaced;
    }
  }
  return replaced;
}

}  // namespace mact_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(mact_controllers::MactServerController, controller_interface::ControllerInterface)
