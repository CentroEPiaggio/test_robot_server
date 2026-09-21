// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <mact_msgs/msg/mact_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "controller_tests/lissajous_trajectory.hpp"

namespace controller_tests {

/**
 * @brief Publishes the Lissajous test trajectory at the control rate.
 *
 * The same node feeds both MACT controllers, on exactly the same topic and
 * with exactly the same parameters, which is what makes the two runs
 * comparable.
 *
 * The node waits for the measured joint state, uses it as the starting pose of
 * the approach phase, and then streams (q_d, dq_d, ddq_d) at `publish_rate_hz`
 * until the motion is over. Time is taken from the node clock rather than
 * counted in timer ticks, so the trajectory stays correct in real (or
 * simulated) time even if the timer jitters, and it works unchanged under
 * use_sim_time.
 *
 * Before the motion starts the node holds the start pose on the reference
 * topic and waits for the controller to confirm, on its state topic, that it
 * is receiving. Publishing straight away would lose whatever part of the
 * approach goes out before discovery between the two completes -- measured at
 * 2.3 s in one run and 0.3 s in another of the same launch file, which both
 * delivers a step in q_d to the controller and leaves two runs at different
 * points of the parameter convergence by the time the Lissajous begins. The
 * handshake removes that variability, and it is why time_from_start is a
 * trustworthy origin for the analysis.
 */
class TrajectoryPublisherNode : public rclcpp::Node
{
public:
  TrajectoryPublisherNode()
  : Node("lissajous_trajectory_publisher")
  {
    declareParameters();
    if (!readParameters()) {
      throw std::runtime_error("invalid trajectory configuration");
    }

    publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
      trajectory_topic_, rclcpp::QoS(1));

    // Keep-last-one: only the most recent joint state matters, and only until
    // the starting pose has been captured.
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::QoS(1),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        jointStateCallback(message);
      });

    // The node clock, not a wall timer, so the same code runs unchanged under
    // use_sim_time.
    if (wait_for_controller_) {
      controller_state_subscription_ = create_subscription<mact_msgs::msg::MactState>(
        controller_state_topic_, rclcpp::QoS(1),
        [this](const mact_msgs::msg::MactState::SharedPtr message) {
          // The controller only reports this once it is actually receiving the
          // reference, which is exactly the condition to wait for.
          if (message->trajectory_valid) {
            controller_ready_ = true;
          }
        });
    }

    const auto period = rclcpp::Duration::from_seconds(1.0 / publish_rate_hz_);
    timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() {onTimer();});

    RCLCPP_INFO(
      get_logger(),
      "Waiting for '%s' to capture the starting pose; will then publish on '%s' at %.0f Hz",
      joint_state_topic_.c_str(), trajectory_topic_.c_str(), publish_rate_hz_);
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("trajectory_topic", "/mact/trajectory");
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
    declare_parameter<std::string>("arm_id", "fr3");
    declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
    declare_parameter<double>("publish_rate_hz", 1000.0);

    declare_parameter<std::vector<double>>("centre", std::vector<double>{});
    declare_parameter<std::vector<double>>("amplitude", std::vector<double>{});
    declare_parameter<std::vector<double>>("frequency", std::vector<double>{});
    declare_parameter<std::vector<double>>("phase", std::vector<double>{});

    declare_parameter<double>("approach_duration", 5.0);
    declare_parameter<double>("ramp_duration", 2.0);
    declare_parameter<double>("run_duration", 30.0);

    declare_parameter<std::vector<double>>("position_min", std::vector<double>{});
    declare_parameter<std::vector<double>>("position_max", std::vector<double>{});

    // Stop publishing (and let the launch file shut everything down) once the
    // motion is over, instead of holding the centre pose forever.
    declare_parameter<std::string>("controller_state_topic", "/mact/state");
    declare_parameter<bool>("wait_for_controller", true);
    declare_parameter<double>("handshake_timeout_s", 30.0);

    declare_parameter<bool>("stop_when_finished", false);
    declare_parameter<double>("hold_duration", 2.0);
  }

  /// Read a parameter that must contain exactly kNumJoints values.
  bool readVector(const std::string & name, Vector7d & target)
  {
    const auto values = get_parameter(name).as_double_array();
    if (static_cast<int>(values.size()) != kNumJoints) {
      RCLCPP_FATAL(
        get_logger(), "'%s' must have %d entries but has %zu", name.c_str(), kNumJoints,
        values.size());
      return false;
    }
    for (int joint = 0; joint < kNumJoints; ++joint) {
      target(joint) = values[joint];
    }
    return true;
  }

  bool readParameters()
  {
    trajectory_topic_ = get_parameter("trajectory_topic").as_string();
    joint_state_topic_ = get_parameter("joint_state_topic").as_string();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    if (!(publish_rate_hz_ > 0.0)) {
      RCLCPP_FATAL(get_logger(), "'publish_rate_hz' must be positive");
      return false;
    }

    joint_names_ = get_parameter("joints").as_string_array();
    if (joint_names_.empty()) {
      const auto arm_id = get_parameter("arm_id").as_string();
      for (int joint = 1; joint <= kNumJoints; ++joint) {
        joint_names_.push_back(arm_id + "_joint" + std::to_string(joint));
      }
    }
    if (static_cast<int>(joint_names_.size()) != kNumJoints) {
      RCLCPP_FATAL(get_logger(), "'joints' must list %d names", kNumJoints);
      return false;
    }

    if (!readVector("centre", configuration_.centre) ||
      !readVector("amplitude", configuration_.amplitude) ||
      !readVector("frequency", configuration_.frequency) ||
      !readVector("phase", configuration_.phase))
    {
      return false;
    }

    configuration_.approach_duration = get_parameter("approach_duration").as_double();
    configuration_.ramp_duration = get_parameter("ramp_duration").as_double();
    configuration_.run_duration = get_parameter("run_duration").as_double();
    controller_state_topic_ = get_parameter("controller_state_topic").as_string();
    wait_for_controller_ = get_parameter("wait_for_controller").as_bool();
    handshake_timeout_s_ = get_parameter("handshake_timeout_s").as_double();
    stop_when_finished_ = get_parameter("stop_when_finished").as_bool();
    hold_duration_ = get_parameter("hold_duration").as_double();

    if (configuration_.ramp_duration * 2.0 > configuration_.run_duration) {
      RCLCPP_FATAL(
        get_logger(),
        "'run_duration' (%.2f s) must leave room for a rise and a fall of "
        "'ramp_duration' (%.2f s each)",
        configuration_.run_duration, configuration_.ramp_duration);
      return false;
    }

    // Optional joint range check: the whole Lissajous must fit inside it.
    const auto minimum = get_parameter("position_min").as_double_array();
    const auto maximum = get_parameter("position_max").as_double_array();
    if (!minimum.empty() || !maximum.empty()) {
      if (static_cast<int>(minimum.size()) != kNumJoints ||
        static_cast<int>(maximum.size()) != kNumJoints)
      {
        RCLCPP_FATAL(get_logger(), "'position_min'/'position_max' must have %d entries each",
          kNumJoints);
        return false;
      }
      for (int joint = 0; joint < kNumJoints; ++joint) {
        const double low = configuration_.centre(joint) -
          std::abs(configuration_.amplitude(joint));
        const double high = configuration_.centre(joint) +
          std::abs(configuration_.amplitude(joint));
        if (low < minimum[joint] || high > maximum[joint]) {
          RCLCPP_FATAL(
            get_logger(),
            "Joint %d would sweep [%.3f, %.3f] rad, outside the allowed "
            "[%.3f, %.3f]: reduce 'amplitude' or move 'centre'",
            joint + 1, low, high, minimum[joint], maximum[joint]);
          return false;
        }
      }
    }

    return true;
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    if (have_start_pose_) {
      return;  // the starting pose is captured once and never revised
    }

    Vector7d start;
    for (int joint = 0; joint < kNumJoints; ++joint) {
      const auto found =
        std::find(message->name.begin(), message->name.end(), joint_names_[joint]);
      if (found == message->name.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Joint '%s' is not in '%s' yet", joint_names_[joint].c_str(),
          joint_state_topic_.c_str());
        return;
      }
      const auto index = static_cast<std::size_t>(found - message->name.begin());
      if (index >= message->position.size()) {
        return;
      }
      start(joint) = message->position[index];
    }

    start_pose_ = start;
    trajectory_.configure(configuration_, start);
    have_start_pose_ = true;
    wait_started_ = now();

    RCLCPP_INFO(
      get_logger(),
      "Start pose captured; holding it on '%s'%s",
      trajectory_topic_.c_str(),
      wait_for_controller_ ?
      " until the controller reports that it is receiving" : "");
  }

  /// Publish one sample of the reference.
  void publish(const TrajectoryPoint & point, double elapsed)
  {
    trajectory_msgs::msg::JointTrajectoryPoint message;
    message.positions.resize(kNumJoints);
    message.velocities.resize(kNumJoints);
    message.accelerations.resize(kNumJoints);
    for (int joint = 0; joint < kNumJoints; ++joint) {
      message.positions[joint] = point.position(joint);
      message.velocities[joint] = point.velocity(joint);
      message.accelerations[joint] = point.acceleration(joint);
    }
    // The motion clock the analysis keys on: 0 while the start pose is held,
    // then counting up from the beginning of the approach.
    message.time_from_start = rclcpp::Duration::from_seconds(elapsed);
    publisher_->publish(message);
  }

  /**
   * @brief Is the controller ready for the motion to begin?
   *
   * Times out rather than hanging, so a run still happens (with a warning) if
   * the controller never answers.
   */
  bool handshakeComplete()
  {
    if (!wait_for_controller_ || controller_ready_) {
      return true;
    }
    if ((now() - wait_started_).seconds() > handshake_timeout_s_) {
      RCLCPP_WARN(
        get_logger(),
        "No controller state on '%s' after %.1f s; starting anyway. The first part of the "
        "approach may not reach the controller.",
        controller_state_topic_.c_str(), handshake_timeout_s_);
      return true;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for the controller to report that it is receiving '%s' ...",
      trajectory_topic_.c_str());
    return false;
  }

  void onTimer()
  {
    if (!have_start_pose_) {
      return;  // nothing sensible to publish yet
    }

    // Hold the start pose, at rest, until the controller is connected. The
    // motion clock stays at zero throughout, so the analysis can tell this
    // phase apart from the approach, and so can the controller: it does not
    // begin adapting until the clock starts running.
    if (!motion_started_) {
      TrajectoryPoint hold;
      hold.position = start_pose_;
      publish(hold, 0.0);
      if (!handshakeComplete()) {
        return;
      }
      motion_started_ = true;
      start_time_ = now();
      RCLCPP_INFO(
        get_logger(),
        "Starting: %.1f s approach to the centre, then %.1f s of Lissajous "
        "(%.1f s ramp in and out)",
        configuration_.approach_duration, configuration_.run_duration,
        configuration_.ramp_duration);
      return;
    }

    const double elapsed = (now() - start_time_).seconds();
    const TrajectoryPoint point = trajectory_.sample(elapsed);
    publish(point, elapsed);

    reportPhase(elapsed);

    if (stop_when_finished_ && elapsed > trajectory_.totalDuration() + hold_duration_) {
      RCLCPP_INFO(get_logger(), "Trajectory finished after %.1f s; shutting down", elapsed);
      rclcpp::shutdown();
    }
  }

  /// Log once per phase transition, so the bag can be segmented afterwards.
  void reportPhase(double elapsed)
  {
    const auto phase = trajectory_.phaseAt(elapsed);
    if (phase == last_phase_) {
      return;
    }
    last_phase_ = phase;
    switch (phase) {
      case LissajousTrajectory::Phase::kApproach:
        RCLCPP_INFO(get_logger(), "[%.2f s] approach", elapsed);
        break;
      case LissajousTrajectory::Phase::kLissajous:
        RCLCPP_INFO(get_logger(), "[%.2f s] Lissajous", elapsed);
        break;
      case LissajousTrajectory::Phase::kHold:
        RCLCPP_INFO(get_logger(), "[%.2f s] holding the centre pose", elapsed);
        break;
    }
  }

  // ------------------------------------------------------------ parameters --
  std::string trajectory_topic_;
  std::string joint_state_topic_;
  std::vector<std::string> joint_names_;
  double publish_rate_hz_{1000.0};
  bool stop_when_finished_{false};
  double hold_duration_{2.0};

  LissajousTrajectory::Configuration configuration_;
  LissajousTrajectory trajectory_;

  // ----------------------------------------------------------------- state --
  bool have_start_pose_{false};
  bool motion_started_{false};
  bool controller_ready_{false};
  Vector7d start_pose_{Vector7d::Zero()};
  rclcpp::Time wait_started_;
  rclcpp::Time start_time_;
  LissajousTrajectory::Phase last_phase_{LissajousTrajectory::Phase::kHold};

  std::string controller_state_topic_;
  bool wait_for_controller_{true};
  double handshake_timeout_s_{30.0};

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<mact_msgs::msg::MactState>::SharedPtr controller_state_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace controller_tests

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<controller_tests::TrajectoryPublisherNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("lissajous_trajectory_publisher"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
