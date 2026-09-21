// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <memory>
#include <string>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <rclcpp/logger.hpp>

#include "mact_controllers/common/types.hpp"

namespace mact_controllers {

/**
 * @brief Where the gravity torque removed from the command comes from.
 *
 * Both libfranka and franka_ign_ros2_control add their own gravity
 * compensation on top of the commanded torque, so the command has to be made
 * gravity-free. *Which* gravity is subtracted decides how exact the
 * cancellation is:
 *
 *  - kEstimate:     reg_G(q) * pi_hat, the controller's own estimate. Needs no
 *                   extra interface, and for the server-based controller not
 *                   even a model — but it leaves a residual
 *                   (G_applied - G_estimated) acting on the joints, which is
 *                   also a bias in the prediction error term of eq. (2).
 *  - kUrdfKdl:      the very computation the Gazebo plugin performs, namely
 *                   KDL::ChainDynParam::JntToGravity over the URDF chain. Same
 *                   model, same solver, same gravity vector, so the
 *                   cancellation in simulation is exact.
 *  - kFrankaModel:  the robot's own gravity vector, read through
 *                   franka_semantic_components::FrankaRobotModel. This is
 *                   literally what libfranka compensates, so the cancellation
 *                   on hardware is exact. Only available with
 *                   franka_hardware; it does not exist in Gazebo.
 *  - kNone:         command the torque as computed, for a simulator that does
 *                   not compensate gravity itself.
 */
enum class GravitySource
{
  kNone,
  kEstimate,
  kUrdfKdl,
  kFrankaModel,
};

/// Parse the `gravity.source` parameter. Returns false on an unknown name.
bool gravitySourceFromString(const std::string & name, GravitySource & source);

/// Name of a source, as it is written in the configuration.
const char * toString(GravitySource source);

/**
 * @brief Gravity torques from the URDF, computed as the Gazebo plugin does.
 *
 * franka_ign_ros2_control builds a KDL chain from the robot description, from
 * the root link to the tip link, and calls JntToGravity with a hard-coded
 * earth gravity of {0, 0, -9.8}. Reproducing it here with the same library and
 * the same model gives the same numbers, which is what makes the cancellation
 * in simulation exact rather than approximate.
 *
 * configure() does all the parsing and allocation; compute() is allocation-free
 * and safe to call from the control loop.
 */
class UrdfGravityModel
{
public:
  /**
   * @param urdf     The robot description.
   * @param gravity  Earth gravity vector [m/s^2]; use {0, 0, -9.8} to match
   *                 the Gazebo plugin exactly.
   * @param root     Chain root; empty means the root of the URDF, which is
   *                 what the plugin uses.
   * @param tip      Chain tip; empty means the last link with a fixed or
   *                 revolute parent, found the way the plugin finds it.
   * @param error_message Filled in when the call fails.
   */
  bool configure(
    const std::string & urdf,
    const std::array<double, 3> & gravity,
    const std::string & root,
    const std::string & tip,
    std::string & error_message);

  /// Gravity torques for the given configuration.
  const Vector7d & compute(const Vector7d & q);

  const std::string & rootLink() const {return root_;}
  const std::string & tipLink() const {return tip_;}

private:
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> solver_;
  KDL::JntArray positions_;
  KDL::JntArray torques_;
  Vector7d gravity_torque_{Vector7d::Zero()};
  std::string root_;
  std::string tip_;
};

}  // namespace mact_controllers
