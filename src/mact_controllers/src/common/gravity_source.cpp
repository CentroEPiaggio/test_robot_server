// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#include "mact_controllers/common/gravity_source.hpp"

#include <queue>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

namespace mact_controllers {

namespace {

/// The tip link as franka_ign_ros2_control finds it: a breadth-first walk from
/// the root, keeping the last leaf that is visited.
std::string findTipLink(const urdf::Model & model)
{
  const auto root = model.getRoot();
  if (!root) {
    return {};
  }

  std::queue<urdf::LinkConstSharedPtr> pending;
  pending.push(root);
  std::string tip;

  while (!pending.empty()) {
    const auto link = pending.front();
    pending.pop();
    if (link->child_links.empty()) {
      tip = link->name;
    } else {
      for (const auto & child : link->child_links) {
        pending.push(child);
      }
    }
  }
  return tip;
}

}  // namespace

bool gravitySourceFromString(const std::string & name, GravitySource & source)
{
  if (name == "none") {
    source = GravitySource::kNone;
  } else if (name == "estimate") {
    source = GravitySource::kEstimate;
  } else if (name == "urdf_kdl") {
    source = GravitySource::kUrdfKdl;
  } else if (name == "franka_model") {
    source = GravitySource::kFrankaModel;
  } else {
    return false;
  }
  return true;
}

const char * toString(GravitySource source)
{
  switch (source) {
    case GravitySource::kNone: return "none";
    case GravitySource::kEstimate: return "estimate";
    case GravitySource::kUrdfKdl: return "urdf_kdl";
    case GravitySource::kFrankaModel: return "franka_model";
  }
  return "unknown";
}

bool UrdfGravityModel::configure(
  const std::string & urdf,
  const std::array<double, 3> & gravity,
  const std::string & root,
  const std::string & tip,
  std::string & error_message)
{
  urdf::Model model;
  if (!model.initString(urdf)) {
    error_message = "the robot description could not be parsed";
    return false;
  }

  root_ = root.empty() ? (model.getRoot() ? model.getRoot()->name : std::string{}) : root;
  tip_ = tip.empty() ? findTipLink(model) : tip;
  if (root_.empty() || tip_.empty()) {
    error_message = "could not determine the chain root or tip from the description";
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    error_message = "could not build a KDL tree from the description";
    return false;
  }
  if (!tree.getChain(root_, tip_, chain_)) {
    error_message = "no chain from '" + root_ + "' to '" + tip_ + "'";
    return false;
  }

  const auto joints = chain_.getNrOfJoints();
  if (joints != static_cast<unsigned int>(kNumJoints)) {
    error_message = "the chain from '" + root_ + "' to '" + tip_ + "' has " +
      std::to_string(joints) + " joints, expected " + std::to_string(kNumJoints);
    return false;
  }

  solver_ = std::make_unique<KDL::ChainDynParam>(
    chain_, KDL::Vector(gravity[0], gravity[1], gravity[2]));
  positions_.resize(joints);
  torques_.resize(joints);
  gravity_torque_.setZero();
  return true;
}

const Vector7d & UrdfGravityModel::compute(const Vector7d & q)
{
  if (!solver_) {
    return gravity_torque_;
  }
  for (int joint = 0; joint < kNumJoints; ++joint) {
    positions_(joint) = q(joint);
  }
  // On failure the previous value is kept rather than a zero injected, which
  // would show up as a torque step on the robot.
  if (solver_->JntToGravity(positions_, torques_) == KDL::SolverI::E_NOERROR) {
    for (int joint = 0; joint < kNumJoints; ++joint) {
      gravity_torque_(joint) = torques_(joint);
    }
  }
  return gravity_torque_;
}

}  // namespace mact_controllers
