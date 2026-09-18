// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#include "mact_controllers/mact_local_controller.hpp"

#include <string>

namespace mact_controllers {

CallbackReturn MactLocalController::onInitDerived()
{
  // Optional override of the parameters compiled into the generated library.
  auto_declare<std::string>("parameter_file", "");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MactLocalController::onConfigureDerived()
{
  const auto logger = get_node()->get_logger();

  if (robot_.ndof != kNumJoints) {
    RCLCPP_FATAL(
      logger, "The generated model has %d degrees of freedom, expected %d",
      robot_.ndof, kNumJoints);
    return CallbackReturn::FAILURE;
  }

  // The constructor of the generated class already holds the parameters that
  // were in the YAML at generation time; a file is only needed to override
  // them.
  const auto parameter_file = get_node()->get_parameter("parameter_file").as_string();
  if (!parameter_file.empty()) {
    if (robot_.load_par(parameter_file) != 1) {
      RCLCPP_FATAL(logger, "Could not load the parameters from '%s'", parameter_file.c_str());
      return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(logger, "Model parameters loaded from '%s'", parameter_file.c_str());
  }

  return CallbackReturn::SUCCESS;
}

bool MactLocalController::fetchInitialEstimate(Eigen::VectorXd & estimate)
{
  const auto par_reg = robot_.get_par_REG();
  if (par_reg.size() != estimate.size()) {
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "The model has %ld regressor parameters but 'num_parameters' is %ld",
      static_cast<long>(par_reg.size()), static_cast<long>(estimate.size()));
    return false;
  }
  estimate = par_reg;
  return true;
}

bool MactLocalController::updateModel(const MotionSample & motion, ModelTerms & terms)
{
  robot_.set_q(motion.q);
  robot_.set_dq(motion.dq);

  // Slotine-Li regressor on the desired motion: Y_r(q, dq, dq_d, ddq_d).
  robot_.set_dqr(motion.dq_d);
  robot_.set_ddqr(motion.ddq_d);
  terms.regressor_r = robot_.get_Yr();
  terms.has_regressor_r = true;

  // Standard regressor on the actual motion: Y(q, dq, ddq). Setting the
  // reference motion equal to the measured one is what makes get_Y() return
  // the regressor of tau = Y * pi. Only needed when the prediction error term
  // of eq. (2) is active, and it costs about as much as Y_r.
  if (needsRegressor()) {
    robot_.set_dqr(motion.dq);
    robot_.set_ddqr(motion.ddq);
    terms.regressor = robot_.get_Y();
    terms.has_regressor = true;
  }

  // Gravity regressor, only when the command has to be made gravity-free.
  if (needsGravityRegressor()) {
    terms.regressor_g = robot_.get_reg_G();
    terms.has_regressor_g = true;
  }

  // A linked model is always available: there is nothing that can go stale.
  terms.age_ms = 0.0;
  return true;
}

}  // namespace mact_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(mact_controllers::MactLocalController, controller_interface::ControllerInterface)
