// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <memory>

#include <thunder_franka/thunder_franka.h>

#include "mact_controllers/mact_controller_base.hpp"

namespace mact_controllers {

/**
 * @brief MACT controller with the Thunder library linked in-process.
 *
 * The "linked" baseline of the extended abstract: the controller owns a
 * thunder_franka instance and evaluates the regressors itself, with no
 * serialisation and no round trip. Everything else — gains, filtering,
 * reference handling, the control law, the update law, saturation and the
 * diagnostics — is inherited from MactControllerBase and is therefore bit-for-
 * bit the same as in MactServerController.
 *
 * The two regressors of eq. (2) are taken from the generated library directly:
 *
 *   - Y_r(q, dq, dq_d, ddq_d), the Slotine-Li regressor on the desired motion,
 *     with the reference slots dqr/ddqr carrying q_d's derivatives;
 *   - Y(q, dq, ddq), the standard regressor on the actual motion, which takes
 *     the measured acceleration as its own input.
 *
 * They share no input, so both can be evaluated in one pass over the state.
 */
class MactLocalController : public MactControllerBase
{
protected:
  CallbackReturn onInitDerived() override;
  CallbackReturn onConfigureDerived() override;

  bool updateModel(const MotionSample & motion, ModelTerms & terms) override;
  bool fetchInitialEstimate(Eigen::VectorXd & estimate) override;

private:
  thunder_franka robot_;
};

}  // namespace mact_controllers
