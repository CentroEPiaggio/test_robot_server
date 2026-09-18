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
 * Note how Y_r and Y are obtained from the same generated function family.
 * Both depend on {q, dq, dqr, ddqr}; the difference is which motion is written
 * into the reference slots before the call:
 *
 *   - Y_r(q, dq, dq_d, ddq_d): reference = the *desired* motion (Slotine-Li);
 *   - Y  (q, dq, dq,   ddq  ): reference = the *actual* motion, which turns the
 *                              standard regressor into tau = Y * pi.
 *
 * A single linked instance can do both because it is re-evaluated on demand.
 * A server holding one (dqr, ddqr) pair cannot, which is exactly why the
 * server-side Y has to be generated as a function of ddq.
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
