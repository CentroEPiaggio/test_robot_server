// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#include "mact_controllers/common/joint_state_estimator.hpp"

namespace mact_controllers {

void JointStateEstimator::configure(double dq_cutoff_hz, double ddq_cutoff_hz) {
  dq_filter_.setCutoff(dq_cutoff_hz);
  ddq_filter_.setCutoff(ddq_cutoff_hz);
}

void JointStateEstimator::reset() {
  dq_filter_.reset();
  ddq_filter_.reset();
  dq_.setZero();
  ddq_.setZero();
  previous_dq_raw_.setZero();
  has_previous_sample_ = false;
}

void JointStateEstimator::update(const Vector7d& dq_raw, double dt) {
  dq_ = dq_filter_.update(dq_raw, dt);

  if (has_previous_sample_ && dt > 0.0) {
    const Vector7d ddq_raw = (dq_raw - previous_dq_raw_) / dt;
    ddq_ = ddq_filter_.update(ddq_raw, dt);
  } else {
    // No finite difference available on the first sample after a reset.
    ddq_.setZero();
  }

  previous_dq_raw_ = dq_raw;
  has_previous_sample_ = true;
}

}  // namespace mact_controllers
