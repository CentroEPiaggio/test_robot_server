// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#include "mact_controllers/common/adaptation_law.hpp"

#include <algorithm>

namespace mact_controllers {

bool AdaptationLaw::configure(
  int num_parameters,
  const std::vector<double> & link_gains,
  const std::vector<double> & param_gains,
  double gamma,
  const std::vector<double> & r_t,
  std::string & error_message)
{
  if (num_parameters <= 0 || num_parameters % kParametersPerLink != 0) {
    error_message = "num_parameters must be a positive multiple of " +
      std::to_string(kParametersPerLink);
    return false;
  }
  const int num_links = num_parameters / kParametersPerLink;

  if (static_cast<int>(link_gains.size()) != num_links) {
    error_message = "R_p_link must have " + std::to_string(num_links) +
      " entries (one per link block) but has " + std::to_string(link_gains.size());
    return false;
  }
  if (static_cast<int>(param_gains.size()) != kParametersPerLink) {
    error_message = "R_p_param must have " + std::to_string(kParametersPerLink) +
      " entries but has " + std::to_string(param_gains.size());
    return false;
  }
  if (static_cast<int>(r_t.size()) != kNumJoints) {
    error_message = "R_t must have " + std::to_string(kNumJoints) +
      " entries but has " + std::to_string(r_t.size());
    return false;
  }
  // R_p and R_t are required to be positive definite; being diagonal here,
  // that reduces to non-negative entries (zero meaning "do not adapt").
  if (std::any_of(link_gains.begin(), link_gains.end(), [](double g) {return g < 0.0;}) ||
    std::any_of(param_gains.begin(), param_gains.end(), [](double g) {return g < 0.0;}) ||
    std::any_of(r_t.begin(), r_t.end(), [](double g) {return g < 0.0;}))
  {
    error_message = "adaptation gains must be non-negative";
    return false;
  }
  if (gamma < 0.0) {
    error_message = "gamma must be non-negative";
    return false;
  }

  gamma_ = gamma;
  for (int joint = 0; joint < kNumJoints; ++joint) {
    r_t_(joint) = r_t[joint];
  }

  gain_diagonal_.resize(num_parameters);
  for (int link = 0; link < num_links; ++link) {
    for (int parameter = 0; parameter < kParametersPerLink; ++parameter) {
      gain_diagonal_(link * kParametersPerLink + parameter) =
        link_gains[link] * param_gains[parameter];
    }
  }

  estimate_.setZero(num_parameters);
  rate_.setZero(num_parameters);
  prediction_error_.setZero();
  return true;
}

void AdaptationLaw::setEstimate(const Eigen::VectorXd & estimate)
{
  if (estimate.size() == estimate_.size()) {
    estimate_ = estimate;
  }
}

void AdaptationLaw::update(
  const RegressorMatrix & regressor_r,
  const Vector7d & error_rate,
  const RegressorMatrix & regressor,
  const Vector7d & torque,
  bool has_regressor,
  double dt)
{
  if (dt <= 0.0 || regressor_r.cols() != estimate_.size()) {
    return;
  }

  // Tracking term: Y_r^T * e_dot.
  rate_.noalias() = regressor_r.transpose() * error_rate;

  // Prediction error term: gamma * Y^T * R_t * (tau - Y*pi_hat).
  if (gamma_ != 0.0 && has_regressor && regressor.cols() == estimate_.size()) {
    prediction_error_.noalias() = torque - regressor * estimate_;
    prediction_error_ = r_t_.cwiseProduct(prediction_error_);
    rate_.noalias() += gamma_ * (regressor.transpose() * prediction_error_);
  }

  // R_p is diagonal, so the product collapses to a coefficient-wise scaling.
  estimate_ += dt * gain_diagonal_.cwiseProduct(rate_);
}

}  // namespace mact_controllers
