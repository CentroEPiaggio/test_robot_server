// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#include "controller_tests/lissajous_trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace controller_tests {

void LissajousTrajectory::configure(
  const Configuration & configuration, const Vector7d & start)
{
  configuration_ = configuration;
  start_ = start;
}

void LissajousTrajectory::quinticScaling(
  double t, double duration, double & s, double & ds, double & dds)
{
  if (duration <= 0.0) {
    s = 1.0;
    ds = 0.0;
    dds = 0.0;
    return;
  }

  const double tau = std::clamp(t / duration, 0.0, 1.0);
  // s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5
  //   s(0) = s'(0) = s''(0) = 0,  s(1) = 1,  s'(1) = s''(1) = 0
  s = tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));

  if (tau <= 0.0 || tau >= 1.0) {
    ds = 0.0;
    dds = 0.0;
    return;
  }
  const double inverse_duration = 1.0 / duration;
  ds = (30.0 * tau * tau - 60.0 * tau * tau * tau + 30.0 * tau * tau * tau * tau) *
    inverse_duration;
  dds = (60.0 * tau - 180.0 * tau * tau + 120.0 * tau * tau * tau) *
    inverse_duration * inverse_duration;
}

LissajousTrajectory::Phase LissajousTrajectory::phaseAt(double t) const
{
  if (t < configuration_.approach_duration) {
    return Phase::kApproach;
  }
  if (t < totalDuration()) {
    return Phase::kLissajous;
  }
  return Phase::kHold;
}

TrajectoryPoint LissajousTrajectory::sample(double t) const
{
  TrajectoryPoint point;

  // ------------------------------------------------------------- approach --
  if (t < configuration_.approach_duration) {
    double s = 0.0;
    double ds = 0.0;
    double dds = 0.0;
    quinticScaling(t, configuration_.approach_duration, s, ds, dds);

    const Vector7d delta = configuration_.centre - start_;
    point.position = start_ + s * delta;
    point.velocity = ds * delta;
    point.acceleration = dds * delta;
    return point;
  }

  // ------------------------------------------------------------ Lissajous --
  const double tau = t - configuration_.approach_duration;
  if (tau >= configuration_.run_duration) {
    point.position = configuration_.centre;
    return point;  // hold, at rest
  }

  // Envelope: quintic rise, unit plateau, quintic fall. The fall is the same
  // polynomial run backwards, so the whole envelope stays C2.
  double envelope = 1.0;
  double envelope_rate = 0.0;
  double envelope_acceleration = 0.0;

  const double ramp = configuration_.ramp_duration;
  const double time_to_end = configuration_.run_duration - tau;
  if (ramp > 0.0 && tau < ramp) {
    quinticScaling(tau, ramp, envelope, envelope_rate, envelope_acceleration);
  } else if (ramp > 0.0 && time_to_end < ramp) {
    quinticScaling(time_to_end, ramp, envelope, envelope_rate, envelope_acceleration);
    // Mirrored in time: the odd derivative changes sign.
    envelope_rate = -envelope_rate;
  }

  for (int joint = 0; joint < kNumJoints; ++joint) {
    const double omega = 2.0 * M_PI * configuration_.frequency(joint);
    const double amplitude = configuration_.amplitude(joint);
    const double angle = omega * tau + configuration_.phase(joint);

    const double u = amplitude * std::sin(angle);
    const double du = amplitude * omega * std::cos(angle);
    const double ddu = -amplitude * omega * omega * std::sin(angle);

    // Product rule on q = centre + s(t) * u(t).
    point.position(joint) = configuration_.centre(joint) + envelope * u;
    point.velocity(joint) = envelope_rate * u + envelope * du;
    point.acceleration(joint) =
      envelope_acceleration * u + 2.0 * envelope_rate * du + envelope * ddu;
  }

  return point;
}

}  // namespace controller_tests
