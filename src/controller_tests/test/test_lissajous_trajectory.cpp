// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include <cmath>

#include "controller_tests/lissajous_trajectory.hpp"

using controller_tests::LissajousTrajectory;
using controller_tests::TrajectoryPoint;
using controller_tests::Vector7d;
using controller_tests::kNumJoints;

namespace {

LissajousTrajectory::Configuration makeConfiguration()
{
  LissajousTrajectory::Configuration configuration;
  configuration.centre << 0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398;
  configuration.amplitude << 0.30, 0.20, 0.30, 0.20, 0.40, 0.30, 0.50;
  configuration.frequency << 0.13, 0.17, 0.19, 0.23, 0.29, 0.31, 0.37;
  configuration.phase << 0.0, 1.570796, 3.141593, 4.712389, 0.0, 1.570796, 3.141593;
  configuration.approach_duration = 5.0;
  configuration.ramp_duration = 2.0;
  configuration.run_duration = 30.0;
  return configuration;
}

LissajousTrajectory makeTrajectory()
{
  Vector7d start;
  start << 0.1, -0.5, 0.2, -2.0, -0.3, 1.2, 0.5;
  LissajousTrajectory trajectory;
  trajectory.configure(makeConfiguration(), start);
  return trajectory;
}

}  // namespace

TEST(LissajousTrajectory, StartsAtTheMeasuredPoseAtRest)
{
  Vector7d start;
  start << 0.1, -0.5, 0.2, -2.0, -0.3, 1.2, 0.5;
  const auto trajectory = makeTrajectory();

  const TrajectoryPoint point = trajectory.sample(0.0);
  EXPECT_TRUE(point.position.isApprox(start, 1e-12));
  EXPECT_LT(point.velocity.norm(), 1e-12);
  EXPECT_LT(point.acceleration.norm(), 1e-12);
}

TEST(LissajousTrajectory, ReachesTheCentreAtRestAfterTheApproach)
{
  const auto configuration = makeConfiguration();
  const auto trajectory = makeTrajectory();

  const TrajectoryPoint point = trajectory.sample(configuration.approach_duration - 1e-9);
  EXPECT_TRUE(point.position.isApprox(configuration.centre, 1e-6));
  EXPECT_LT(point.velocity.norm(), 1e-6);
  EXPECT_LT(point.acceleration.norm(), 1e-6);
}

TEST(LissajousTrajectory, EndsAtTheCentreAtRest)
{
  const auto configuration = makeConfiguration();
  const auto trajectory = makeTrajectory();

  const TrajectoryPoint point = trajectory.sample(trajectory.totalDuration() + 1.0);
  EXPECT_TRUE(point.position.isApprox(configuration.centre, 1e-9));
  EXPECT_LT(point.velocity.norm(), 1e-9);
  EXPECT_LT(point.acceleration.norm(), 1e-9);
}

/// The whole point of the quintic approach and of the envelope: no jump in
/// position, velocity or acceleration anywhere, which a torque controller
/// would otherwise turn into an impulse.
TEST(LissajousTrajectory, IsContinuousThroughout)
{
  const auto trajectory = makeTrajectory();
  const double dt = 1e-4;
  const double end = trajectory.totalDuration() + 1.0;

  TrajectoryPoint previous = trajectory.sample(0.0);
  for (double t = dt; t <= end; t += dt) {
    const TrajectoryPoint point = trajectory.sample(t);
    // Bounds are generous multiples of what the trajectory itself can change
    // in one step; a discontinuity would be orders of magnitude larger.
    EXPECT_LT((point.position - previous.position).cwiseAbs().maxCoeff(), 1e-2) << "at t=" << t;
    EXPECT_LT((point.velocity - previous.velocity).cwiseAbs().maxCoeff(), 1e-2) << "at t=" << t;
    EXPECT_LT(
      (point.acceleration - previous.acceleration).cwiseAbs().maxCoeff(), 1e-1) << "at t=" << t;
    previous = point;
  }
}

/// The reported velocity and acceleration must really be the derivatives of
/// the reported position: the controller feeds them forward through the model,
/// so an inconsistency would show up as a systematic tracking error.
TEST(LissajousTrajectory, DerivativesMatchFiniteDifferences)
{
  const auto trajectory = makeTrajectory();
  const double h = 1e-6;

  for (double t : {1.0, 4.0, 5.5, 6.5, 12.0, 20.0, 33.5, 34.5}) {
    const TrajectoryPoint before = trajectory.sample(t - h);
    const TrajectoryPoint at = trajectory.sample(t);
    const TrajectoryPoint after = trajectory.sample(t + h);

    const Vector7d velocity = (after.position - before.position) / (2.0 * h);
    const Vector7d acceleration =
      (after.position - 2.0 * at.position + before.position) / (h * h);

    EXPECT_LT((velocity - at.velocity).cwiseAbs().maxCoeff(), 1e-4) << "velocity at t=" << t;
    EXPECT_LT((acceleration - at.acceleration).cwiseAbs().maxCoeff(), 1e-2)
      << "acceleration at t=" << t;
  }
}

TEST(LissajousTrajectory, StaysWithinTheRequestedAmplitude)
{
  const auto configuration = makeConfiguration();
  const auto trajectory = makeTrajectory();

  for (double t = configuration.approach_duration; t <= trajectory.totalDuration(); t += 1e-3) {
    const TrajectoryPoint point = trajectory.sample(t);
    for (int joint = 0; joint < kNumJoints; ++joint) {
      const double excursion = std::abs(point.position(joint) - configuration.centre(joint));
      EXPECT_LE(excursion, configuration.amplitude(joint) + 1e-9)
        << "joint " << joint << " at t=" << t;
    }
  }
}
