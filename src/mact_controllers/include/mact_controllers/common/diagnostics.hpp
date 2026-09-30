// Copyright 2026 Giorgio Simonini
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <limits>

#include <Eigen/Core>
#include <rclcpp/time.hpp>

#include "mact_controllers/common/types.hpp"

namespace mact_controllers {

using SteadyClock = std::chrono::steady_clock;

/// Microseconds elapsed between two readings of the steady clock.
inline double elapsedUs(SteadyClock::time_point start, SteadyClock::time_point end)
{
  return std::chrono::duration<double, std::micro>(end - start).count();
}

/**
 * @brief The stages update() is timed in.
 *
 * The order is the one of the STAGE_* constants of mact_msgs/MactState, which
 * indexes the per-stage arrays with exactly these values.
 */
enum class Stage : std::size_t
{
  kState = 0,       ///< read the state, filter, reference, error check
  kModel,           ///< updateModel()
  kRegressorR,      ///<   of which Y_r
  kRegressor,       ///<   of which Y
  kRegressorG,      ///<   of which reg_G
  kGravity,         ///< gravity torque
  kControl,         ///< control law, saturation, command write
  kAdaptation,      ///< parameter update law
  kCycleEnd,        ///< onCycleEnd()
  kSnapshot,        ///< hand-over to the diagnostics timer
  kCount,
};

constexpr std::size_t kNumStages = static_cast<std::size_t>(Stage::kCount);

/// Min / max / sum of a duration over a window [us].
struct DurationStats
{
  double min{std::numeric_limits<double>::max()};
  double max{0.0};
  double sum{0.0};

  void add(double value)
  {
    min = std::min(min, value);
    max = std::max(max, value);
    sum += value;
  }

  void merge(const DurationStats & other)
  {
    min = std::min(min, other.min);
    max = std::max(max, other.max);
    sum += other.sum;
  }
};

/**
 * @brief Loop timing aggregated over the cycles since the last publication.
 *
 * Plain values only, so that it can be accumulated, merged and reset from the
 * real-time loop without allocating.
 */
struct TimingWindow
{
  uint32_t cycles{0};
  DurationStats total;
  std::array<DurationStats, kNumStages> stages;

  void merge(const TimingWindow & other)
  {
    cycles += other.cycles;
    total.merge(other.total);
    for (std::size_t stage = 0; stage < kNumStages; ++stage) {
      stages[stage].merge(other.stages[stage]);
    }
  }

  void reset() {*this = TimingWindow{};}
};

/**
 * @brief One cycle's worth of diagnostics, handed from the loop to the timer.
 *
 * Only the raw quantities are copied here: filling the message, computing the
 * errors and publishing is left to the non-real-time side. Every member is
 * fixed-size except pi_hat, which is sized once at configuration; assigning a
 * vector of the same size to it does not reallocate.
 *
 * The counters are cumulative since activation rather than per window, so that
 * a snapshot the loop could not write loses nothing: the next one carries the
 * updated count.
 */
struct DiagnosticsSnapshot
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  MotionSample motion;
  Vector7d tau_model{Vector7d::Zero()};
  Vector7d tau_command{Vector7d::Zero()};
  Vector7d tau_measured{Vector7d::Zero()};
  Vector7d tau_gravity{Vector7d::Zero()};
  Eigen::VectorXd pi_hat;

  double trajectory_time{0.0};
  double model_age_ms{0.0};
  bool model_valid{false};
  bool trajectory_valid{false};
  bool adaptation_enabled{false};

  uint32_t degraded_cycles{0};
  /// Cycles in which a reference had been received but was stale.
  uint32_t stale_reference_cycles{0};
  /// Cycles in which the configured gravity source had nothing to give.
  uint32_t gravity_unavailable_cycles{0};

  TimingWindow timing;
  /// Written by the loop since the timer last took it.
  bool fresh{false};
};

}  // namespace mact_controllers
