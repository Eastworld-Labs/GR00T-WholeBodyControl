/**
 * @file thermal_squat.hpp
 * @brief Squat, then stop, before an overheating leg motor cuts out.
 *
 * A G1 leg motor switches itself off at its winding limit (G1-021, 2026-10-27: the
 * right ankle roll went from 90 to 130 C in 18 s, reported motorstate 512, "winding
 * overheating", and the robot fell from standing). When a leg or waist motor stays at
 * `trigger_c` for `hold_s`, SONIC now squats with the planner for `squat_s` and then
 * stops (damping), so the robot settles from low down instead.
 *
 * If the planner has not started squatting after `max_wait_s`, the squat is given up
 * and the operator gets control back: stopping then would make the robot fall for
 * certain, while the motor still protects itself and the operator can still act.
 *
 * Pure logic, called from the control thread: update() once per tick with each
 * motor's hotter sensor, squat_started() once the planner is squatting.
 */
#pragma once

#include <array>
#include <cstdint>
#include <string>  // robot_parameters.hpp uses std::string without including it

#include "robot_parameters.hpp"

struct ThermalSquatConfig {
  double trigger_c = 100.0;  ///< Above normal peaks (85-88 C), 13 s before the cut-out; 0 disables.
  double hold_s = 2.0;       ///< The motor must stay this hot (sensor readings jump).
  double squat_s = 3.0;      ///< Squatting this long before the stop.
  double max_wait_s = 6.0;   ///< Give up if the planner has not started squatting by then.
  double height = 0.1;       ///< Planner squat height: its lowest, so the drop at the stop is small.
  int first_motor = 0;       ///< Hardware order: 0-11 legs, 12-14 waist. An arm cut-out
  int last_motor = 14;       ///< drops the arm, not the robot, so arms only warn.
};

class ThermalSquat {
 public:
  /// Squatting ends in Stop, or in GaveUp when the squat never started (once per run).
  enum class Phase { Off, Squatting, Stop, GaveUp };

  explicit ThermalSquat(ThermalSquatConfig config = {}) : config_(config) { hot_since_.fill(-1.0); }

  const ThermalSquatConfig& config() const { return config_; }
  bool enabled() const { return config_.trigger_c > 0.0; }
  Phase phase() const { return phase_; }
  int motor() const { return motor_; }          ///< The motor that triggered, or -1.
  int16_t temp_c() const { return temp_c_; }    ///< Its temperature then.
  bool squat_seen() const { return squat_at_ >= 0.0; }

  /// One control tick. `temps` holds each motor's hotter sensor, hardware order.
  Phase update(double t, const std::array<int16_t, G1_NUM_MOTOR>& temps) {
    if (!enabled()) return phase_;
    if (phase_ == Phase::Off) {
      for (int m = config_.first_motor; m <= config_.last_motor && m < G1_NUM_MOTOR; ++m) {
        if (temps[m] < config_.trigger_c) {
          hot_since_[m] = -1.0;
          continue;
        }
        if (hot_since_[m] < 0.0) hot_since_[m] = t;
        if (t - hot_since_[m] >= config_.hold_s) {
          phase_ = Phase::Squatting;
          triggered_at_ = t;
          motor_ = m;
          temp_c_ = temps[m];
          break;
        }
      }
    } else if (phase_ == Phase::Squatting) {
      if (squat_seen() && t - squat_at_ >= config_.squat_s) {
        phase_ = Phase::Stop;
      } else if (!squat_seen() && t - triggered_at_ >= config_.max_wait_s) {
        phase_ = Phase::GaveUp;
      }
    }
    return phase_;
  }

  /// The planner has started the squat: the stop comes `squat_s` later.
  void squat_started(double t) {
    if (phase_ == Phase::Squatting && !squat_seen()) squat_at_ = t;
  }

 private:
  ThermalSquatConfig config_;
  Phase phase_ = Phase::Off;
  std::array<double, G1_NUM_MOTOR> hot_since_{};
  double triggered_at_ = -1.0;
  double squat_at_ = -1.0;
  int motor_ = -1;
  int16_t temp_c_ = 0;
};
