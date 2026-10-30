// Checks ThermalSquat's timing with no robot (the thermal_squat_test build target, or
// g++ -std=c++20 -Iinclude tests/thermal_squat_test.cpp).
#include <cstdio>
#include <cstdlib>

#include "thermal_squat.hpp"

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  std::printf("%s  %s\n", ok ? "ok  " : "FAIL", what);
  if (!ok) ++failures;
}

using Temps = std::array<int16_t, G1_NUM_MOTOR>;
using Phase = ThermalSquat::Phase;

Temps all(int16_t c) {
  Temps t;
  t.fill(c);
  return t;
}

// Runs the 50 Hz control loop from t0 to t1 with these temperatures.
Phase run(ThermalSquat& ts, double& t, double t1, const Temps& temps) {
  for (; t < t1 - 1e-9; t += 0.02) ts.update(t, temps);
  return ts.phase();
}

}  // namespace

int main() {
  {
    ThermalSquat ts;
    double t = 0.0;
    check(run(ts, t, 600.0, all(99)) == Phase::Off, "99 C for 10 min: no squat");
  }
  {
    ThermalSquat ts;
    double t = 0.0;
    Temps temps = all(50);
    temps[11] = 100;  // the right ankle roll
    run(ts, t, 1.9, temps);
    check(ts.phase() == Phase::Off, "100 C for 1.9 s: not yet");
    run(ts, t, 2.1, temps);
    check(ts.phase() == Phase::Squatting && ts.motor() == 11 && ts.temp_c() == 100,
          "100 C held 2 s on the right ankle roll: squat");
    ts.squat_started(t);
    run(ts, t, t + 2.9, all(50));
    check(ts.phase() == Phase::Squatting, "cooling does not cancel a squat once begun");
    run(ts, t, t + 0.2, all(50));
    check(ts.phase() == Phase::Stop, "stop 3 s after the squat began");
  }
  {
    ThermalSquat ts;
    double t = 0.0;
    Temps jump = all(50);
    for (int i = 0; i < 300; ++i, t += 0.02) {
      jump[3] = (i % 10 == 0) ? 105 : 70;  // a reading that jumps up for single samples
      ts.update(t, jump);
    }
    check(ts.phase() == Phase::Off, "single-sample jumps to 105 C: no squat");
  }
  {
    ThermalSquat ts;
    double t = 0.0;
    Temps arm = all(50);
    arm[22] = 120;  // right shoulder pitch
    check(run(ts, t, 5.0, arm) == Phase::Off, "a hot arm only warns");
    Temps waist = all(50);
    waist[14] = 101;
    check(run(ts, t, 8.0, waist) == Phase::Squatting, "a hot waist motor squats");
  }
  {
    ThermalSquat ts;
    double t = 0.0;
    Temps temps = all(50);
    temps[4] = 105;
    run(ts, t, 2.1, temps);
    check(ts.phase() == Phase::Squatting, "squat requested");
    run(ts, t, 2.1 + 5.9, temps);
    check(ts.phase() == Phase::Squatting, "still waiting for the planner at 5.9 s");
    run(ts, t, 2.1 + 6.1, temps);
    check(ts.phase() == Phase::GaveUp, "no squat after 6 s: give up, no stop");
    run(ts, t, t + 60.0, temps);
    check(ts.phase() == Phase::GaveUp, "still hot after giving up: no second attempt, no stop");
    ts.squat_started(t);
    run(ts, t, t + 5.0, temps);
    check(ts.phase() == Phase::GaveUp, "a late squat after giving up does not stop");
  }
  {
    ThermalSquatConfig off;
    off.trigger_c = 0.0;
    ThermalSquat ts(off);
    double t = 0.0;
    check(run(ts, t, 10.0, all(120)) == Phase::Off && !ts.enabled(), "trigger 0 disables it");
  }
  std::printf("%s\n", failures == 0 ? "all passed" : "FAILED");
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
