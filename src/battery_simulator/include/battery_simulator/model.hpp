#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace battery_simulator {
struct Model {
  double soc = 80.0;
  double idle_rate = 12.0;       // percentage points / simulation hour
  double moving_rate = 30.0;
  double charge_rate = 60.0;
  bool connected = false;

  void validate() const {
    if (!std::isfinite(soc) || soc < 0 || soc > 100 ||
        !std::isfinite(idle_rate) || idle_rate < 0 ||
        !std::isfinite(moving_rate) || moving_rate < 0 ||
        !std::isfinite(charge_rate) || charge_rate <= 0)
      throw std::invalid_argument("SOC must be 0..100; discharge rates >=0; charge rate >0; all finite");
  }
  double rate(bool moving) const {
    if (connected) return soc < 100 ? charge_rate : 0.0;
    return soc > 0 ? -(moving ? moving_rate : idle_rate) : 0.0;
  }
  void advance(double seconds, bool moving) {
    // Caller resets its time baseline on clock jumps. No discharge while paused.
    if (!std::isfinite(seconds) || seconds <= 0 || seconds > 30.0) return;
    if (moving) connected = false;  // simulation failsafe, not a real undocking procedure
    soc = std::clamp(soc + rate(moving) * seconds / 3600.0, 0.0, 100.0);
  }
};
}  // namespace battery_simulator
