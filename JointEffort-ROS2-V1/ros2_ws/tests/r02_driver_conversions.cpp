// Test the exact conversions compiled into the patched driver. No ROS or serial access.
#include <cassert>
#include <cmath>
#include <iostream>
#include "sts_hardware_interface/sts_conversions.hpp"
int main() {
  namespace c = sts_hardware_interface::conversions;
  for (int direction : {-1, 1}) {
    for (int raw = 500; raw <= 3500; ++raw) {
      double q = c::raw_position_to_radians(raw, 2048, direction);
      assert(c::radians_to_raw_position(q, 2048, direction) == raw);
      assert(std::abs(q - direction*(raw-2048)*c::STEPS_TO_RAD) < 1e-12);
    }
    assert(c::raw_velocity_to_rad_s(100,direction)*direction > 0);
  }
  assert(c::raw_position_to_radians(2000,2048) == c::raw_position_to_radians(2000,2048,-1));
  assert(c::radians_to_raw_position(100,2048,1)==4095);
  assert(c::radians_to_raw_position(100,2048,-1)==0);
  std::cout << "PASS: 6002 count/angle round trips, both directions, velocity signs and endpoint clamps.\n";
}
