#include "ov_hw290/imu_protocol.hpp"
#include <iostream>
#include <string>
int main() {
  hw290::DeviceClock clock;
  int64_t receipt; std::string packet;
  while (std::cin >> receipt >> packet) {
    try {
      auto s=hw290::parse_sample(packet); auto stamp=clock.update(s,receipt);
      std::cout << (stamp ? std::to_string(*stamp) : "warmup") << ' ' << clock.dropped
                << ' ' << s.accel_lsb_per_g << ' ' << s.gyro_lsb_per_dps << '\n';
    } catch (const std::exception&) { std::cout << "error\n"; }
  }
}
