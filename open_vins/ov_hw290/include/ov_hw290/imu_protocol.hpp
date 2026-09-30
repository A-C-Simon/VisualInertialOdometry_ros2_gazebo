#pragma once
#include <algorithm>
#include <array>
#include <charconv>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace hw290 {
struct Sample {
  uint32_t sequence, micros;
  std::array<int, 3> accel, gyro;
  int temperature;
  double accel_lsb_per_g, gyro_lsb_per_dps;
};
inline int64_t integer(std::string_view text, int base = 10) {
  int64_t value = 0;
  const auto result = std::from_chars(text.data(), text.data()+text.size(), value, base);
  if (text.empty() || result.ec != std::errc{} || result.ptr != text.data()+text.size())
    throw std::invalid_argument("invalid integer");
  return value;
}
inline Sample parse_sample(std::string_view line) {
  while (!line.empty() && (line.back()=='\r' || line.back()=='\n')) line.remove_suffix(1);
  const auto star = line.rfind('*');
  if (star == std::string_view::npos || line.size()-star != 3)
    throw std::invalid_argument("invalid checksum field");
  const auto payload = line.substr(0, star);
  unsigned char checksum = 0;
  for (const unsigned char byte : payload) checksum ^= byte;
  if (integer(line.substr(star+1),16) != checksum) throw std::invalid_argument("checksum mismatch");
  std::vector<std::string_view> fields;
  size_t start = 0;
  for (size_t i=0; i<=payload.size(); ++i) {
    if (i==payload.size() || payload[i]==',') {
      fields.push_back(payload.substr(start,i-start)); start=i+1;
    }
  }
  if (fields.size()!=10 || (fields[0]!="IMU1" && fields[0]!="IMU2" && fields[0]!="IMU3"))
    throw std::invalid_argument("unsupported record");
  std::array<int64_t,9> values{};
  for (size_t i=0; i<values.size(); ++i) {
    values[i]=integer(fields[i+1]);
    if (i<2 ? (values[i]<0 || values[i]>UINT32_MAX) : (values[i]<-32768 || values[i]>32767))
      throw std::invalid_argument("measurement out of range");
  }
  const double accel_scale = fields[0]=="IMU3" ? 4096.0 : fields[0]=="IMU2" ? 8192.0 : 16384.0;
  const double gyro_scale = fields[0]=="IMU3" ? 32.8 : fields[0]=="IMU2" ? 65.5 : 131.0;
  Sample s{static_cast<uint32_t>(values[0]), static_cast<uint32_t>(values[1]), {}, {},
           static_cast<int>(values[8]), accel_scale, gyro_scale};
  for (int i=0;i<3;++i) { s.accel[i]=values[i+2]; s.gyro[i]=values[i+5]; }
  return s;
}
class DeviceClock {
 public:
  uint64_t dropped = 0;
  std::optional<int64_t> update(const Sample& s, int64_t receipt_ns) {
    uint32_t delta_us = 0;
    if (last_us_) {
      delta_us = s.micros-*last_us_;
      const uint32_t delta_sequence = s.sequence-*sequence_;
      if (delta_sequence==0 || delta_sequence>0x7fffffff)
        throw std::runtime_error("duplicate/out-of-order sample or MCU reset; restart bridge");
      if (delta_us==0 || delta_us>2000000)
        throw std::runtime_error("MCU clock reset or sample gap >2 s; restart bridge");
      dropped += delta_sequence-1;
    }
    last_us_=s.micros; sequence_=s.sequence;
    elapsed_ns_+=static_cast<int64_t>(delta_us)*1000;
    const auto candidate=receipt_ns-elapsed_ns_;
    if (++count_<=100) {
      anchor_=anchor_ ? std::min(*anchor_,candidate) : candidate;
      stamp_=*anchor_+elapsed_ns_;
      return std::nullopt;
    }
    const auto predicted=stamp_+static_cast<int64_t>(delta_us)*1000;
    const auto error=receipt_ns-predicted;
    if (!have_minimum_ || error<minimum_error_) minimum_error_=error;
    have_minimum_=true;
    const int64_t limit=static_cast<int64_t>(delta_us)*10;
    const auto correction=std::clamp(static_cast<int64_t>(minimum_error_*0.02),-limit,limit);
    stamp_=predicted+correction;
    if (count_%100==0) have_minimum_=false;
    return stamp_;
  }
 private:
  std::optional<uint32_t> last_us_, sequence_;
  std::optional<int64_t> anchor_;
  int64_t minimum_error_=0;
  bool have_minimum_=false;
  uint64_t count_=0;
  int64_t elapsed_ns_=0, stamp_=0;
};
}  // namespace hw290
