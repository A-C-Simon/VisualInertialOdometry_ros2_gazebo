#include "ov_hw290/imu_protocol.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace {
int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct SerialPort {
  int fd=-1;
  ~SerialPort() { if (fd>=0) close(fd); }
  void open_port(const std::string& path, int baud) {
    if (baud!=115200) throw std::runtime_error("HW290 firmware requires baud=115200");
    fd=open(path.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
    if (fd<0) throw std::runtime_error("Cannot open "+path+": "+std::strerror(errno));
    if (flock(fd,LOCK_EX|LOCK_NB)<0) throw std::runtime_error("Serial port already in use: "+path);
    termios t{};
    if (tcgetattr(fd,&t)<0) throw std::runtime_error("Cannot read serial settings");
    cfmakeraw(&t); cfsetispeed(&t,B115200); cfsetospeed(&t,B115200);
    t.c_cflag |= CLOCAL|CREAD; t.c_cflag &= ~CRTSCTS;
    t.c_cc[VMIN]=1; t.c_cc[VTIME]=0;
    if (tcsetattr(fd,TCSANOW,&t)<0) throw std::runtime_error("Cannot configure serial port");
    int bits=TIOCM_DTR|TIOCM_RTS;
    // Nano auto-reset on open matches pyserial; PTYs do not implement modem lines.
    if (ioctl(fd,TIOCMBIS,&bits)<0 && errno!=ENOTTY && errno!=EINVAL)
      throw std::runtime_error("Cannot assert serial DTR/RTS");
    tcflush(fd,TCIFLUSH);
  }
};
}

class Hw290Imu : public rclcpp::Node {
 public:
  Hw290Imu() : Node("hw290_imu") {
    auto path=declare_parameter<std::string>("port","/dev/ttyUSB0");
    baud_=declare_parameter<int>("baud",115200);
    frame_=declare_parameter<std::string>("frame_id","imu");
    const auto raw_path=declare_parameter<std::string>("raw_log_path","");
    if (!raw_path.empty()) {
      raw_log_.open(raw_path,std::ios::out|std::ios::trunc);
      if (!raw_log_) throw std::runtime_error("Cannot open IMU raw log: "+raw_path);
      raw_log_ << "# host_receipt_unix_ns serial_record (includes sequence, MCU clock and raw temperature)\n";
    }
    if (get_parameter("use_sim_time").as_bool())
      throw std::runtime_error("Live HW290 acquisition requires use_sim_time=false");
    pub_=create_publisher<sensor_msgs::msg::Imu>("/imu0",100);
    serial_.open_port(path,baud_);
    offset_=now().nanoseconds()-steady_ns();
    started_=steady_ns();
    RCLCPP_INFO(get_logger(),"Awaiting IMU1/IMU2/IMU3 records; one-second device-clock warmup (C++ event-driven serial)");
  }
  void run(rclcpp::executors::SingleThreadedExecutor& executor) {
    int64_t next_diagnostics=started_+5000000000LL, next_spin=started_;
    while (rclcpp::ok()) {
      pollfd event{serial_.fd,POLLIN,0};
      const int ready=poll(&event,1,100); // Sleep until bytes arrive; bound shutdown latency.
      if (ready<0 && errno!=EINTR) throw std::runtime_error("Serial poll failed");
      if (ready>0) {
        if (event.revents & (POLLERR|POLLHUP|POLLNVAL))
          throw std::runtime_error("IMU serial disconnected or device error");
        if (event.revents & POLLIN) receive();
      }
      const auto current=steady_ns();
      if (last_valid_ && current-*last_valid_>3000000000LL)
        throw std::runtime_error("IMU stream stopped for over 3 seconds; check power and I2C wiring");
      if (!published_ && current-started_>12000000000LL)
        throw std::runtime_error("IMU startup timed out; check firmware diagnostics");
      if (current>=next_diagnostics) {
        const double elapsed=(current-diagnostic_time_)*1e-9;
        const double hz=diagnostic_time_ ? (published_-diagnostic_count_)/elapsed : 0.0;
        RCLCPP_INFO(get_logger(),"published=%llu corrupt=%llu sequence_gaps=%llu saturated=%llu",
          static_cast<unsigned long long>(published_),static_cast<unsigned long long>(bad_),
          static_cast<unsigned long long>(clock_.dropped),static_cast<unsigned long long>(saturated_));
        if (diagnostic_time_)
          RCLCPP_INFO(get_logger(),"IMU delivery rate=%.1f Hz (expected 100 Hz)",hz);
        diagnostic_time_=current; diagnostic_count_=published_;
        next_diagnostics=current+5000000000LL;
      }
      // Service parameter requests without a high-frequency polling ROS timer.
      if (current>=next_spin) { executor.spin_some(); next_spin=current+250000000LL; }
    }
  }
 private:
  void receive() {
    char bytes[8192]; const auto size=read(serial_.fd,bytes,sizeof(bytes));
    const auto receipt=steady_ns();
    if (size<0) {
      if (errno==EAGAIN || errno==EINTR) return;
      throw std::runtime_error("IMU serial read failed");
    }
    if (!size) throw std::runtime_error("IMU serial EOF");
    buffer_.append(bytes,static_cast<size_t>(size));
    if (buffer_.size()>16384) throw std::runtime_error("serial buffer overflow; restart sensor pipeline");
    size_t end;
    while ((end=buffer_.find('\n'))!=std::string::npos) {
      const std::string line=buffer_.substr(0,end);
      if (raw_log_.is_open()) {
        raw_log_ << receipt+offset_ << ' ' << line << '\n';
        if (!raw_log_) throw std::runtime_error("IMU raw log write failed");
      }
      const auto remaining=buffer_.size()-end-1;
      buffer_.erase(0,end+1);
      if (line.rfind("IMU1,",0)!=0 && line.rfind("IMU2,",0)!=0 && line.rfind("IMU3,",0)!=0) {
        if (line.rfind("MPU a/g:",0)==0) throw std::runtime_error("Legacy firmware: flash timestamped IMU firmware");
        RCLCPP_INFO(get_logger(),"Firmware: %s",line.c_str());
        if (line.find("ERROR IMU")!=std::string::npos)
          throw std::runtime_error("IMU firmware reported failure: "+line);
        continue;
      }
      hw290::Sample s;
      try { s=hw290::parse_sample(line); }
      catch (const std::invalid_argument&) { ++bad_; continue; }
      if (previous_device_us_) {
        const uint32_t dt=s.micros-*previous_device_us_;
        if (dt>50000 && dt<2000000)
          throw std::runtime_error("IMU unhealthy: MCU sample gap >50 ms; source rate too low for VIO");
      }
      previous_device_us_=s.micros;
      if (!rate_start_) { rate_start_=receipt; rate_device_start_=s.micros; rate_count_=0; }
      ++rate_count_;
      if (receipt-*rate_start_>=1000000000LL) {
        const double host_rate=(rate_count_-1)*1e9/(receipt-*rate_start_);
        const uint32_t device_elapsed=s.micros-rate_device_start_;
        const double source_rate=device_elapsed ? (rate_count_-1)*1e6/device_elapsed : 0;
        const bool source_bad=source_rate<80 || source_rate>120;
        const bool host_bad=host_rate<80 || host_rate>120;
        if (source_bad || (!rate_ready_ && host_bad))
          throw std::runtime_error("IMU unhealthy: host/source rate outside 80-120 Hz (host="+
            std::to_string(host_rate)+", source="+std::to_string(source_rate)+")");
        // Acquisition intervals remain authoritative during serial catch-up.
        // A short host stall can be followed by >120 packets/s without the
        // sensor changing its 100 Hz rate. Require sustained slow delivery.
        slow_delivery_windows_=host_rate<80 ? slow_delivery_windows_+1 : 0;
        if (slow_delivery_windows_>=3)
          throw std::runtime_error("IMU unhealthy: delivery below 80 Hz for three consecutive windows");
        if (rate_ready_ && host_bad)
          RCLCPP_WARN(get_logger(),"IMU delivery jitter: host %.1f Hz, verified source %.1f Hz",host_rate,source_rate);
        if (!rate_ready_) {
          RCLCPP_INFO(get_logger(),"IMU_READY: verified host/source rate %.1f/%.1f Hz",host_rate,source_rate);
          rate_ready_=true;
        }
        rate_start_=receipt; rate_device_start_=s.micros; rate_count_=1;
      }
      const auto observed=receipt-static_cast<int64_t>((line.size()+1+remaining)*10e9/baud_);
      const auto stamp=clock_.update(s,observed); // Reset/duplicate is fatal, not a corrupt packet.
      last_valid_=receipt;
      if (!stamp) continue;
      bool saturated=false;
      for (int i=0;i<3;++i) saturated |= std::abs(s.accel[i])>=32760 || std::abs(s.gyro[i])>=32760;
      if (saturated) {
        ++saturated_; RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),1000,"IMU range saturated; measurement rejected");
        continue;
      }
      sensor_msgs::msg::Imu msg;
      msg.header.stamp=rclcpp::Time(*stamp+offset_,RCL_SYSTEM_TIME);
      msg.header.frame_id=frame_; msg.orientation_covariance[0]=-1.0;
      const double acc=9.80665/s.accel_lsb_per_g, gyro=std::acos(-1.0)/(180.0*s.gyro_lsb_per_dps);
      msg.linear_acceleration.x=s.accel[0]*acc; msg.linear_acceleration.y=s.accel[1]*acc; msg.linear_acceleration.z=s.accel[2]*acc;
      msg.angular_velocity.x=s.gyro[0]*gyro; msg.angular_velocity.y=s.gyro[1]*gyro; msg.angular_velocity.z=s.gyro[2]*gyro;
      pub_->publish(msg); ++published_;
    }
  }
  SerialPort serial_;
  std::ofstream raw_log_;
  hw290::DeviceClock clock_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  std::string frame_,buffer_;
  int baud_=115200;
  int64_t offset_=0,started_=0;
  std::optional<int64_t> last_valid_;
  std::optional<uint32_t> previous_device_us_;
  std::optional<int64_t> rate_start_;
  uint32_t rate_device_start_=0;
  uint64_t rate_count_=0,diagnostic_count_=0;
  int64_t diagnostic_time_=0;
  bool rate_ready_=false;
  unsigned slow_delivery_windows_=0;
  uint64_t published_=0,bad_=0,saturated_=0;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  int status=0;
  try {
    auto node=std::make_shared<Hw290Imu>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node); node->run(executor); executor.remove_node(node);
  } catch (const std::exception& error) {
    if (rclcpp::ok()) { RCLCPP_FATAL(rclcpp::get_logger("hw290_imu"),"%s",error.what()); status=1; }
  }
  rclcpp::shutdown(); return status;
}
