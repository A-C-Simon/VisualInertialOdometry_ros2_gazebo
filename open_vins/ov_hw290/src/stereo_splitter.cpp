#include <cstring>
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

class StereoSplitter final : public rclcpp::Node {
public:
  StereoSplitter() : Node("hw290_stereo_splitter") {
    const auto calibration_file = declare_parameter<std::string>("calibration_file", "");
    auto_timestamp_correction_ = declare_parameter<bool>("auto_timestamp_correction", true);
    stamp_offset_sec_ = declare_parameter<double>("timestamp_offset_sec", 0.0);
    monochrome_ = declare_parameter<bool>("monochrome", true);
    rectify_images_ = declare_parameter<bool>("rectify_images", true);
    if (calibration_file.empty()) {
      throw std::runtime_error("calibration_file parameter is required");
    }
    cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      throw std::runtime_error("Could not open stereo calibration: " + calibration_file);
    }
    cv::Mat k0, k1, d0, d1, r0, r1, p0, p1;
    fs["K1"] >> k0;
    fs["K2"] >> k1;
    fs["D1"] >> d0;
    fs["D2"] >> d1;
    fs["R1"] >> r0;
    fs["R2"] >> r1;
    fs["P1"] >> p0;
    fs["P2"] >> p1;
    const int calibrated_width = static_cast<int>(fs["image_width"]);
    const int calibrated_height = static_cast<int>(fs["image_height"]);
    if (k0.empty() || k1.empty() || d0.empty() || d1.empty() ||
        r0.empty() || r1.empty() || p0.empty() || p1.empty() ||
        calibrated_width <= 0 || calibrated_height <= 0) {
      throw std::runtime_error("Stereo calibration is incomplete");
    }
    const double sx = 640.0 / calibrated_width;
    const double sy = 480.0 / calibrated_height;
    for (cv::Mat *matrix : {&k0, &k1, &p0, &p1}) {
      matrix->row(0) *= sx;
      matrix->row(1) *= sy;
    }
    cv::initUndistortRectifyMap(k0, d0, r0, p0(cv::Rect(0, 0, 3, 3)),
                                cv::Size(640, 480), CV_32FC1, left_map_x_, left_map_y_);
    cv::initUndistortRectifyMap(k1, d1, r1, p1(cv::Rect(0, 0, 3, 3)),
                                cv::Size(640, 480), CV_32FC1, right_map_x_, right_map_y_);
    intrinsic_[0] = k0;
    intrinsic_[1] = k1;
    distortion_[0] = d0;
    distortion_[1] = d1;
    projection_[0] = p0;
    projection_[1] = p1;
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    left_ = create_publisher<sensor_msgs::msg::Image>("/cam0/image_raw", qos);
    right_ = create_publisher<sensor_msgs::msg::Image>("/cam1/image_raw", qos);
    left_info_ = create_publisher<sensor_msgs::msg::CameraInfo>("/cam0/camera_info", qos);
    right_info_ = create_publisher<sensor_msgs::msg::CameraInfo>("/cam1/camera_info", qos);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr image) { split(image); });
    RCLCPP_INFO(get_logger(), "%s 640x480 stereo views from /image_raw using %s",
                rectify_images_ ? "Rectifying" : "Publishing raw", calibration_file.c_str());
    RCLCPP_INFO(get_logger(), "Publishing %s stereo images", monochrome_ ? "mono8" : "rgb8");
  }

private:
  sensor_msgs::msg::CameraInfo info(const std_msgs::msg::Header &header, bool right) {
    sensor_msgs::msg::CameraInfo out;
    out.header = header;
    out.width = 640;
    out.height = 480;
    out.distortion_model = "plumb_bob";
    const size_t index = right ? 1 : 0;
    const cv::Mat &p = projection_[index];
    const cv::Mat &k = rectify_images_ ? p(cv::Rect(0, 0, 3, 3)) : intrinsic_[index];
    out.k = {k.at<double>(0, 0), k.at<double>(0, 1), k.at<double>(0, 2),
             k.at<double>(1, 0), k.at<double>(1, 1), k.at<double>(1, 2),
             k.at<double>(2, 0), k.at<double>(2, 1), k.at<double>(2, 2)};
    out.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    if (rectify_images_) {
      out.d = {0, 0, 0, 0, 0};
    } else {
      const cv::Mat flattened = distortion_[index].reshape(1, 1);
      out.d.resize(flattened.cols);
      for (int col = 0; col < flattened.cols; ++col) {
        out.d[col] = flattened.at<double>(0, col);
      }
    }
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 4; ++col) {
        out.p[row * 4 + col] =
            rectify_images_ ? p.at<double>(row, col)
                            : (col < 3 ? k.at<double>(row, col) : 0.0);
      }
    }
    return out;
  }

  void split(const sensor_msgs::msg::Image::ConstSharedPtr &source) {
    if (source->width != 1280 || source->height != 480 ||
        source->encoding != "rgb8" || source->step < 1280 * 3 ||
        source->data.size() < static_cast<size_t>(source->step) * source->height) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Expected a 1280x480 rgb8 side by side frame");
      return;
    }
    sensor_msgs::msg::Image l, r;
    l.header = source->header;
    r.header = source->header;
    if (auto_timestamp_correction_ && !timestamp_calibrated_) {
      const double age_sec = (get_clock()->now() - rclcpp::Time(source->header.stamp)).seconds();
      // The installed usb_cam converts V4L2 monotonic time to epoch time with
      // a process-start-dependent subsecond error. Estimate that constant
      // error from the first second of live frames, while leaving bag playback
      // (whose source timestamps are far in the past) untouched.
      if (age_sec >= 0.0 && age_sec < 1.5) {
        timestamp_ages_.push_back(age_sec);
      }
      if (timestamp_ages_.size() >= 30) {
        auto middle = timestamp_ages_.begin() + timestamp_ages_.size() / 2;
        std::nth_element(timestamp_ages_.begin(), middle, timestamp_ages_.end());
        const double median_age = *middle;
        if (median_age > 0.03) {
          stamp_offset_sec_ += median_age - 0.01;
        }
        timestamp_calibrated_ = true;
        RCLCPP_INFO(get_logger(), "Camera timestamp correction %.3f s (median input age %.3f s)",
                    stamp_offset_sec_, median_age);
      }
    }
    if (stamp_offset_sec_ != 0.0) {
      const auto corrected = rclcpp::Time(l.header.stamp) + rclcpp::Duration::from_seconds(stamp_offset_sec_);
      l.header.stamp = corrected;
      r.header.stamp = l.header.stamp;
    }
    l.header.frame_id = "cam0";
    r.header.frame_id = "cam1";
    l.height = r.height = 480;
    l.width = r.width = 640;
    l.encoding = r.encoding = monochrome_ ? "mono8" : "rgb8";
    l.is_bigendian = r.is_bigendian = source->is_bigendian;
    l.step = r.step = monochrome_ ? 640 : 640 * 3;
    l.data.resize(static_cast<size_t>(l.step) * l.height);
    r.data.resize(static_cast<size_t>(r.step) * r.height);
    cv::Mat raw_left(480, 640, CV_8UC3,
                     const_cast<uint8_t *>(source->data.data()), source->step);
    cv::Mat raw_right(480, 640, CV_8UC3,
                      const_cast<uint8_t *>(source->data.data() + 640 * 3), source->step);
    cv::Mat output_left, output_right;
    if (rectify_images_) {
      cv::remap(raw_left, output_left, left_map_x_, left_map_y_, cv::INTER_LINEAR);
      cv::remap(raw_right, output_right, right_map_x_, right_map_y_, cv::INTER_LINEAR);
    } else {
      // Cropped side-by-side views retain the source stride, so clone them
      // before copying their pixels into independent ROS image messages.
      output_left = raw_left.clone();
      output_right = raw_right.clone();
    }
    // Match cv_bridge's RGB8 -> MONO8 conversion after rectification exactly.
    // Converting before remap would change pixels through rounding.
    if (monochrome_) {
      cv::cvtColor(output_left, output_left, cv::COLOR_RGB2GRAY);
      cv::cvtColor(output_right, output_right, cv::COLOR_RGB2GRAY);
    }
    std::memcpy(l.data.data(), output_left.data, l.data.size());
    std::memcpy(r.data.data(), output_right.data, r.data.size());
    left_->publish(l);
    right_->publish(r);
    left_info_->publish(info(l.header, false));
    right_info_->publish(info(r.header, true));
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_, right_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_, right_info_;
  cv::Mat left_map_x_, left_map_y_, right_map_x_, right_map_y_;
  cv::Mat intrinsic_[2], distortion_[2];
  cv::Mat projection_[2];
  bool auto_timestamp_correction_ = true;
  bool monochrome_ = true;
  bool rectify_images_ = true;
  bool timestamp_calibrated_ = false;
  double stamp_offset_sec_ = 0.0;
  std::vector<double> timestamp_ages_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoSplitter>());
  rclcpp::shutdown();
  return 0;
}
