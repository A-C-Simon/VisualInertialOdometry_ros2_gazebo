// ROS 2 stereo-inertial wrapper for ORB-SLAM3 (ELP + HW-290).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Subscribes to rectified /cam0, /cam1 (rgb8/mono8, e.g. from ov_hw290
// stereo_splitter at 640x480) and /imu0 (HW-290 MPU6050 at 100 Hz via
// hw290_imu.py). Runs ORB-SLAM3 System::IMU_STEREO and publishes the same
// pose/path/points/tracking-image/state topics as stereo_node.cpp.
//
// Timestamp base: ROS header time (seconds) for both images and IMU, so the
// IMU buffer and stereo frames share one clock. IMU messages use arrival-time
// stamps from hw290_imu.py; the splitter forwards usb_cam stamps (with optional
// auto timestamp correction). The provisional 0.155 s cam-vs-imu offset from
// kalibr_imucam_chain.yaml is NOT compensated here - ORB-SLAM3 has no
// time-shift parameter.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <ImuTypes.h>
#include <MapPoint.h>
#include <System.h>

namespace orb_slam_ros2
{

using Image = sensor_msgs::msg::Image;
using Imu = sensor_msgs::msg::Imu;
using StereoPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

class StereoImuNode : public rclcpp::Node
{
public:
  StereoImuNode()
  : Node("orb_slam3_stereo_imu")
  {
    declare_parameters();
    read_parameters();
    validate_files();

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("tracked_map_points", 10);
    tracking_image_pub_ = create_publisher<Image>("tracking_image", 10);
    tracking_state_pub_ = create_publisher<std_msgs::msg::Int32>("tracking_state", 10);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    RCLCPP_INFO(get_logger(), "Loading ORB vocabulary; this can take a few seconds...");
    slam_ = std::make_unique<ORB_SLAM3::System>(
      vocabulary_path_, settings_path_, ORB_SLAM3::System::IMU_STEREO, use_viewer_);
    image_scale_ = slam_->GetImageScale();

    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        slam_->Reset();
        path_.poses.clear();
        imu_buffer_.clear();
        last_stereo_t_ = -1.0;
        response->success = true;
        response->message = "ORB-SLAM3 atlas reset";
      });

    imu_sub_ = create_subscription<Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&StereoImuNode::imu_callback, this, std::placeholders::_1));

    left_sub_.subscribe(this, left_topic_, rmw_qos_profile_sensor_data);
    right_sub_.subscribe(this, right_topic_, rmw_qos_profile_sensor_data);
    synchronizer_ = std::make_unique<message_filters::Synchronizer<StereoPolicy>>(
      StereoPolicy(sync_queue_size_), left_sub_, right_sub_);
    synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_sync_interval_));
    synchronizer_->registerCallback(
      std::bind(&StereoImuNode::stereo_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "ORB-SLAM3 IMU_STEREO ready: left=%s right=%s imu=%s viewer=%s scale=%.3f",
      left_topic_.c_str(), right_topic_.c_str(), imu_topic_.c_str(),
      use_viewer_ ? "on" : "off", image_scale_);
    RCLCPP_INFO(
      get_logger(),
      "Hold the rig still ~2 s after start, then move slowly with rotation+translation for IMU init.");
  }

  ~StereoImuNode() override
  {
    shutdown_slam();
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("vocabulary_path", "");
    declare_parameter<std::string>("settings_path", "");
    declare_parameter<std::string>("left_topic", "/cam0/image_raw");
    declare_parameter<std::string>("right_topic", "/cam1/image_raw");
    declare_parameter<std::string>("imu_topic", "/imu0");
    declare_parameter<int>("sync_queue_size", 10);
    declare_parameter<double>("max_sync_interval", 0.02);
    declare_parameter<bool>("use_viewer", true);
    declare_parameter<bool>("publish_tf", true);
    declare_parameter<bool>("publish_tracking_image", true);
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("camera_frame", "camera_optical_frame");
    declare_parameter<int>("max_path_length", 10000);
    declare_parameter<std::string>("trajectory_path", "vio_session.txt");
    declare_parameter<std::string>("keyframe_trajectory_path", "kf_vio_session.txt");
    declare_parameter<std::string>("timing_path", "vio_timing.txt");
  }

  void read_parameters()
  {
    vocabulary_path_ = get_parameter("vocabulary_path").as_string();
    settings_path_ = get_parameter("settings_path").as_string();
    left_topic_ = get_parameter("left_topic").as_string();
    right_topic_ = get_parameter("right_topic").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();
    sync_queue_size_ = static_cast<int>(get_parameter("sync_queue_size").as_int());
    max_sync_interval_ = get_parameter("max_sync_interval").as_double();
    use_viewer_ = get_parameter("use_viewer").as_bool();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    publish_tracking_image_ = get_parameter("publish_tracking_image").as_bool();
    map_frame_ = get_parameter("map_frame").as_string();
    camera_frame_ = get_parameter("camera_frame").as_string();
    max_path_length_ = static_cast<std::size_t>(
      std::max<int64_t>(1, get_parameter("max_path_length").as_int()));
    trajectory_path_ = get_parameter("trajectory_path").as_string();
    keyframe_trajectory_path_ = get_parameter("keyframe_trajectory_path").as_string();
    timing_path_ = get_parameter("timing_path").as_string();
  }

  void validate_files() const
  {
    if (vocabulary_path_.empty() || !std::filesystem::is_regular_file(vocabulary_path_)) {
      throw std::runtime_error("vocabulary_path is not a readable file: " + vocabulary_path_);
    }
    if (settings_path_.empty() || !std::filesystem::is_regular_file(settings_path_)) {
      throw std::runtime_error("settings_path is not a readable file: " + settings_path_);
    }
    if (sync_queue_size_ < 2) {
      throw std::runtime_error("sync_queue_size must be at least 2");
    }
  }

  void imu_callback(const Imu::SharedPtr msg)
  {
    const double t = rclcpp::Time(msg->header.stamp).seconds();
    if (!std::isfinite(t)) {
      return;
    }
    ORB_SLAM3::IMU::Point p(
      static_cast<float>(msg->linear_acceleration.x),
      static_cast<float>(msg->linear_acceleration.y),
      static_cast<float>(msg->linear_acceleration.z),
      static_cast<float>(msg->angular_velocity.x),
      static_cast<float>(msg->angular_velocity.y),
      static_cast<float>(msg->angular_velocity.z),
      t);
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (!imu_buffer_.empty() && t <= imu_buffer_.back().t) {
      // Out-of-order IMU sample; drop to keep the buffer monotonic.
      return;
    }
    imu_buffer_.push_back(p);
    // Keep ~15 s at 100 Hz.
    while (imu_buffer_.size() > 1500) {
      imu_buffer_.pop_front();
    }
    ++imu_count_;
  }

  void stereo_callback(const Image::ConstSharedPtr & left_msg, const Image::ConstSharedPtr & right_msg)
  {
    if (stop_if_slam_shutdown()) {
      return;
    }
    cv::Mat left_mono, right_mono;
    try {
      left_mono = cv_bridge::toCvCopy(left_msg, "mono8")->image;
      right_mono = cv_bridge::toCvCopy(right_msg, "mono8")->image;
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Image conversion failed: %s", error.what());
      return;
    }
    const double timestamp = rclcpp::Time(left_msg->header.stamp).seconds();
    if (!std::isfinite(timestamp)) {
      return;
    }
    if (timestamp <= last_input_timestamp_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping non-monotonic stereo pair (timestamp %.9f)", timestamp);
      return;
    }
    last_input_timestamp_ = timestamp;

    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      // Drop IMU samples older than 3 s before this frame; keep the rest so a
      // frame is never starved by earlier destructive popping.
      while (imu_buffer_.size() > 1 && imu_buffer_.front().t < timestamp - 3.0) {
        imu_buffer_.pop_front();
      }
      if (last_stereo_t_ < 0.0) {
        // First frame: bound the initial vector to the most recent ~2 s.
        while (imu_buffer_.size() > 200 &&
               imu_buffer_.front().t <= timestamp &&
               imu_buffer_.back().t - imu_buffer_.front().t > 2.0)
        {
          imu_buffer_.pop_front();
        }
      }
      for (const auto & p : imu_buffer_) {
        if (p.t > last_stereo_t_ && p.t <= timestamp) {
          vImuMeas.push_back(p);
        }
      }
      if (vImuMeas.size() > 500) {
        // Pathological pre-roll; keep the newest ~2 s at 100 Hz + margin.
        vImuMeas.erase(vImuMeas.begin(), vImuMeas.end() - 300);
      }
    }
    if (vImuMeas.size() < 2 ||
        !(vImuMeas.front().t < timestamp - kImuPreintegrationMarginS))
    {
      // Tracking::PreintegrateIMU() needs at least 2 samples spanning the
      // frame interval: one strictly older than (frame_time - mImuPer) plus a
      // closing sample. Anything less prints "Empty IMU measurements vector"
      // and segfaults on the null preintegrator. Skip the frame and keep
      // last_stereo_t_ so the next frame reuses these samples.
      // mImuPer = 1/IMU.Frequency = 0.01 s here; the margin adds slack.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping stereo frame %.3f: %zu IMU samples in (%.3f, %.3f], oldest %.3f (imu received: %zu)",
        timestamp, vImuMeas.size(), last_stereo_t_, timestamp,
        vImuMeas.empty() ? -1.0 : vImuMeas.front().t, imu_count_);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      last_stereo_t_ = timestamp;
    }

    const rclcpp::Time stamp(left_msg->header.stamp);
    process_stereo(left_mono, right_mono, timestamp, stamp, vImuMeas);
  }

  void process_stereo(
    const cv::Mat & left_input, const cv::Mat & right_input,
    double timestamp, const rclcpp::Time & stamp,
    const std::vector<ORB_SLAM3::IMU::Point> & vImuMeas)
  {
    if (left_input.empty() || right_input.empty() || left_input.size() != right_input.size()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Stereo images must be non-empty and have identical dimensions");
      return;
    }
    cv::Mat left = left_input;
    cv::Mat right = right_input;
    if (image_scale_ != 1.0F) {
      cv::resize(left_input, left, cv::Size(), image_scale_, image_scale_, cv::INTER_LINEAR);
      cv::resize(right_input, right, cv::Size(), image_scale_, image_scale_, cv::INTER_LINEAR);
    }

    const auto t1 = std::chrono::steady_clock::now();
    const Sophus::SE3f t_camera_world = slam_->TrackStereo(left, right, timestamp, vImuMeas);
    const auto t2 = std::chrono::steady_clock::now();
    if (stop_if_slam_shutdown()) {
      return;
    }
    const double track_ms =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - t1).count();
    track_times_ms_.push_back(static_cast<float>(track_ms));
    ++frame_count_;
    if (frame_count_ % 30 == 0) {
      const std::size_t n = track_times_ms_.size();
      const std::size_t from = n >= 30 ? n - 30 : 0;
      float mn = *std::min_element(track_times_ms_.begin() + from, track_times_ms_.end());
      float mx = *std::max_element(track_times_ms_.begin() + from, track_times_ms_.end());
      float mean = std::accumulate(track_times_ms_.begin() + from, track_times_ms_.end(), 0.0F) /
        static_cast<float>(n - from);
      RCLCPP_INFO(
        get_logger(), "TrackStereo last-30: mean %.1f ms min %.1f max %.1f (imu/frame %zu)",
        mean, mn, mx, vImuMeas.size());
    }

    const int state = slam_->GetTrackingState();
    std_msgs::msg::Int32 state_msg;
    state_msg.data = state;
    tracking_state_pub_->publish(state_msg);
    log_state_transition(state);

    const auto tracked_points = slam_->GetTrackedMapPoints();
    publish_points(tracked_points, stamp);
    if (publish_tracking_image_) {
      publish_tracking_image(left, tracked_points, stamp);
    }

    if (state == 2 || state == 5) {
      publish_pose(t_camera_world.inverse(), stamp);
      ++successful_pose_count_;
    }
  }

  void publish_pose(const Sophus::SE3f & t_world_camera, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;
    const Eigen::Vector3f translation = t_world_camera.translation();
    const Eigen::Quaternionf rotation(t_world_camera.rotationMatrix());
    pose.pose.position.x = translation.x();
    pose.pose.position.y = translation.y();
    pose.pose.position.z = translation.z();
    pose.pose.orientation.x = rotation.x();
    pose.pose.orientation.y = rotation.y();
    pose.pose.orientation.z = rotation.z();
    pose.pose.orientation.w = rotation.w();
    pose_pub_->publish(pose);

    path_.header = pose.header;
    path_.poses.push_back(pose);
    if (path_.poses.size() > max_path_length_) {
      path_.poses.erase(path_.poses.begin());
    }
    path_pub_->publish(path_);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = pose.header;
      transform.child_frame_id = camera_frame_;
      transform.transform.translation.x = pose.pose.position.x;
      transform.transform.translation.y = pose.pose.position.y;
      transform.transform.translation.z = pose.pose.position.z;
      transform.transform.rotation = pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void publish_points(
    const std::vector<ORB_SLAM3::MapPoint *> & tracked_points, const rclcpp::Time & stamp)
  {
    std::vector<Eigen::Vector3f> positions;
    positions.reserve(tracked_points.size());
    for (auto * point : tracked_points) {
      if (point != nullptr && !point->isBad()) {
        const Eigen::Vector3f position = point->GetWorldPos();
        if (position.allFinite()) {
          positions.push_back(position);
        }
      }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = map_frame_;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(positions.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto & position : positions) {
      *x = position.x();
      *y = position.y();
      *z = position.z();
      ++x;
      ++y;
      ++z;
    }
    points_pub_->publish(cloud);
  }

  void publish_tracking_image(
    const cv::Mat & left_mono, const std::vector<ORB_SLAM3::MapPoint *> & tracked_points,
    const rclcpp::Time & stamp)
  {
    cv::Mat visualization;
    cv::cvtColor(left_mono, visualization, cv::COLOR_GRAY2BGR);
    const auto keypoints = slam_->GetTrackedKeyPointsUn();
    const std::size_t count = std::min(keypoints.size(), tracked_points.size());
    for (std::size_t i = 0; i < count; ++i) {
      if (tracked_points[i] != nullptr && !tracked_points[i]->isBad()) {
        cv::circle(visualization, keypoints[i].pt, 2, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
      }
    }
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = camera_frame_;
    tracking_image_pub_->publish(*cv_bridge::CvImage(header, "bgr8", visualization).toImageMsg());
  }

  void log_state_transition(int state)
  {
    if (state == last_tracking_state_) {
      return;
    }
    static const std::vector<std::string> names = {
      "NO_IMAGES_YET", "NOT_INITIALIZED", "OK", "RECENTLY_LOST", "LOST", "OK_KLT"};
    const std::string name = state >= 0 && static_cast<std::size_t>(state) < names.size() ?
      names[static_cast<std::size_t>(state)] : "UNKNOWN(" + std::to_string(state) + ")";
    RCLCPP_INFO(get_logger(), "Tracking state: %s", name.c_str());
    last_tracking_state_ = state;
  }

  bool stop_if_slam_shutdown()
  {
    if (!slam_ || !slam_->isShutDown()) {
      return false;
    }
    if (!shutdown_requested_) {
      shutdown_requested_ = true;
      RCLCPP_INFO(get_logger(), "ORB-SLAM3 viewer requested shutdown; stopping ROS 2 node");
      rclcpp::shutdown();
    }
    return true;
  }

  void shutdown_slam()
  {
    if (shutdown_ || !slam_) {
      return;
    }
    shutdown_ = true;
    RCLCPP_INFO(get_logger(), "Shutting down ORB-SLAM3...");
    if (!slam_->isShutDown()) {
      slam_->Shutdown();
    }
    if (successful_pose_count_ > 0 && !trajectory_path_.empty()) {
      slam_->SaveTrajectoryEuRoC(trajectory_path_);
      RCLCPP_INFO(get_logger(), "Saved trajectory to %s", trajectory_path_.c_str());
    }
    if (successful_pose_count_ > 0 && !keyframe_trajectory_path_.empty()) {
      slam_->SaveKeyFrameTrajectoryEuRoC(keyframe_trajectory_path_);
      RCLCPP_INFO(
        get_logger(), "Saved keyframe trajectory to %s", keyframe_trajectory_path_.c_str());
    }
    if (successful_pose_count_ == 0) {
      RCLCPP_INFO(get_logger(), "No valid poses were tracked; trajectory files were not written");
    }
    if (!timing_path_.empty() && !track_times_ms_.empty()) {
      std::ofstream out(timing_path_);
      if (out.is_open()) {
        out << "# frame track_ms\n";
        for (std::size_t i = 0; i < track_times_ms_.size(); ++i) {
          out << i << " " << track_times_ms_[i] << "\n";
        }
        RCLCPP_INFO(
          get_logger(), "Saved %zu frame timings to %s", track_times_ms_.size(),
          timing_path_.c_str());
      }
    }
    if (!track_times_ms_.empty()) {
      const float mean = std::accumulate(track_times_ms_.begin(), track_times_ms_.end(), 0.0F) /
        static_cast<float>(track_times_ms_.size());
      const float mx = *std::max_element(track_times_ms_.begin(), track_times_ms_.end());
      RCLCPP_INFO(
        get_logger(), "TrackStereo overall: frames=%zu mean=%.1f ms max=%.1f ms",
        track_times_ms_.size(), mean, mx);
    }
  }

  std::string vocabulary_path_;
  std::string settings_path_;
  std::string left_topic_;
  std::string right_topic_;
  std::string imu_topic_;
  int sync_queue_size_{10};
  double max_sync_interval_{0.02};
  bool use_viewer_{true};
  bool publish_tf_{true};
  bool publish_tracking_image_{true};
  std::string map_frame_;
  std::string camera_frame_;
  std::size_t max_path_length_{10000};
  std::string trajectory_path_;
  std::string keyframe_trajectory_path_;
  std::string timing_path_;

  std::unique_ptr<ORB_SLAM3::System> slam_;
  float image_scale_{1.0F};
  bool shutdown_{false};
  bool shutdown_requested_{false};
  int last_tracking_state_{-1};
  double last_input_timestamp_{-1.0};
  double last_stereo_t_{-1.0};
  std::size_t successful_pose_count_{0};
  std::size_t frame_count_{0};
  std::size_t imu_count_{0};
  std::vector<float> track_times_ms_;

  std::mutex imu_mutex_;
  std::deque<ORB_SLAM3::IMU::Point> imu_buffer_;

  // Minimum age of the oldest IMU sample relative to the stereo frame time
  // before TrackStereo may be called. Tracking::PreintegrateIMU() consumes
  // samples older than (frame_time - mImuPer) plus one closing sample, where
  // mImuPer = 1/IMU.Frequency (0.01 s at 100 Hz). Anything less segfaults.
  static constexpr double kImuPreintegrationMarginS = 0.012;

  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  message_filters::Subscriber<Image> left_sub_;
  message_filters::Subscriber<Image> right_sub_;
  std::unique_ptr<message_filters::Synchronizer<StereoPolicy>> synchronizer_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<Image>::SharedPtr tracking_image_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tracking_state_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Path path_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
};

}  // namespace orb_slam_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<orb_slam_ros2::StereoImuNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("orb_slam3_stereo_imu"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
