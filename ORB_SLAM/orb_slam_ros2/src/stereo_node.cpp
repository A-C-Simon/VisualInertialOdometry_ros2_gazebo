// ROS 2 stereo wrapper for ORB-SLAM3.
// SPDX-License-Identifier: GPL-3.0-or-later

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <MapPoint.h>
#include <System.h>

namespace orb_slam_ros2
{

using Image = sensor_msgs::msg::Image;
using StereoPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

class StereoNode : public rclcpp::Node
{
public:
  StereoNode()
  : Node("orb_slam3_stereo"), steady_start_(std::chrono::steady_clock::now())
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
      vocabulary_path_, settings_path_, ORB_SLAM3::System::STEREO, use_viewer_);
    image_scale_ = slam_->GetImageScale();

    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        slam_->Reset();
        path_.poses.clear();
        response->success = true;
        response->message = "ORB-SLAM3 atlas reset";
      });
    localization_service_ = create_service<std_srvs::srv::Trigger>(
      "activate_localization_mode",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        slam_->ActivateLocalizationMode();
        response->success = true;
        response->message = "Localization-only mode requested";
      });
    slam_service_ = create_service<std_srvs::srv::Trigger>(
      "activate_slam_mode",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        slam_->DeactivateLocalizationMode();
        response->success = true;
        response->message = "Mapping mode requested";
      });

    if (input_mode_ == "device") {
      start_device();
    } else if (input_mode_ == "topics") {
      start_topic_subscriptions();
    } else {
      throw std::runtime_error("input_mode must be 'device' or 'topics'");
    }

    RCLCPP_INFO(
      get_logger(), "ORB-SLAM3 ready: input=%s, viewer=%s, image scale=%.3f",
      input_mode_.c_str(), use_viewer_ ? "on" : "off", image_scale_);
  }

  ~StereoNode() override
  {
    shutdown_slam();
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("vocabulary_path", "");
    declare_parameter<std::string>("settings_path", "");
    declare_parameter<std::string>("input_mode", "device");
    declare_parameter<int>("device_id", 0);
    declare_parameter<int>("capture_width", 3200);
    declare_parameter<int>("capture_height", 1200);
    declare_parameter<double>("capture_fps", 30.0);
    declare_parameter<std::string>("capture_fourcc", "MJPG");
    declare_parameter<int>("warmup_frames", 10);
    declare_parameter<std::string>("left_topic", "/camera/left/image_raw");
    declare_parameter<std::string>("right_topic", "/camera/right/image_raw");
    declare_parameter<int>("sync_queue_size", 10);
    declare_parameter<double>("max_sync_interval", 0.02);
    declare_parameter<bool>("use_viewer", true);
    declare_parameter<bool>("publish_tf", true);
    declare_parameter<bool>("publish_tracking_image", true);
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("camera_frame", "camera_optical_frame");
    declare_parameter<int>("max_path_length", 10000);
    declare_parameter<std::string>("trajectory_path", "live_session.txt");
    declare_parameter<std::string>("keyframe_trajectory_path", "kf_live_session.txt");
  }

  void read_parameters()
  {
    vocabulary_path_ = get_parameter("vocabulary_path").as_string();
    settings_path_ = get_parameter("settings_path").as_string();
    input_mode_ = get_parameter("input_mode").as_string();
    device_id_ = static_cast<int>(get_parameter("device_id").as_int());
    capture_width_ = static_cast<int>(get_parameter("capture_width").as_int());
    capture_height_ = static_cast<int>(get_parameter("capture_height").as_int());
    capture_fps_ = get_parameter("capture_fps").as_double();
    capture_fourcc_ = get_parameter("capture_fourcc").as_string();
    warmup_frames_ = static_cast<int>(get_parameter("warmup_frames").as_int());
    left_topic_ = get_parameter("left_topic").as_string();
    right_topic_ = get_parameter("right_topic").as_string();
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

  void start_device()
  {
    camera_.open(device_id_, cv::CAP_V4L2);
    if (!camera_.isOpened()) {
      throw std::runtime_error("failed to open /dev/video" + std::to_string(device_id_));
    }
    if (capture_fourcc_.size() != 4) {
      throw std::runtime_error("capture_fourcc must contain exactly four characters");
    }
    camera_.set(
      cv::CAP_PROP_FOURCC,
      cv::VideoWriter::fourcc(
        capture_fourcc_[0], capture_fourcc_[1], capture_fourcc_[2], capture_fourcc_[3]));
    camera_.set(cv::CAP_PROP_FRAME_WIDTH, capture_width_);
    camera_.set(cv::CAP_PROP_FRAME_HEIGHT, capture_height_);
    camera_.set(cv::CAP_PROP_FPS, capture_fps_);

    cv::Mat warmup;
    for (int i = 0; i < warmup_frames_; ++i) {
      camera_.read(warmup);
    }
    validate_side_by_side_frame(warmup);
    if (warmup.cols != capture_width_ || warmup.rows != capture_height_) {
      RCLCPP_WARN(
        get_logger(),
        "Camera returned %dx%d, but calibration/capture parameters expect %dx%d",
        warmup.cols, warmup.rows, capture_width_, capture_height_);
    }
    RCLCPP_INFO(
      get_logger(), "Opened /dev/video%d at %dx%d (per eye %dx%d)", device_id_,
      warmup.cols, warmup.rows, warmup.cols / 2, warmup.rows);

    capture_timer_ = create_wall_timer(
      std::chrono::milliseconds(1), std::bind(&StereoNode::capture_once, this));
  }

  void start_topic_subscriptions()
  {
    left_sub_.subscribe(this, left_topic_, rmw_qos_profile_sensor_data);
    right_sub_.subscribe(this, right_topic_, rmw_qos_profile_sensor_data);
    synchronizer_ = std::make_unique<message_filters::Synchronizer<StereoPolicy>>(
      StereoPolicy(sync_queue_size_), left_sub_, right_sub_);
    synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_sync_interval_));
    synchronizer_->registerCallback(
      std::bind(&StereoNode::stereo_callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(
      get_logger(), "Waiting for synchronized images on %s and %s",
      left_topic_.c_str(), right_topic_.c_str());
  }

  void capture_once()
  {
    if (stop_if_slam_shutdown()) {
      return;
    }
    cv::Mat full;
    if (!camera_.read(full)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Camera read failed; retrying");
      return;
    }
    try {
      validate_side_by_side_frame(full);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
      return;
    }

    const int eye_width = full.cols / 2;
    cv::Mat left = full(cv::Rect(0, 0, eye_width, full.rows)).clone();
    cv::Mat right = full(cv::Rect(eye_width, 0, eye_width, full.rows)).clone();
    const auto stamp = now();
    const double timestamp = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - steady_start_).count();
    process_stereo(left, right, timestamp, stamp);
  }

  static void validate_side_by_side_frame(const cv::Mat & frame)
  {
    if (frame.empty() || frame.cols < 2 || (frame.cols % 2) != 0) {
      throw std::runtime_error(
              "invalid side-by-side camera frame (width=" + std::to_string(frame.cols) +
              ", height=" + std::to_string(frame.rows) + ")");
    }
  }

  void stereo_callback(const Image::ConstSharedPtr & left_msg, const Image::ConstSharedPtr & right_msg)
  {
    if (stop_if_slam_shutdown()) {
      return;
    }
    try {
      const cv::Mat left = cv_bridge::toCvShare(left_msg, "bgr8")->image;
      const cv::Mat right = cv_bridge::toCvShare(right_msg, "bgr8")->image;
      const rclcpp::Time stamp(left_msg->header.stamp);
      double timestamp = stamp.seconds();
      if (timestamp <= last_input_timestamp_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping non-monotonic stereo pair (timestamp %.9f)", timestamp);
        return;
      }
      last_input_timestamp_ = timestamp;
      process_stereo(left, right, timestamp, stamp);
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Image conversion failed: %s", error.what());
    }
  }

  void process_stereo(
    const cv::Mat & left_input, const cv::Mat & right_input,
    double timestamp, const rclcpp::Time & stamp)
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

    // ORB-SLAM3 returns Tcw (world -> camera). ROS poses/TF need Twc.
    const Sophus::SE3f t_camera_world = slam_->TrackStereo(left, right, timestamp);
    if (stop_if_slam_shutdown()) {
      return;
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

    // Tracking::OK=2 and OK_KLT=5. Do not publish stale/undefined poses while lost.
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
    const cv::Mat & left, const std::vector<ORB_SLAM3::MapPoint *> & tracked_points,
    const rclcpp::Time & stamp)
  {
    cv::Mat visualization = left.clone();
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
      capture_timer_.reset();
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
    capture_timer_.reset();
    camera_.release();
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
  }

  std::string vocabulary_path_;
  std::string settings_path_;
  std::string input_mode_;
  int device_id_{0};
  int capture_width_{3200};
  int capture_height_{1200};
  double capture_fps_{30.0};
  std::string capture_fourcc_;
  int warmup_frames_{10};
  std::string left_topic_;
  std::string right_topic_;
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

  std::unique_ptr<ORB_SLAM3::System> slam_;
  float image_scale_{1.0F};
  bool shutdown_{false};
  bool shutdown_requested_{false};
  int last_tracking_state_{-1};
  double last_input_timestamp_{-1.0};
  std::size_t successful_pose_count_{0};
  const std::chrono::steady_clock::time_point steady_start_;

  cv::VideoCapture camera_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
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
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr localization_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr slam_service_;
};

}  // namespace orb_slam_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<orb_slam_ros2::StereoNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("orb_slam3_stereo"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
