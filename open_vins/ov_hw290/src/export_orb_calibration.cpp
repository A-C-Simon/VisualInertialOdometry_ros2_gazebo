// Export the selected OpenVINS rectified camera/IMU calibration for ORB-SLAM3.
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <opencv2/core.hpp>

namespace fs = std::filesystem;

cv::FileStorage open_yaml(const fs::path &path) {
  cv::FileStorage file(path.string(), cv::FileStorage::READ);
  if (!file.isOpened()) throw std::runtime_error("Cannot open " + path.string());
  return file;
}

double number(const cv::FileNode &node) {
  if (!(node.isReal() || node.isInt())) throw std::runtime_error("Missing numeric calibration value");
  double value = static_cast<double>(node);
  if (!std::isfinite(value)) throw std::runtime_error("Nonfinite calibration value");
  return value;
}

cv::Mat transform(const cv::FileNode &camera) {
  auto node = camera["T_imu_cam"];
  bool invert = node.empty();
  if (invert) node = camera["T_cam_imu"];
  cv::Mat result(4, 4, CV_64F);
  if (node.isSeq() && node.size() == 4) {
    for (int row = 0; row < 4; ++row) {
      if (!node[row].isSeq() || node[row].size() != 4)
        throw std::runtime_error("Transform must have four rows and columns");
      for (int col = 0; col < 4; ++col) result.at<double>(row, col) = number(node[row][col]);
    }
  } else {
    node >> result;
    if (result.rows != 4 || result.cols != 4)
      throw std::runtime_error("Transform must be 4x4");
    result.convertTo(result, CV_64F);
  }
  const auto rotation = result(cv::Rect(0, 0, 3, 3));
  if (!cv::checkRange(result) ||
      cv::norm(rotation * rotation.t() - cv::Mat::eye(3, 3, CV_64F)) > 1e-5 ||
      std::abs(cv::determinant(rotation) - 1) > 1e-5 ||
      cv::norm(result.row(3) - (cv::Mat_<double>(1, 4) << 0, 0, 0, 1)) > 1e-8)
    throw std::runtime_error("Calibration transform is not a rigid SE3 transform");
  return invert ? result.inv() : result;
}

int main(int argc, char **argv) {
  if (argc != 5) {
    std::cerr << "Usage: export_orb_calibration ESTIMATOR_YAML STEREO_YAML OUTPUT_YAML FEATURES\n";
    return 2;
  }
  try {
    fs::path estimator_path = fs::absolute(argv[1]);
    auto estimator = open_yaml(estimator_path);
    std::string cam_file, imu_file;
    estimator["relative_config_imucam"] >> cam_file;
    estimator["relative_config_imu"] >> imu_file;
    if (cam_file.empty() || imu_file.empty()) throw std::runtime_error("Missing calibration file paths");
    auto chain = open_yaml(estimator_path.parent_path() / cam_file);
    auto noise = open_yaml(estimator_path.parent_path() / imu_file);
    auto stereo = open_yaml(argv[2]);
    auto left = chain["cam0"], right = chain["cam1"];
    auto imu = noise["imu0"];
    cv::Mat p1, p2;
    stereo["P1"] >> p1;
    stereo["P2"] >> p2;
    if (p1.rows != 3 || p1.cols != 4 || p2.rows != 3 || p2.cols != 4)
      throw std::runtime_error("Stereo profile needs rectified P1/P2 projection matrices");
    p1.convertTo(p1, CV_64F);
    p2.convertTo(p2, CV_64F);
    if (!cv::checkRange(p1) || !cv::checkRange(p2)) throw std::runtime_error("Nonfinite projections");
    if (p1.at<double>(0, 0) <= 0 || p1.at<double>(1, 1) <= 0)
      throw std::runtime_error("Rectified focal lengths must be positive");
    int width = static_cast<int>(stereo["image_width"]);
    int height = static_cast<int>(stereo["image_height"]);
    if (width <= 0 || height <= 0) throw std::runtime_error("Missing stereo image dimensions");
    const int rows[4] = {0, 1, 0, 1}, cols[4] = {0, 1, 2, 2};
    const std::string names[4] = {"fx", "fy", "cx", "cy"};
    for (int index = 0; index < 2; ++index) {
      auto camera = index == 0 ? left : right;
      auto intrinsic = camera["intrinsics"];
      if (intrinsic.size() != 4 || camera["resolution"].size() != 2 ||
          number(camera["resolution"][0]) != width || number(camera["resolution"][1]) != height)
        throw std::runtime_error("Camera chain and splitter dimensions disagree");
      for (int j = 0; j < 4; ++j) {
        if (std::abs(number(intrinsic[j]) - p1.at<double>(rows[j], cols[j])) > 1e-5 ||
            std::abs(p2.at<double>(rows[j], cols[j]) - p1.at<double>(rows[j], cols[j])) > 1e-5)
          throw std::runtime_error("Rectified intrinsics disagree across the selected profiles");
      }
      if (camera["distortion_coeffs"].size() != 4)
        throw std::runtime_error("Expected the rectified radtan chain");
      for (auto coefficient : camera["distortion_coeffs"])
        if (std::abs(number(coefficient)) > 1e-12)
          throw std::runtime_error("ORB export expects zero distortion after rectification");
    }
    double baseline = (p1.at<double>(0, 3) - p2.at<double>(0, 3)) / p1.at<double>(0, 0);
    if (!std::isfinite(baseline) || !(baseline > 0))
      throw std::runtime_error("Expected finite positive horizontal stereo baseline");
    auto tbc = transform(left);
    cv::Mat right_to_left = tbc.inv() * transform(right);
    cv::Mat expected = cv::Mat::eye(4, 4, CV_64F);
    expected.at<double>(0, 3) = baseline;
    if (cv::norm(right_to_left - expected) > 1e-5)
      throw std::runtime_error("Camera/IMU transforms do not match the rectified stereo baseline");
    std::size_t consumed = 0;
    int features = std::stoi(argv[4], &consumed);
    // Upstream stereo initialization requires more than 500 detected features.
    if (consumed != std::string(argv[4]).size() || features < 501 || features > 3000)
      throw std::runtime_error("Stereo-inertial features must be an integer in [501, 3000]");
    double offset = number(left["timeshift_cam_imu"]);
    for (const auto *key : {"gyroscope_noise_density", "accelerometer_noise_density",
                            "gyroscope_random_walk", "accelerometer_random_walk", "update_rate"})
      if (number(imu[key]) <= 0) throw std::runtime_error(std::string("IMU value must be positive: ") + key);
    // ORB uses dotted keys, which OpenCV's writer rejects although its reader
    // accepts them. Emit the flat YAML explicitly and check it with the reader.
    std::ofstream out(argv[3]);
    if (!out) throw std::runtime_error("Cannot write output profile");
    out << std::setprecision(17) << std::showpoint;
    out << "%YAML:1.0\n---\n# Generated from the selected OpenVINS camera/IMU and splitter profiles.\n";
    out << "File.version: \"1.0\"\nCamera.type: \"Rectified\"\n";
    auto value = [&](const std::string &key, auto scalar) { out << key << ": " << scalar << '\n'; };
    for (int j = 0; j < 4; ++j) value("Camera1." + names[j], p1.at<double>(rows[j], cols[j]));
    value("Camera.width", width); value("Camera.height", height);
    value("Camera.fps", 30); value("Camera.RGB", 1);
    value("Stereo.b", baseline); value("Stereo.ThDepth", 40.);
    out << "IMU.T_b_c1: !!opencv-matrix\n  rows: 4\n  cols: 4\n  dt: f\n  data: [";
    for (int row = 0; row < 4; ++row)
      for (int col = 0; col < 4; ++col)
        out << (row == 0 && col == 0 ? "" : ", ") << tbc.at<double>(row, col);
    out << "]\n";
    value("IMU.InsertKFsWhenLost", 0);
    value("IMU.NoiseGyro", number(imu["gyroscope_noise_density"]));
    value("IMU.NoiseAcc", number(imu["accelerometer_noise_density"]));
    value("IMU.GyroWalk", number(imu["gyroscope_random_walk"]));
    value("IMU.AccWalk", number(imu["accelerometer_random_walk"]));
    value("IMU.Frequency", number(imu["update_rate"]));
    value("HW290.CameraImuOffset", offset);
    value("ORBextractor.nFeatures", features); value("ORBextractor.scaleFactor", 1.2);
    value("ORBextractor.nLevels", 8); value("ORBextractor.iniThFAST", 20); value("ORBextractor.minThFAST", 7);
    value("Viewer.KeyFrameSize", .05); value("Viewer.KeyFrameLineWidth", 1.);
    value("Viewer.GraphLineWidth", .9); value("Viewer.PointSize", 2.);
    value("Viewer.CameraSize", .08); value("Viewer.CameraLineWidth", 3.);
    value("Viewer.ViewpointX", 0.); value("Viewer.ViewpointY", -.7); value("Viewer.ViewpointZ", -1.8);
    value("Viewer.ViewpointF", 500.); value("Viewer.imageViewScale", 1.);
    value("loopClosing", 0);
    out.close();
    if (!out) throw std::runtime_error("Writing output profile failed");
    auto check = open_yaml(argv[3]);
    std::cout << std::setprecision(17) << offset << '\n';
  } catch (const std::exception &error) {
    std::cerr << "ORB calibration export failed: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
