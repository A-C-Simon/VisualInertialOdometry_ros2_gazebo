// Convert a raw Kalibr camera/IMU fit into matching splitter and VIO profiles.
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;

std::string read(const fs::path &path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot read " + path.string());
  std::ostringstream text;
  text << input.rdbuf();
  return text.str();
}

YAML::Node yaml(std::string text) {
  if (text.rfind("%YAML", 0) == 0) text.erase(0, text.find('\n') + 1);
  return YAML::Load(text);
}

double scalar(const YAML::Node &node) {
  if (!node.IsScalar()) throw std::runtime_error("Missing numeric value");
  const double value = node.as<double>();
  if (!std::isfinite(value)) throw std::runtime_error("Nonfinite value");
  return value;
}

cv::Mat sequence(const YAML::Node &node, int rows, int cols) {
  if (!node.IsSequence() || static_cast<int>(node.size()) != rows)
    throw std::runtime_error("Invalid matrix dimensions");
  cv::Mat matrix(rows, cols, CV_64F);
  for (int row = 0; row < rows; ++row) {
    if (!node[row].IsSequence() || static_cast<int>(node[row].size()) != cols)
      throw std::runtime_error("Invalid matrix row");
    for (int col = 0; col < cols; ++col) matrix.at<double>(row, col) = scalar(node[row][col]);
  }
  return matrix;
}

cv::Mat rigid(const YAML::Node &node) {
  cv::Mat matrix = sequence(node, 4, 4), rotation = matrix(cv::Rect(0, 0, 3, 3));
  if (cv::norm(rotation * rotation.t() - cv::Mat::eye(3, 3, CV_64F)) > 1e-5 ||
      std::abs(cv::determinant(rotation) - 1) > 1e-5 ||
      cv::norm(matrix.row(3) - (cv::Mat_<double>(1, 4) << 0, 0, 0, 1)) > 1e-8)
    throw std::runtime_error("Invalid rigid transform");
  return matrix;
}

int main(int argc, char **argv) {
  if (argc != 5) {
    std::cerr << "Usage: build_rectified_profile RAW_CHAIN IMU_MODEL ESTIMATOR_TEMPLATE OUTPUT_DIR\n";
    return 2;
  }
  try {
    const std::string raw = read(argv[1]), noise = read(argv[2]);
    std::string estimator = read(argv[3]);
    auto chain = yaml(raw), imu = yaml(noise);
    if (!imu["imu0"]) throw std::runtime_error("Expected an OpenVINS imu0 model");
    cv::Mat k[2], d[2], body[2];
    double offset[2];
    cv::Size size;
    for (int i = 0; i < 2; ++i) {
      auto camera = chain["cam" + std::to_string(i)];
      if (camera["camera_model"].as<std::string>() != "pinhole" ||
          camera["distortion_model"].as<std::string>() != "radtan" ||
          camera["intrinsics"].size() != 4 || camera["distortion_coeffs"].size() != 4 ||
          camera["resolution"].size() != 2)
        throw std::runtime_error("Expected two pinhole/radtan cameras");
      const double width = scalar(camera["resolution"][0]), height = scalar(camera["resolution"][1]);
      if (width != 640 || height != 480)
        throw std::runtime_error("HW290 splitter requires 640x480 per camera");
      size = cv::Size(static_cast<int>(width), static_cast<int>(height));
      k[i] = cv::Mat::eye(3, 3, CV_64F);
      k[i].at<double>(0, 0) = scalar(camera["intrinsics"][0]);
      k[i].at<double>(1, 1) = scalar(camera["intrinsics"][1]);
      k[i].at<double>(0, 2) = scalar(camera["intrinsics"][2]);
      k[i].at<double>(1, 2) = scalar(camera["intrinsics"][3]);
      if (k[i].at<double>(0, 0) <= 0 || k[i].at<double>(1, 1) <= 0)
        throw std::runtime_error("Nonpositive focal length");
      d[i] = cv::Mat(1, 4, CV_64F);
      for (int j = 0; j < 4; ++j) d[i].at<double>(0, j) = scalar(camera["distortion_coeffs"][j]);
      body[i] = rigid(camera["T_cam_imu"]);
      offset[i] = scalar(camera["timeshift_cam_imu"]);
    }
    cv::Mat stereo = rigid(chain["cam1"]["T_cn_cnm1"]);
    if (cv::norm(body[1] * body[0].inv() - stereo) > 1e-5)
      throw std::runtime_error("Stereo and camera/IMU transforms disagree");
    cv::Mat rotation = stereo(cv::Rect(0, 0, 3, 3)), translation = stereo(cv::Rect(3, 0, 1, 3));
    cv::Mat rect[2], projection[2], q;
    cv::stereoRectify(k[0], d[0], k[1], d[1], size, rotation, translation,
                     rect[0], rect[1], projection[0], projection[1], q, cv::CALIB_ZERO_DISPARITY, 0);
    double baseline = -projection[1].at<double>(0, 3) / projection[1].at<double>(0, 0);
    if (!(baseline > 0) || std::abs(projection[1].at<double>(1, 3)) > 1e-8)
      throw std::runtime_error("Expected positive horizontal stereo baseline");
    for (const std::string key : {"relative_config_imucam", "relative_config_imu"}) {
      std::istringstream lines(estimator);
      std::ostringstream output;
      std::string line;
      int matches = 0;
      while (std::getline(lines, line)) {
        if (line.rfind(key + ":", 0) == 0) {
          line = key + ": \"" + (key == "relative_config_imu" ? "imu.yaml" : "camchain.yaml") + "\"";
          ++matches;
        }
        output << line << '\n';
      }
      if (matches != 1) throw std::runtime_error("Expected one estimator path: " + key);
      estimator = output.str();
    }
    const fs::path output(argv[4]);
    if (!fs::create_directory(output)) throw std::runtime_error("Output directory must not already exist");
    auto write = [&](const std::string &name, const std::string &text) {
      std::ofstream file(output / name);
      file << text;
      file.close();
      if (!file) throw std::runtime_error("Cannot write " + name);
    };
    write("camchain_raw.yaml", raw);
    write("imu.yaml", noise);
    write("estimator_config.yaml", estimator);
    cv::FileStorage split((output / "stereo_opencv.yaml").string(), cv::FileStorage::WRITE);
    split << "image_width" << size.width << "image_height" << size.height;
    split << "K1" << k[0] << "D1" << d[0] << "K2" << k[1] << "D2" << d[1];
    split << "R" << rotation << "T" << translation << "R1" << rect[0] << "R2" << rect[1];
    split << "P1" << projection[0] << "P2" << projection[1] << "Q" << q;
    split.release();
    std::ostringstream result;
    result << std::setprecision(17) << "%YAML:1.0\n---\n";
    for (int i = 0; i < 2; ++i) {
      cv::Mat rectification = cv::Mat::eye(4, 4, CV_64F);
      rect[i].copyTo(rectification(cv::Rect(0, 0, 3, 3)));
      cv::Mat transform = (rectification * body[i]).inv();
      result << "cam" << i << ":\n  cam_overlaps: [" << (1 - i) << "]\n"
             << "  camera_model: pinhole\n  distortion_model: radtan\n"
             << "  distortion_coeffs: [0., 0., 0., 0.]\n  intrinsics: ["
             << projection[i].at<double>(0, 0) << ", " << projection[i].at<double>(1, 1) << ", "
             << projection[i].at<double>(0, 2) << ", " << projection[i].at<double>(1, 2) << "]\n"
             << "  resolution: [640, 480]\n  rostopic: /cam" << i << "/image_raw\n"
             << "  timeshift_cam_imu: " << offset[i] << "\n  T_imu_cam:\n";
      for (int row = 0; row < 4; ++row) {
        result << "    - [";
        for (int col = 0; col < 4; ++col)
          result << (col ? ", " : "") << transform.at<double>(row, col);
        result << "]\n";
      }
    }
    write("camchain.yaml", result.str());
    std::cout << std::setprecision(17) << "baseline_m=" << baseline << " cam0_offset_s=" << offset[0] << '\n';
  } catch (const std::exception &error) {
    std::cerr << "Rectified profile conversion failed: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
