// Verify translation invariance using ORB's actual visual and inertial edges.
#include "G2oTypes.h"
#include "CameraModels/Pinhole.h"
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: orb_translation_invariance ORB_SETTINGS.yaml\n";
    return 2;
  }
  cv::FileStorage settings(argv[1], cv::FileStorage::READ);
  cv::Mat tbc;
  settings["IMU.T_b_c1"] >> tbc;
  if (!settings.isOpened() || tbc.rows != 4 || tbc.cols != 4) return 2;
  tbc.convertTo(tbc, CV_64F);
  Eigen::Matrix3d rbc;
  Eigen::Vector3d pbc;
  for (int r = 0; r < 3; ++r) {
    pbc(r) = tbc.at<double>(r, 3);
    for (int c = 0; c < 3; ++c) rbc(r,c) = tbc.at<double>(r,c);
  }
  std::vector<float> intrinsics;
  for (const char* name : {"Camera1.fx", "Camera1.fy", "Camera1.cx", "Camera1.cy"})
    intrinsics.push_back(static_cast<float>(settings[name]));
  ORB_SLAM3::Pinhole camera(intrinsics);
  const double bf = static_cast<double>(settings["Stereo.b"]) * intrinsics[0];
  ORB_SLAM3::IMU::Calib calibration(
      Sophus::SE3f(rbc.cast<float>(), pbc.cast<float>()), .05f, .2f, .001f, .003f);
  ORB_SLAM3::IMU::Preintegrated preintegration(ORB_SLAM3::IMU::Bias(), calibration);
  for (int i = 0; i < 25; ++i)
    preintegration.IntegrateNewMeasurement(Eigen::Vector3f(.2f,-.1f,9.81f),
                                          Eigen::Vector3f(.1f,-.03f,.02f), .01f);

  double max_visual = 0, max_inertial = 0, max_body_translation = 0;
  for (int sample = 0; sample < 30; ++sample) {
    ORB_SLAM3::ImuCamPose pose[2];
    for (int frame = 0; frame < 2; ++frame) {
      const auto rotation = ORB_SLAM3::ExpSO3(.03*sample, -.07*frame, .11*sample);
      const Eigen::Vector3d translation(.02*sample, .08*frame, -.01*sample);
      pose[frame].SetParam({rotation}, {translation}, {rbc}, {pbc}, bf);
      pose[frame].pCamera = {&camera};
    }
    ORB_SLAM3::VertexPose first, second;
    first.setEstimate(pose[0]); second.setEstimate(pose[1]);
    ORB_SLAM3::VertexVelocity velocity1, velocity2;
    velocity1.setEstimate(Eigen::Vector3d(.2,-.1,.3));
    velocity2.setEstimate(Eigen::Vector3d(.3,.2,-.1));
    ORB_SLAM3::VertexGyroBias gyro;
    ORB_SLAM3::VertexAccBias accel;
    gyro.setEstimate(Eigen::Vector3d(.01,.02,-.01));
    accel.setEstimate(Eigen::Vector3d(.03,-.04,.02));
    ORB_SLAM3::EdgeInertial inertial(&preintegration);
    inertial.setVertex(0,&first); inertial.setVertex(1,&velocity1);
    inertial.setVertex(2,&gyro); inertial.setVertex(3,&accel);
    inertial.setVertex(4,&second); inertial.setVertex(5,&velocity2);
    inertial.computeError();
    const auto inertial_before = inertial.error().eval();

    g2o::VertexSBAPointXYZ point;
    const Eigen::Vector3d world_point = pose[0].Rcw[0].transpose() *
        (Eigen::Vector3d(.2,.1,2.5)-pose[0].tcw[0]);
    point.setEstimate(world_point);
    ORB_SLAM3::EdgeStereo visual;
    visual.setVertex(0,&point); visual.setVertex(1,&first);
    visual.setMeasurement(Eigen::Vector3d(320,240,310));
    visual.computeError();
    const auto visual_before = visual.error().eval();

    for (double magnitude : {0., .1, 1., 10., 100., 1000.}) {
      const Eigen::Vector3d shift(magnitude, -.7*magnitude, .3*magnitude);
      ORB_SLAM3::ImuCamPose shifted[2];
      for (int frame = 0; frame < 2; ++frame) {
        shifted[frame].SetParam(pose[frame].Rcw,
            {pose[frame].tcw[0]-pose[frame].Rcw[0]*shift}, {rbc}, {pbc}, bf);
        shifted[frame].pCamera = {&camera};
        max_body_translation = std::max(max_body_translation,
            (shifted[frame].twb-pose[frame].twb-shift).norm());
      }
      first.setEstimate(shifted[0]); second.setEstimate(shifted[1]);
      point.setEstimate(world_point+shift);
      visual.computeError(); inertial.computeError();
      max_visual = std::max(max_visual, (visual.error()-visual_before).norm());
      max_inertial = std::max(max_inertial, (inertial.error()-inertial_before).norm());
    }
  }
  std::cout << "edge_checks=180 max_visual_error_change_px=" << max_visual
            << " max_inertial_error_change=" << max_inertial
            << " max_body_translation_error_m=" << max_body_translation << '\n';
  return std::isfinite(max_visual) && std::isfinite(max_inertial) &&
      max_visual < 1e-8 && max_inertial < 1e-9 && max_body_translation < 1e-9 ? 0 : 1;
}
