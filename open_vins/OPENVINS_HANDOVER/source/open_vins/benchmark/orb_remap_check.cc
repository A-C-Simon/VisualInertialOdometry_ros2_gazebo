#include "orb_remap8u.hpp"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <thread>
void compare_api(const cv::Mat &input, const cv::Mat &mx, const cv::Mat &my) {
  cv::Mat ref, actual;
  cv::remap(input, ref, mx, my, cv::INTER_LINEAR);
  orb_fast::rectifyImmutableMaps(input, actual, mx, my);
  if (cv::norm(ref, actual, cv::NORM_INF) != 0)
    throw std::runtime_error("API pixels differ");
}
int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "usage: remap_check SETTINGS IMAGE_LIST\n";
    return 2;
  }
  cv::setNumThreads(1);
  cv::FileStorage fs(argv[1], cv::FileStorage::READ);
  cv::Mat K[2], D[2], T;
  for (int i = 0; i < 2; ++i) {
    std::string prefix = "Camera" + std::to_string(i + 1) + ".";
    K[i] = (cv::Mat_<double>(3, 3) << double(fs[prefix + "fx"]), 0, double(fs[prefix + "cx"]), 0, double(fs[prefix + "fy"]),
            double(fs[prefix + "cy"]), 0, 0, 1);
    D[i] = (cv::Mat_<double>(1, 4) << double(fs[prefix + "k1"]), double(fs[prefix + "k2"]), double(fs[prefix + "p1"]),
            double(fs[prefix + "p2"]));
  }
  fs["Stereo.T_c1_c2"] >> T;
  T.convertTo(T, CV_64F);
  T = T.inv();
  cv::Mat R1, R2, P1, P2, Q;
  cv::Size size(752, 480);
  cv::stereoRectify(K[0], D[0], K[1], D[1], size, T(cv::Rect(0, 0, 3, 3)), T(cv::Rect(3, 0, 1, 3)), R1, R2, P1, P2, Q,
                    cv::CALIB_ZERO_DISPARITY, -1, size);
  cv::Mat mx[2], my[2], fx[2], fy[2];
  for (int i = 0; i < 2; ++i) {
    cv::initUndistortRectifyMap(K[i], D[i], i ? R2 : R1, (i ? P2 : P1)(cv::Rect(0, 0, 3, 3)), size, CV_32F, mx[i], my[i]);
    cv::convertMaps(mx[i], my[i], fx[i], fy[i], CV_16SC2);
  }
  cv::Mat sample(480, 752, CV_8U);
  orb_fast::Remap8u plans[2] = {orb_fast::Remap8u(mx[0], my[0], sample), orb_fast::Remap8u(mx[1], my[1], sample)};
  cv::RNG rng(315897);
  size_t random_pixels = 0;
  for (int k = 0; k < 1000; ++k) {
    int w = rng.uniform(1, 220), h = rng.uniform(1, 150), ow = rng.uniform(1, 225), oh = rng.uniform(1, 155);
    cv::Mat storage(h + 5, w + 13, CV_8U);
    rng.fill(storage, cv::RNG::UNIFORM, 0, 256);
    cv::Mat input = storage(cv::Rect(3, 2, w, h)), mapx(oh, ow, CV_32F), mapy(oh, ow, CV_32F);
    rng.fill(mapx, cv::RNG::UNIFORM, -10, w + 10);
    rng.fill(mapy, cv::RNG::UNIFORM, -10, h + 10);
    if (k % 5 == 0) {
      for (int y = 0; y < oh; ++y)
        for (int x = 0; x < ow; ++x) {
          mapx.at<float>(y, x) = float(x);
          mapy.at<float>(y, x) = float(y);
        }
    }
    cv::Mat ref, actual;
    cv::remap(input, ref, mapx, mapy, cv::INTER_LINEAR);
    orb_fast::Remap8u plan(mapx, mapy, input);
    plan.apply(input, actual);
    if (cv::countNonZero(ref != actual)) {
      std::cerr << "Random mismatch " << k << "\n";
      return 1;
    }
    random_pixels += ref.total();
    compare_api(input, mapx, mapy);
    // Fallback for color, aliasing and external map storage.
    if (k % 100 == 0) {
      cv::Mat rgb;
      cv::cvtColor(input, rgb, cv::COLOR_GRAY2BGR);
      compare_api(rgb, mapx, mapy);
      cv::Mat external(oh, ow, CV_32F, mapx.data, mapx.step);
      compare_api(input, external, mapy);
      cv::Mat alias = input.clone(), alias_ref = alias.clone();
      cv::remap(alias_ref, alias_ref, mapx, mapy, cv::INTER_LINEAR);
      orb_fast::rectifyImmutableMaps(alias, alias, mapx, mapy);
      if (cv::norm(alias, alias_ref, cv::NORM_INF) != 0)
        throw std::runtime_error("Alias fallback differs");
      cv::Mat fixed1, fixed2;
      cv::convertMaps(mapx, mapy, fixed1, fixed2, CV_16SC2);
      compare_api(input, fixed1, fixed2);
    }
  }
  std::ifstream list(argv[2]);
  std::string path;
  size_t images = 0, pixels = 0;
  cv::Mat src, a, b;
  while (std::getline(list, path)) {
    src = cv::imread(path, 0);
    for (int i = 0; i < 2; ++i) {
      cv::remap(src, a, mx[i], my[i], cv::INTER_LINEAR);
      orb_fast::rectifyImmutableMaps(src, b, mx[i], my[i]);
      if (cv::countNonZero(a != b)) {
        std::cerr << "Dataset mismatch\n";
        return 1;
      }
      ++images;
      pixels += a.total();
    }
  }
  // Exercise independent per-thread caches and repeated map replacement.
  std::thread workers[2];
  for (auto &worker : workers)
    worker = std::thread([&] {
      for (int j = 0; j < 20; ++j)
        compare_api(src, mx[j % 2], my[j % 2]);
    });
  for (auto &worker : workers)
    worker.join();
  if (images == 0) {
    std::cerr << "No dataset images checked\n";
    return 3;
  }
  auto run = [&](bool native) {
    auto t = std::chrono::steady_clock::now();
    for (int k = 0; k < 400; ++k)
      for (int i = 0; i < 2; ++i) {
        if (native)
          orb_fast::rectifyImmutableMaps(src, a, mx[i], my[i]);
        else
          cv::remap(src, a, mx[i], my[i], cv::INTER_LINEAR);
      }
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
  };
  std::vector<double> reference, candidate;
  for (int r = 0; r < 5; ++r) {
    if (r % 2) {
      candidate.push_back(run(true));
      reference.push_back(run(false));
    } else {
      reference.push_back(run(false));
      candidate.push_back(run(true));
    }
  }
  std::sort(reference.begin(), reference.end());
  std::sort(candidate.begin(), candidate.end());
  std::cout << "exact_rectified_images=" << images << " exact_pixels=" << pixels << " random_pixels=" << random_pixels
            << " float_seconds=" << reference[2] << " native_seconds=" << candidate[2]
            << " kernel_percent_decrease=" << 100 * (reference[2] - candidate[2]) / reference[2] << "\n";
}
