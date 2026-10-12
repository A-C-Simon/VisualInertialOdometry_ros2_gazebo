#include "ORBextractor.h"
#include "orb_stereo_workers.hpp"
#include <algorithm>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <vector>
struct Extract {
  ORB_SLAM3::ORBextractor *extractor;
  const cv::Mat *image;
  std::vector<cv::KeyPoint> points;
  cv::Mat descriptors;
  std::vector<int> overlap{0, 0};
  int mono;
  static void call(void *p) {
    auto &x = *static_cast<Extract *>(p);
    x.mono = (*x.extractor)(*x.image, cv::Mat(), x.points, x.descriptors, x.overlap);
  }
};
double cpu() {
  timespec t;
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t);
  return t.tv_sec + t.tv_nsec * 1e-9;
}
void increment(void *p) { ++*static_cast<unsigned *>(p); }
void throws(void *) { throw std::runtime_error("job failure"); }
int main(int argc, char **argv) {
  if (argc != 2 && argc != 3)
    return 2;
  cv::setNumThreads(1);
  orb_fast::StereoWorkers pool;
  unsigned calls[2] = {0, 0};
  for (unsigned i = 0; i < 1000; ++i)
    pool.run({increment, &calls[0]}, {increment, &calls[1]});
  if (calls[0] != 1000 || calls[1] != 1000)
    return 1;
  bool propagated = false;
  try {
    pool.run({throws, nullptr}, {increment, &calls[1]});
  } catch (const std::runtime_error &) {
    propagated = true;
  }
  if (!propagated || calls[1] != 1001)
    return 1;
  pool.run({increment, &calls[0]}, {increment, &calls[1]});
  if (calls[0] != 1001 || calls[1] != 1002)
    return 1;
  unsigned caller_counts[4][2]{};
  std::thread callers[4];
  for (unsigned i = 0; i < 4; ++i)
    callers[i] = std::thread([&, i] {
      for (int j = 0; j < 100; ++j)
        pool.run({increment, &caller_counts[i][0]}, {increment, &caller_counts[i][1]});
    });
  for (auto &caller : callers)
    caller.join();
  for (auto &pair : caller_counts)
    if (pair[0] != 100 || pair[1] != 100)
      return 1;
  ORB_SLAM3::ORBextractor a(600, 1.6f, 5, 20, 7), b(600, 1.6f, 5, 20, 7), c(600, 1.6f, 5, 20, 7), d(600, 1.6f, 5, 20, 7);
  std::ifstream list(argv[1]);
  std::string path;
  std::vector<cv::Mat> images;
  while (std::getline(list, path)) {
    auto im = cv::imread(path, 0);
    if (im.empty())
      return 2;
    images.push_back(im);
  }
  if (images.empty())
    return 2;
  Extract refs[2] = {{&a, nullptr}, {&b, nullptr}}, candidates[2] = {{&c, nullptr}, {&d, nullptr}};
  size_t points = 0;
  for (size_t i = 0; i < images.size(); ++i) {
    for (int side = 0; side < 2; ++side) {
      refs[side].image = candidates[side].image = &images[(i + side) % images.size()];
      refs[side].overlap = candidates[side].overlap = i % 2 ? std::vector<int>{40, 700} : std::vector<int>{0, 0};
    }
    std::thread left(Extract::call, &refs[0]), right(Extract::call, &refs[1]);
    left.join();
    right.join();
    pool.run({Extract::call, &candidates[0]}, {Extract::call, &candidates[1]});
    for (int side = 0; side < 2; ++side) {
      auto &x = refs[side];
      auto &y = candidates[side];
      if (x.mono != y.mono || x.points.size() != y.points.size() || cv::norm(x.descriptors, y.descriptors, cv::NORM_INF) != 0)
        return 1;
      for (size_t k = 0; k < x.points.size(); ++k) {
        auto &p = x.points[k];
        auto &q = y.points[k];
        if (p.pt != q.pt || p.size != q.size || p.angle != q.angle || p.response != q.response || p.octave != q.octave ||
            p.class_id != q.class_id)
          return 1;
      }
      for (int level = 0; level < 5; ++level)
        if (cv::norm(x.extractor->mvImagePyramid[level], y.extractor->mvImagePyramid[level], cv::NORM_INF) != 0)
          return 1;
      points += x.points.size();
    }
  }
  if (argc == 3) {
    if (std::string(argv[2]) != "--check-only")
      return 2;
    std::cout << "exact_stereo_pairs=" << images.size() << " exact_keypoints_descriptors=" << points
              << " synchronization_exception_checks=passed\n";
    return 0;
  }
  auto run = [&](bool persistent) {
    double start = cpu();
    for (int k = 0; k < 500; ++k) {
      for (int side = 0; side < 2; ++side)
        candidates[side].image = &images[(k + side) % images.size()];
      if (persistent)
        pool.run({Extract::call, &candidates[0]}, {Extract::call, &candidates[1]});
      else {
        std::thread left(Extract::call, &candidates[0]), right(Extract::call, &candidates[1]);
        left.join();
        right.join();
      }
    }
    return cpu() - start;
  };
  std::vector<double> reference, candidate;
  for (int r = 0; r < 3; ++r) {
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
  std::cout << std::setprecision(17) << "exact_stereo_pairs=" << images.size() << " exact_keypoints_descriptors=" << points
            << " reference_cpu_s=" << reference[1] << " candidate_cpu_s=" << candidate[1]
            << " cpu_percent_decrease=" << 100 * (reference[1] - candidate[1]) / reference[1] << "\n";
}
