#include "Frame.h"
#include "ORBextractor.h"
#include "CameraModels/Pinhole.h"
#include <opencv2/core.hpp>
#include <cstddef>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <new>

int main(int argc, char **argv) {
    const float poison = argc > 1 ? std::strtof(argv[1], nullptr) : 0.001f;
    cv::setNumThreads(1);
    cv::Mat left(480, 640, CV_8UC1), right(480, 640, CV_8UC1);
    cv::RNG rng(0x715abc);
    rng.fill(left, cv::RNG::UNIFORM, 0, 256);
    rng.fill(right, cv::RNG::UNIFORM, 0, 256);
    left(cv::Rect(30, 0, 610, 480)).copyTo(right(cv::Rect(0, 0, 610, 480)));
    cv::Mat K = (cv::Mat_<float>(3,3) << 400,0,320,0,400,240,0,0,1);
    cv::Mat distortion = cv::Mat::zeros(4,1,CV_32F);
    ORB_SLAM3::ORBextractor extract_left(600,1.2f,8,20,7), extract_right(600,1.2f,8,20,7);
    ORB_SLAM3::Pinhole camera(std::vector<float>{400,400,320,240});
    // Raw storage is legal to seed before construction. A correct constructor
    // must overwrite the baseline before it is read by stereo matching.
    void *storage = nullptr;
    if (posix_memalign(&storage, alignof(ORB_SLAM3::Frame), sizeof(ORB_SLAM3::Frame))) return 2;
    std::memset(storage, 0, sizeof(ORB_SLAM3::Frame));
    std::memcpy(static_cast<char *>(storage) + offsetof(ORB_SLAM3::Frame, mb),
                &poison, sizeof(poison));
    auto *frame = new(storage) ORB_SLAM3::Frame(left,right,0.,&extract_left,&extract_right,
                          nullptr,K,distortion,24.f,2.4f,&camera);
    int valid = 0;
    double sum_depth = 0;
    for (float depth : frame->mvDepth) {
        if (depth > 0) { ++valid; sum_depth += depth; }
    }
    std::cout << "storage_baseline=" << poison << " initialized_baseline=" << frame->mb
              << " features=" << frame->N << " valid_depths=" << valid
              << " depth_sum=" << sum_depth << '\n';
    frame->~Frame();
    std::free(storage);
    return valid > 0 ? 0 : 1;
}
