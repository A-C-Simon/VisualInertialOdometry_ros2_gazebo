#include "stereo_patch_distance.hpp"

#include <array>
#include <chrono>
#include <iostream>

using ORB_SLAM3::detail::stereo_patch_l1;

int main() {
    // Odd row strides and ROI views exercise the storage used by stereo patches.
    cv::Mat left_image(483, 757, CV_8UC1), right_image(487, 773, CV_8UC1);
    cv::RNG rng(0x513abc);
    rng.fill(left_image, cv::RNG::UNIFORM, 0, 256);
    rng.fill(right_image, cv::RNG::UNIFORM, 0, 256);
    int checks = 0;
    for (int k = 0; k < 30000; ++k) {
        const int x = rng.uniform(10, 737), y = rng.uniform(0, 473);
        const cv::Mat left = left_image(cv::Rect(x, y, 11, 11));
        std::array<float, 11> reference{}, candidate{};
        for (int offset = -5; offset <= 5; ++offset) {
            const cv::Mat right = right_image(cv::Rect(x + offset, y, 11, 11));
            reference[offset + 5] = float(cv::norm(left, right, cv::NORM_L1));
            candidate[offset + 5] = float(stereo_patch_l1(left, right_image, x + offset, y));
            if (reference[offset + 5] != candidate[offset + 5]) {
                std::cerr << "Patch distance mismatch at trial " << k << '\n';
                return 1;
            }
            ++checks;
        }
        if (reference != candidate) return 2;
    }
    // Exercise the exact minimum and maximum sums separately from random input.
    for (int l : {0, 255}) {
        for (int r : {0, 255}) {
            cv::Mat left(11, 11, CV_8UC1, cv::Scalar(l));
            cv::Mat right(13, 17, CV_8UC1, cv::Scalar(r));
            const int expected = 121 * std::abs(l - r);
            const float reference = float(cv::norm(left, right(cv::Rect(2, 1, 11, 11)), cv::NORM_L1));
            if (stereo_patch_l1(left, right, 2, 1) != expected || reference != expected) return 3;
            ++checks;
        }
    }
    volatile long long checksum = 0;
    const auto measure = [&](bool fast) {
        const auto start = std::chrono::steady_clock::now();
        for (int k = 0; k < 200000; ++k) {
            const int x = 10 + k % 700, y = k % 460;
            const cv::Mat left = left_image(cv::Rect(x, y, 11, 11));
            for (int offset = -5; offset <= 5; ++offset) {
                if (fast) {
                    checksum += stereo_patch_l1(left, right_image, x + offset, y);
                } else {
                    const cv::Mat right = right_image(cv::Rect(x + offset, y, 11, 11));
                    checksum += int(cv::norm(left, right, cv::NORM_L1));
                }
            }
        }
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    };
    const double reference_seconds = measure(false), candidate_seconds = measure(true);
    std::cout << "equal_patch_distances=" << checks
              << " baseline_seconds=" << reference_seconds
              << " pointer_seconds=" << candidate_seconds
              << " ratio=" << candidate_seconds / reference_seconds
              << " checksum=" << checksum << '\n';
}
