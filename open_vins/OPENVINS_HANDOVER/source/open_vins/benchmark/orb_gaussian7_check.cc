#include "orb_gaussian7.hpp"
#include <chrono>
#include <iostream>

int main() {
    cv::setNumThreads(1);
    cv::RNG rng(827512);
    long long pixels = 0;
    for (int i = 0; i < 600; ++i) {
        int w = 1 + rng.uniform(0, 800), h = 1 + rng.uniform(0, 520);
        cv::Mat storage(h + 8, w + 11, CV_8U);
        rng.fill(storage, cv::RNG::UNIFORM, 0, 256);
        cv::Mat src = storage(cv::Rect(3, 4, w, h));
        if (i % 5 == 0) src.setTo((i % 2) ? 255 : 0);
        if (i % 5 == 1)
            for (int y = 0; y < h; ++y)
                for (int x = 0; x < w; ++x)
                    src.at<uchar>(y, x) = ((x + y) % 2) * 255;
        cv::Mat original = src.clone(), ref = src.clone(), out;
        cv::GaussianBlur(ref, ref, cv::Size(7, 7), 2, 2, cv::BORDER_REFLECT_101);
        orb_fast::gaussian7(src, out);
        if (cv::countNonZero(ref != out) || cv::countNonZero(src != original)) {
            std::cerr << "Mismatch at " << w << "x" << h << '\n';
            return 1;
        }
        pixels += static_cast<long long>(w) * h;
    }
    cv::Mat src(480, 752, CV_8U), ref, out;
    rng.fill(src, cv::RNG::UNIFORM, 0, 256);
    auto run = [&](bool fast) {
        auto start = std::chrono::steady_clock::now();
        for (int i = 0; i < 400; ++i) {
            if (fast) orb_fast::gaussian7(src, out);
            else {
                ref = src.clone();
                cv::GaussianBlur(ref, ref, cv::Size(7, 7), 2, 2, cv::BORDER_REFLECT_101);
            }
        }
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    };
    std::cout << "opencv=" << CV_VERSION << " images=600 exact_pixels=" << pixels
              << " opencv_s=" << run(false) << " fixed_s=" << run(true) << '\n';
}
