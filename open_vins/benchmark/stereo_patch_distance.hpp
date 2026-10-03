#pragma once

#include <opencv2/core.hpp>
#include <cstdlib>

namespace ORB_SLAM3 {
namespace detail {

// Exact L1 distance for the existing 11 x 11 grayscale stereo patch.
// The largest possible sum is 30,855, exactly representable as int and float.
inline int stereo_patch_l1(const cv::Mat &left, const cv::Mat &right,
                           int right_x, int right_y) {
    CV_DbgAssert(left.rows == 11 && left.cols == 11 && left.type() == CV_8UC1);
    CV_DbgAssert(right.type() == CV_8UC1 && right_x >= 0 && right_y >= 0);
    CV_DbgAssert(right_x + 11 <= right.cols && right_y + 11 <= right.rows);
    int total = 0;
    for (int row = 0; row < 11; ++row) {
        const unsigned char *l = left.ptr<unsigned char>(row);
        const unsigned char *r = right.ptr<unsigned char>(right_y + row) + right_x;
        for (int col = 0; col < 11; ++col)
            total += std::abs(int(l[col]) - int(r[col]));
    }
    return total;
}

}  // namespace detail
}  // namespace ORB_SLAM3
