#pragma once
#include <climits>
#include <cstdint>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <vector>
#if defined(__AVX2__)
#include <immintrin.h>
#endif
namespace orb_fast {
// Exact Q5 bilinear sampling with constant-zero borders. Vector gathers read
// four bytes: the conservative interior excludes the last four columns so
// they stay within visible rows, including ROI inputs and the final row.
class Remap8u {
  cv::Mat xy_, fraction_;
  std::vector<int> offsets_;
  int width_, height_;
  std::size_t stride_;

public:
  Remap8u(const cv::Mat &mx, const cv::Mat &my, const cv::Mat &src) : width_(src.cols), height_(src.rows), stride_(src.step) {
    CV_Assert(!src.empty() && src.type() == CV_8UC1 && src.cols < SHRT_MAX && src.rows < SHRT_MAX && mx.cols < SHRT_MAX &&
              mx.rows < SHRT_MAX && src.step <= std::size_t(INT_MAX) / src.rows && mx.type() == CV_32FC1 && my.type() == CV_32FC1 &&
              !mx.empty() && mx.size() == my.size());
    cv::convertMaps(mx, my, xy_, fraction_, CV_16SC2);
    offsets_.resize(xy_.total());
    for (int y = 0; y < xy_.rows; ++y)
      for (int x = 0; x < xy_.cols; ++x) {
        auto p = xy_.at<cv::Vec2s>(y, x);
        bool valid = p[0] >= 0 && p[0] <= src.cols - 5 && p[1] >= 0 && p[1] < src.rows - 1;
        offsets_[std::size_t(y) * xy_.cols + x] = valid ? int(p[1] * src.step + p[0]) : -1;
      }
  }
  void apply(const cv::Mat &src, cv::Mat &dst) const {
    CV_Assert(src.type() == CV_8UC1 && src.cols == width_ && src.rows == height_ && src.step == stride_);
    dst.create(xy_.size(), CV_8UC1);
    auto sample = [&](int x, int y) { return x >= 0 && x < width_ && y >= 0 && y < height_ ? int(src.ptr<unsigned char>(y)[x]) : 0; };
    for (int y = 0; y < xy_.rows; ++y) {
      int x = 0;
      auto *out = dst.ptr<unsigned char>(y);
      auto *coord = xy_.ptr<cv::Vec2s>(y);
      auto *f = fraction_.ptr<unsigned short>(y);
      auto *idx = offsets_.data() + std::size_t(y) * xy_.cols;
#if defined(__AVX2__)
      const auto byte = _mm256_set1_epi32(255), mask = _mm256_set1_epi32(31), size = _mm256_set1_epi32(32), round = _mm256_set1_epi32(512);
      for (; x + 8 <= xy_.cols; x += 8) {
        auto indices = _mm256_loadu_si256(reinterpret_cast<const __m256i *>(idx + x));
        if (_mm256_movemask_epi8(_mm256_cmpgt_epi32(indices, _mm256_set1_epi32(-1))) != -1) {
          for (int k = 0; k < 8; ++k) {
            int a = f[x + k] & 31, b = f[x + k] >> 5, u = coord[x + k][0], v = coord[x + k][1];
            out[x + k] = ((sample(u, v) * (32 - a) + sample(u + 1, v) * a) * (32 - b) +
                          (sample(u, v + 1) * (32 - a) + sample(u + 1, v + 1) * a) * b + 512) >>
                         10;
          }
          continue;
        }
        auto fraction = _mm256_cvtepu16_epi32(_mm_loadu_si128(reinterpret_cast<const __m128i *>(f + x)));
        auto a = _mm256_and_si256(fraction, mask), b = _mm256_srli_epi32(fraction, 5);
        auto left = _mm256_sub_epi32(size, a), topweight = _mm256_sub_epi32(size, b);
        auto t = _mm256_i32gather_epi32(reinterpret_cast<const int *>(src.data), indices, 1);
        auto bottom =
            _mm256_i32gather_epi32(reinterpret_cast<const int *>(src.data), _mm256_add_epi32(indices, _mm256_set1_epi32(stride_)), 1);
        auto h0 = _mm256_add_epi32(_mm256_mullo_epi32(_mm256_and_si256(t, byte), left),
                                   _mm256_mullo_epi32(_mm256_and_si256(_mm256_srli_epi32(t, 8), byte), a));
        auto h1 = _mm256_add_epi32(_mm256_mullo_epi32(_mm256_and_si256(bottom, byte), left),
                                   _mm256_mullo_epi32(_mm256_and_si256(_mm256_srli_epi32(bottom, 8), byte), a));
        auto value =
            _mm256_srli_epi32(_mm256_add_epi32(_mm256_add_epi32(_mm256_mullo_epi32(h0, topweight), _mm256_mullo_epi32(h1, b)), round), 10);
        auto lo = _mm256_castsi256_si128(value), hi = _mm256_extracti128_si256(value, 1);
        auto words = _mm_packus_epi32(lo, hi), bytes = _mm_packus_epi16(words, words);
        _mm_storel_epi64(reinterpret_cast<__m128i *>(out + x), bytes);
      }
#endif
      for (; x < xy_.cols; ++x) {
        int a = f[x] & 31, b = f[x] >> 5, u = coord[x][0], v = coord[x][1];
        out[x] = ((sample(u, v) * (32 - a) + sample(u + 1, v) * a) * (32 - b) +
                  (sample(u, v + 1) * (32 - a) + sample(u + 1, v + 1) * a) * b + 512) >>
                 10;
      }
    }
  }
};

// Maps in System::Settings are immutable after construction. Retaining their
// owners makes pointer-based cache keys safe across destroyed/recreated
// Systems. This helper must not be used with maps modified in place.
struct RectificationPlan {
  cv::Mat mapx, mapy;
  cv::Size input_size;
  std::size_t input_stride;
  Remap8u remap;
  RectificationPlan(const cv::Mat &x, const cv::Mat &y, const cv::Mat &source)
      : mapx(x), mapy(y), input_size(source.size()), input_stride(source.step), remap(x, y, source) {}
  bool matches(const cv::Mat &x, const cv::Mat &y, const cv::Mat &source) const {
    return mapx.u == x.u && mapy.u == y.u && mapx.data == x.data && mapy.data == y.data && mapx.step == x.step && mapy.step == y.step &&
           mapx.size() == x.size() && mapy.size() == y.size() && input_size == source.size() && input_stride == source.step;
  }
};
inline bool overlaps(const cv::Mat &a, const cv::Mat &b) {
  if (a.empty() || b.empty())
    return false;
  const auto ab = std::uintptr_t(a.datastart), ae = std::uintptr_t(a.dataend);
  const auto bb = std::uintptr_t(b.datastart), be = std::uintptr_t(b.dataend);
  return ab < be && bb < ae;
}
inline void rectifyImmutableMaps(const cv::Mat &source, cv::Mat &output, const cv::Mat &mapx, const cv::Mat &mapy) {
#if defined(__AVX2__) && CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR == 5 && CV_VERSION_REVISION == 4
  if (cv::checkHardwareSupport(CV_CPU_AVX2) && !source.empty() && source.type() == CV_8UC1 && source.cols < SHRT_MAX &&
      source.rows < SHRT_MAX && mapx.cols < SHRT_MAX && mapx.rows < SHRT_MAX && source.step <= std::size_t(INT_MAX) / source.rows &&
      mapx.type() == CV_32FC1 && mapy.type() == CV_32FC1 && !mapx.empty() && mapx.size() == mapy.size() && mapx.u && mapy.u &&
      !overlaps(source, output) && !overlaps(mapx, output) && !overlaps(mapy, output)) {
    struct Cache {
      std::unique_ptr<RectificationPlan> plans[2];
      unsigned next = 0;
    };
    static thread_local Cache cache;
    for (const auto &plan : cache.plans) {
      if (plan && plan->matches(mapx, mapy, source)) {
        plan->remap.apply(source, output);
        return;
      }
    }
    auto &slot = cache.plans[cache.next++ % 2];
    slot.reset(new RectificationPlan(mapx, mapy, source));
    slot->remap.apply(source, output);
    return;
  }
#endif
  cv::remap(source, output, mapx, mapy, cv::INTER_LINEAR);
}
} // namespace orb_fast
