#pragma once
#include <opencv2/imgproc.hpp>
#include <cstdint>
#if defined(__AVX2__)
#include <immintrin.h>
#endif
namespace orb_fast {
// The OpenCV 4.5.4 Q8 coefficients are all even, so Q7 is exact.
// Each horizontal result is at most 255*128=32640, safe for signed
// 16-bit multiply-add. The vertical result fits a signed 32-bit integer.
// Reflect each image independently, as the original cloned pyramid does.
inline void gaussian7(const cv::Mat& src, cv::Mat& dst) {
    CV_Assert(src.type()==CV_8UC1 && src.data!=dst.data);
#if CV_VERSION_MAJOR != 4 || CV_VERSION_MINOR != 5 || CV_VERSION_REVISION != 4
    cv::GaussianBlur(src,dst,cv::Size(7,7),2,2,cv::BORDER_REFLECT_101|cv::BORDER_ISOLATED);
    return;
#endif
    if (src.cols<7 || src.rows<7) {
        cv::GaussianBlur(src,dst,cv::Size(7,7),2,2,cv::BORDER_REFLECT_101|cv::BORDER_ISOLATED);
        return;
    }
    cv::Mat horizontal;
    horizontal.create(src.size(),CV_16UC1);
    dst.create(src.size(),CV_8UC1);
    const int c[7]={9,17,24,28,24,17,9};
    for(int y=0;y<src.rows;++y) {
        const auto* row=src.ptr<uint8_t>(y); auto* out=horizontal.ptr<uint16_t>(y);
        int x=0;
        for(;x<3;++x) { unsigned s=0; for(int k=0;k<7;++k) s+=c[k]*row[cv::borderInterpolate(x+k-3,src.cols,cv::BORDER_REFLECT_101)]; out[x]=s; }
#if defined(__AVX2__)
        for(;x+16<=src.cols-3;x+=16) {
            auto center=_mm256_cvtepu8_epi16(_mm_loadu_si128(reinterpret_cast<const __m128i*>(row+x)));
            auto sum=_mm256_mullo_epi16(center,_mm256_set1_epi16(c[3]));
            for(int k=0;k<3;++k) {
                auto a=_mm256_cvtepu8_epi16(_mm_loadu_si128(reinterpret_cast<const __m128i*>(row+x+k-3)));
                auto b=_mm256_cvtepu8_epi16(_mm_loadu_si128(reinterpret_cast<const __m128i*>(row+x+3-k)));
                sum=_mm256_add_epi16(sum,_mm256_mullo_epi16(_mm256_add_epi16(a,b),_mm256_set1_epi16(c[k])));
            }
            _mm256_storeu_si256(reinterpret_cast<__m256i*>(out+x),sum);
        }
#endif
        for(;x<src.cols-3;++x) { unsigned s=0; for(int k=0;k<7;++k) s+=c[k]*row[x+k-3]; out[x]=s; }
        for(;x<src.cols;++x) { unsigned s=0; for(int k=0;k<7;++k) s+=c[k]*row[cv::borderInterpolate(x+k-3,src.cols,cv::BORDER_REFLECT_101)]; out[x]=s; }
    }
    for(int y=0;y<src.rows;++y) {
        const uint16_t* rows[7];
        for(int k=0;k<7;++k) rows[k]=horizontal.ptr<uint16_t>(cv::borderInterpolate(y+k-3,src.rows,cv::BORDER_REFLECT_101));
        auto* out=dst.ptr<uint8_t>(y); int x=0;
#if defined(__AVX2__)
        for(;x+16<=src.cols;x+=16) {
            auto center=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(rows[3]+x));
            auto zero=_mm256_setzero_si256(),weight=_mm256_set1_epi32(c[3]);
            // Unpack into the same four-lane order for every term.
            auto low=_mm256_add_epi32(_mm256_madd_epi16(_mm256_unpacklo_epi16(center,zero),weight),_mm256_set1_epi32(8192));
            auto high=_mm256_add_epi32(_mm256_madd_epi16(_mm256_unpackhi_epi16(center,zero),weight),_mm256_set1_epi32(8192));
            for(int k=0;k<3;++k) {
                auto a=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(rows[k]+x));
                auto b=_mm256_loadu_si256(reinterpret_cast<const __m256i*>(rows[6-k]+x));
                auto w=_mm256_set1_epi32(c[k]|(c[k]<<16));
                low=_mm256_add_epi32(low,_mm256_madd_epi16(_mm256_unpacklo_epi16(a,b),w));
                high=_mm256_add_epi32(high,_mm256_madd_epi16(_mm256_unpackhi_epi16(a,b),w));
            }
            low=_mm256_srli_epi32(low,14); high=_mm256_srli_epi32(high,14);
            auto packed=_mm256_packus_epi32(low,high);
            auto bytes=_mm_packus_epi16(_mm256_castsi256_si128(packed),_mm256_extracti128_si256(packed,1));
            _mm_storeu_si128(reinterpret_cast<__m128i*>(out+x),bytes);
        }
#endif
        for(;x<src.cols;++x) { unsigned s=8192;for(int k=0;k<7;++k)s+=c[k]*rows[k][x];out[x]=s>>14; }
    }
}
}
