// Link the original ORBextractor.cc with -DORB_SLAM3=ORBReference and the
// candidate ORBextractor.cc normally. This compares the complete front end.
#include "ORBextractor.h"
#undef ORBEXTRACTOR_H
#define ORB_SLAM3 ORBReference
#include "ORBextractor.h"
#undef ORB_SLAM3
#include <fstream>
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    cv::setNumThreads(1);
    ORBReference::ORBextractor reference(600, 1.2f, 8, 20, 7);
    ORB_SLAM3::ORBextractor candidate(600, 1.2f, 8, 20, 7);
    std::ifstream input(argv[1]);
    if (!input) return 2;
    std::string path;
    size_t images = 0, points = 0;
    while (std::getline(input, path)) {
        cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);
        if (image.empty()) return 2;
        // Exercise buffer size changes as well as native dataset images.
        if (images % 3 == 1) cv::resize(image, image, cv::Size(640, 480));
        std::vector<cv::KeyPoint> a, b;
        cv::Mat da, db;
        std::vector<int> overlap{0, 1000};
        int na = reference(image, cv::Mat(), a, da, overlap);
        int nb = candidate(image, cv::Mat(), b, db, overlap);
        if (na != nb || a.size() != b.size() || da.size() != db.size()) return 1;
        for (size_t i = 0; i < a.size(); ++i) {
            const auto &x = a[i], &y = b[i];
            if (x.pt != y.pt || x.size != y.size || x.angle != y.angle ||
                x.response != y.response || x.octave != y.octave || x.class_id != y.class_id)
                return 1;
        }
        if (!da.empty() && cv::countNonZero(da != db)) return 1;
        for (int level = 0; level < 8; ++level)
            if (cv::countNonZero(reference.mvImagePyramid[level] != candidate.mvImagePyramid[level]))
                return 1;
        points += a.size();
        ++images;
    }
    if (!images) return 2;
    std::cout << "exact_images=" << images << " exact_keypoints_and_descriptors=" << points << '\n';
}
