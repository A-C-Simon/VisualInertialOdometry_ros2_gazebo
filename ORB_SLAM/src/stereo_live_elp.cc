/**
 * stereo_live_elp.cc - Live side-by-side stereo for ELP 3DGS1200P01
 * Input: /dev/video0, single MJPG frame 3200x1200 = left 1600x1200 | right 1600x1200
 * Splits and feeds ORB_SLAM3 System::STEREO with raw (distorted) images.
 * Calibration (PinHole + T_c1_c2) is handled inside ORB_SLAM3 via settings yaml.
 *
 * Usage: ./stereo_live_elp path_to_vocabulary path_to_settings [device_id] [trajectory_name]
 *   device_id default 0 (/dev/video0)
 */

#include <signal.h>
#include <iostream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <System.h>

using namespace std;

bool b_continue_session = true;
void exit_loop_handler(int) {
    cout << "Finishing session (Ctrl+C)..." << endl;
    b_continue_session = false;
}

int main(int argc, char **argv) {
    if (argc < 3 || argc > 5) {
        cerr << endl << "Usage: ./stereo_live_elp path_to_vocabulary path_to_settings [device_id] [trajectory_name]" << endl;
        cerr << "  e.g.: ./stereo_live_elp ../../Vocabulary/ORBvoc.txt ./ELP_1600x1200.yaml 0 live_traj" << endl;
        return 1;
    }

    int device_id = 0;
    string traj_name;
    if (argc >= 4) {
        // 4th arg may be device id or traj name; try parse as int
        try {
            size_t pos = 0;
            int v = stoi(string(argv[3]), &pos);
            if (pos == string(argv[3]).size()) {
                device_id = v;
                if (argc == 5) traj_name = string(argv[4]);
            } else {
                traj_name = string(argv[3]);
            }
        } catch (...) {
            traj_name = string(argv[3]);
        }
    }

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // --- Open side-by-side camera ---
    cv::VideoCapture cap(device_id, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        cerr << "Failed to open /dev/video" << device_id << endl;
        return 1;
    }
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 3200);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1200);
    // Try 30fps; driver may drop to ~12-15 at full res, fine for SLAM
    cap.set(cv::CAP_PROP_FPS, 30);

    double w = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double h = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    cout << "Camera /dev/video" << device_id << " opened at " << w << "x" << h << endl;
    if ((int)w != 3200 || (int)h != 1200) {
        cerr << "WARNING: expected 3200x1200 side-by-side. Got " << w << "x" << h
             << ". Make sure no other app holds the camera." << endl;
    }

    // Warm up + grab one frame to verify split
    cv::Mat full;
    for (int i = 0; i < 10; ++i) cap.read(full);
    if (full.empty() || full.cols < 2 || (full.cols % 2) != 0) {
        cerr << "Failed to grab valid side-by-side frame (cols=" << (full.empty() ? -1 : full.cols) << ")" << endl;
        return 1;
    }
    int W = full.cols / 2, H = full.rows;
    cout << "Split per-eye: " << W << "x" << H << " (expect 1600x1200)" << endl;

    // --- Create SLAM ---
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true, 0, traj_name);
    float imageScale = SLAM.GetImageScale();
    cout << "SLAM image scale: " << imageScale << endl;

    auto t0 = std::chrono::steady_clock::now();
    int nFrames = 0;

    cv::Mat imLeft, imRight;
    while (b_continue_session && !SLAM.isShutDown()) {
        if (!cap.read(full)) {
            cerr << "Camera read failed, retrying..." << endl;
            continue;
        }
        if (full.cols != W * 2) {
            // Handle resolution change mid-stream
            W = full.cols / 2; H = full.rows;
        }
        // Split: left | right (ELP order per INSTRUCTIONS.txt)
        imLeft = full(cv::Rect(0, 0, W, H));
        imRight = full(cv::Rect(W, 0, W, H));

        double tframe = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::steady_clock::now() - t0).count();

        cv::Mat imL = imLeft.clone();
        cv::Mat imR = imRight.clone();
        if (imageScale != 1.f) {
            int width = imL.cols * imageScale;
            int height = imL.rows * imageScale;
            cv::resize(imL, imL, cv::Size(width, height));
            cv::resize(imR, imR, cv::Size(width, height));
        }

        SLAM.TrackStereo(imL, imR, tframe);
        nFrames++;
    }

    cout << "Tracked " << nFrames << " frames. Shutting down..." << endl;
    SLAM.Shutdown();
    SLAM.SaveTrajectoryEuRoC(traj_name.empty() ? "CameraTrajectory.txt" : traj_name + ".txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC(traj_name.empty() ? "KeyFrameTrajectory.txt" : "kf_" + traj_name + ".txt");
    cap.release();
    cout << "Done." << endl;
    return 0;
}
