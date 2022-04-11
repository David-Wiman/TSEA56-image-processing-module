#include "image_processing.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

using namespace std;

ImageProcessing::ImageProcessing(bool visualize): video_capture{cv::CAP_ANY} {

    // Check if we succeeded to open video capture
    if (!video_capture.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return;
    }
}

ImageProcessing::~ImageProcessing() {
    video_capture.release();
    cv::destroyAllWindows();
}

image_proc_t ImageProcessing::process_next_frame() {

    // Get next frame
    cv::Mat frame{};
    video_capture.grab();
    video_capture.retrieve(frame);

    pre_mesument = lateral_mesument;  // XXX Undefined!
    int found_sidelines = image_process(frame, out, lateral_mesument, stop_distance);

    if (visualize)
        cv::imshow("frame", out);

    if (found_sidelines == 1) {
        mesument_diff = lateral_mesument - pre_mesument;
        if (is_down) {
            if (mesument_diff  > 100) {
                is_down = false;
            } else {
                lateral_mesument += mesument_diff;
            }
        } else if (is_up) {
            if (mesument_diff  < 100) {
                is_up = false;
            } else {
                lateral_mesument += mesument_diff;
            }
        } else if (mesument_diff < -100) {
            is_down = true;
        } else if(mesument_diff > 100) {
            is_up = true;
        } else {
            cout << "something wrong with prefilter" << endl;
        }
    } else {
        cout << "No sidelines" << endl;
    }
    if (cv::waitKey(10) > 0) break;
}
