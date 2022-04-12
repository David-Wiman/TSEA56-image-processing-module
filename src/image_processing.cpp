#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include "image_processing.h"
#include "help_funtions.h"

using namespace std;

ImageProcessing::ImageProcessing(bool visualize2, int lateral_position):visualize{visualize2}, lateral_position{lateral_position}, video_capture{cv::CAP_ANY} {
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
    cv::Mat out{};

    video_capture.grab();
    video_capture.retrieve(frame);
    int stop_distance;
    int angle{};

    int pre_lateral = lateral_position;  // XXX Undefined!
    int found_sidelines_success = image_process(frame, true, lateral_position, stop_distance);
    image_proc_t output;

    if (visualize) {
        cv::imshow("frame", out);
    }

    int lateral_diff = lateral_position - pre_lateral;
    if (found_sidelines_success != 1 || abs(lateral_diff) > 100) {
        cout << "No sidelines" << endl;
        output.success = false;
        return output;
    } else {
        // kalman
    }
    output.success = true;
    output.angle = angle;
    output.lateral_position = lateral_position;
    output.stop_distance = stop_distance;
    return output;
}
