#include "image_processing.h"
#include "help_funtions.h"

ImageProcessing::ImageProcessing(const bool vl) :visualize{vl}, video_capture{cv::CAP_ANY} {
    // Check if we succeeded to open video capture
    if (!video_capture.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return;
    }
    // video_capture.set(3, 180); // set frame size
    // video_capture.set(4, 100); // set frame size
    // video_capture.set(cv::CAP_PROP_AUTOFOCUS, 0); // turn the autofocus off

    transformation_matrix = perspective_transform_init();
}

ImageProcessing::~ImageProcessing() {
    video_capture.release();
    cv::destroyAllWindows();
}

image_proc_t ImageProcessing::process_next_frame() {
    int stop_distance = 0;
    float road_angle = 0;
    float lateral_position = 0;
    const int R = 10;  // Mesunet noise
    const float Q = 10;  // Process noise


    // Get next frame
    video_capture.grab();
    video_capture.retrieve(frame);

    // int pre_lateral = lateral_position;
    // perspective_transform(frame, transformation_matrix);
    int found_sidelines_success = image_process(frame, true, lateral_position, road_angle, stop_distance);
    if (visualize) {
        cv::imshow("frame", frame);
    }

    float lateral_diff = lateral_position - lateral_model;
    if (found_sidelines_success != 1 || abs(lateral_diff) > 100) {
        std::cout << "No sidelines" << std::endl;
        output.success = false;
        return output;
    } else {

        kalman(P, lateral_model, lateral_position, R);
        P = P + Q;
        std::cout << "lat:" << lateral_model <<", angle:"<<road_angle << ", stop:" << stop_distance << std::endl;

    }
    output.success = true;
    output.road_angle = road_angle;
    output.lateral_position = lateral_model;
    output.stop_distance = stop_distance;

    return output;
}
