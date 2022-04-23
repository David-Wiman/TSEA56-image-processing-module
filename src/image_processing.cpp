#include "image_processing.h"
#include "help_funtions.h"

ImageProcessing::ImageProcessing(const bool vl) :visualize{vl}, video_capture{cv::CAP_ANY} {
    // Check if we succeeded to open video capture
    if (!video_capture.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return;
    }
    video_capture.grab();
    video_capture.retrieve(frame);

    // video_capture.set(3, 180); // set frame size
    // video_capture.set(4, 100); // set frame size
    // video_capture.set(cv::CAP_PROP_AUTOFOCUS, 0); // turn the autofocus off

    // mapx = get_transform_mtx("mapx.txt", frame.size().width, frame.size().height);
    // mapy = get_transform_mtx("mapy.txt", frame.size().width, frame.size().height);
    mapy = get_transform_mtx("./Matrices/mapy.txt", 640, 480);
    mapx = get_transform_mtx("./Matrices/mapx.txt", 640, 480);
    }

ImageProcessing::~ImageProcessing() {
    video_capture.release();
    cv::destroyAllWindows();
}

image_proc_t ImageProcessing::process_next_frame() {
    int stop_distance = 0;
    float angle_right = 0;
    float angle_left = 0;
    float lateral_position = 0;
    const int R = 10;  // Mesunet noise
    const float Q = 10;  // Process noise


    // Get next frame
    video_capture.grab();
    video_capture.retrieve(frame);

    cv::Mat frame2;

    // int pre_lateral = lateral_position;
    // perspective_transform(frame, transformation_matrix);
    cv::remap(frame, frame2, mapx, mapy, cv::INTER_LINEAR);


    int found_sidelines_success = image_process(frame2, true, lateral_position, angle_left, angle_right, stop_distance);
    if (visualize) {
        cv::imshow("frame", frame2);
    }

    float lateral_diff = lateral_position - lateral_model;
    if (found_sidelines_success != 1 || abs(lateral_diff) > 100) {
        std::cout << "No sidelines" << std::endl;
        output.success = false;
        return output;
    } else {
        kalman(P, lateral_model, lateral_position, R);
        P = P + Q;
        std::cout << "lat:" << lateral_model <<", right angle:" << angle_right <<", left angle:" << angle_left << ", stop:" << stop_distance << std::endl;
    }
    output.success = true;
    output.angle_right = angle_right;
    output.angle_left = angle_left;
    output.lateral_position = lateral_model;
    output.stop_distance = stop_distance;

    return output;
}

// --------- DATA ATT LOGGA I LOGGFIL ----------
// Allt i output :P