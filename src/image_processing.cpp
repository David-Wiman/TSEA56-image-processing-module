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

    transformation_matrix = get_transform_mtx("matrix_constants.txt", 3, 3);
    mtx_matrix = get_transform_mtx("mtx.txt", 3, 3);
    dist_matrix = get_transform_mtx("dist.txt", 1, 5);
    newcameramtx_matrix = get_transform_mtx("newcameramtx.txt", 3, 3);


    // std::ifstream fin("matrix_constants.txt");
    // int element;

    // cv::Mat transformation_matrix = cv::Mat(3,3, CV_64F);

    // for (int i=0; i<3; i++) {
    //     for (int j=0; j<3; j++){
    //         fin >> element;
    //         transformation_matrix.at<double>(i,j) = element;
    //     }
    // }
    // for (int i=0; i<3; i++) {
    //     for (int j=0; j<3; j++){
    //         std::cout << matrix.at<double>(i,j) << " ";
    //     }
    //     std::cout << "\n";
    // }

    // transformation_matrix = perspective_transform_init();
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
        std::cout << "lat:" << lateral_model <<", angle:" << road_angle << ", stop:" << stop_distance << std::endl;
    }
    output.success = true;
    output.road_angle = road_angle;
    output.lateral_position = lateral_model;
    output.stop_distance = stop_distance;

    return output;
}
