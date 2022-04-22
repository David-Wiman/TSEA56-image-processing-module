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

    cv::Mat perspective_matrix = get_transform_mtx("perspective_matrix.txt", 3, 3);
    cv::Mat mtx_matrix = get_transform_mtx("mtx.txt", 3, 3);
    cv::Mat dist_matrix = get_transform_mtx("dist.txt", 1, 5);
    cv::Mat newcameramtx_matrix = get_transform_mtx("newcameramtx.txt", 3, 3);

    cv::initUndistortRectifyMap(mtx_matrix, dist_matrix, cv::Mat(), (perspective_matrix * newcameramtx_matrix), cv::Size(frame.size().width, frame.size().height), CV_32FC1, mapx, mapy);

    // cv::getOptimalNewCameraMatrix(mtx_matrix, dist_matrix, gray.size(), 0.1, gray.size(), 0),
	// 		gray.size(), CV_16SC2, map1, map2);

    // getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(WIDTH, HEIGHT), 1,
    //                                                   cv::Size(WIDTH, HEIGHT), 0)
    // initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), data ,cv::Size(WIDTH, HEIGHT), cv::CV_16SC2, map1, map2);


    // cv::initUndistortRectifyMap(*m_camData, *m_distData, cv::Mat(), *m_camData, cv::Size(resolution.first, resolution.second), cv::CV_32FC1, *m_undistMap1, *m_undistMap2);


    // //create undistorted image


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
    float angle_right = 0;
    float angle_left = 0;
    float lateral_position = 0;
    const int R = 10;  // Mesunet noise
    const float Q = 10;  // Process noise


    // Get next frame
    video_capture.grab();
    video_capture.retrieve(frame);

    // int pre_lateral = lateral_position;
    // perspective_transform(frame, transformation_matrix);
    cv::remap(frame, frame, mapx, mapy, cv::INTER_LINEAR);

    int found_sidelines_success = image_process(frame, true, lateral_position, angle_left, angle_right, stop_distance);
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