#include <string>

#include "image_processing.h"
#include "help_funtions.h"
#include "log.h"

using std::cout;
using std::endl;


ImageProcessing::ImageProcessing(std::string path_name, const bool sf)
: path_root{path_name}, save_frames{sf} {
    // Check if we succeeded to open video capture
    // if (!video_capture.isOpened()) {
    //     Logger::log(ERROR, __FILE__, "Image processing", "Unable to open camera");
    //     return;
    // }
    // video_capture.set(cv::CAP_PROP_FRAME_WIDTH, 320);  // set frame size
    // video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 240);  // set frame size
    // video_capture.set(cv::CAP_PROP_AUTOFOCUS, 0); // turn the autofocus off

    mapy = get_transform_mtx(std::string{path_root + "/Matrices/mapy.txt"}, 320, 240);
    mapx = get_transform_mtx(std::string{path_root + "/Matrices/mapx.txt"}, 320, 240);
    mask = cv::imread(cv::samples::findFile(std::string{path_root + "mask.png"}));
}

ImageProcessing::~ImageProcessing() {
    cv::destroyAllWindows();
}

image_proc_t ImageProcessing::process_next_frame(cv::Mat &frame) {
    const int R = 10;  // Measurement noise
    const float Q = 10;  // Process noise
    cv::Mat frame2{};
    image_proc_t output{};
    // Get next frame


    cv::remap(frame, frame2, mapx, mapy, cv::INTER_LINEAR);
    // Remove inaccurate pixels in botton corners from fisheye+ipm with a mask
    frame2 = frame2 + mask;

    // Find lines and calculate angles and distances
    output = image_process(frame2, save_frames);
    int angle_diff = output.angle_left - output.angle_right;

    if (save_frames) {
        cv::imwrite("out.jpg", frame2);
    }
    if (lateral_model == 1000) {
        lateral_model = static_cast<float> (output.lateral_position);
    }
    if (pre_left == 1000) {
        pre_left = output.angle_left;
    }
    if (pre_right == 1000) {
        pre_right = output.angle_right;
    }
    int left_diff = output.angle_left - static_cast<int>(pre_left);
    int right_diff = output.angle_right - static_cast<int>(pre_right);
    if (output.status_code == 0) {
        if (left_counter >= 5) {
            pre_left = output.angle_left;
            left_counter = 0;
        }
        if (right_counter >= 5) {
            pre_right = output.angle_right;
            right_counter = 0;
        }
        #define left_correct (1 << 0)
        #define right_correct (1 << 1)
        switch ((abs(left_diff) < 20 ? left_correct : 0) | (abs(right_diff) < 20 ? right_correct : 0)) {
            case 0:
                left_counter++;
                right_counter++;
                output.status_code = 2;
                cout << "No angle" << endl;
                break;
            case left_correct:
                right_counter++;
                output.angle_right = output.angle_left;
                output.status_code = 1;
                cout << "No detected right angle" << endl;
                break;
            case right_correct:
                left_counter++;
                output.angle_left = output.angle_right;
                output.status_code = 1;
                cout << "No detected left angle" << endl;
                break;
            case left_correct + right_correct:
                kalman(P, lateral_model, output.lateral_position, R);
                P = P + Q;
                pre_left = output.angle_left;
                pre_right = output.angle_right;
                left_counter = 0;
                right_counter = 0;
                break;
            default: assert(false);
                cout << "Something went wrong with bits" << endl;
        }
    }

    output.lateral_position = static_cast<int> (0.3838 * lateral_model - 16.121);

    Logger::log_img_data(output);
    cout << angle_diff << endl;
    cout<< output.status_code << " : " << output.lateral_position << " : " <<
                output.angle_left << " : " << output.angle_right << " : " <<
                output.stop_distance << endl;
    return output;
}

// --------- TODO ----------

// For output.severity
//  - if all is good (two sidelines detected), return 0
//  - if something is wrong (only one sideline detected for example), return 1
//  - if something is VERY wrong (no sidelines detected or only one for a
//    certain amount of time), return 2

// reset model if no sidelines found in xx cycles
