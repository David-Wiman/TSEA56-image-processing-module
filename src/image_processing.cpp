#include "image_processing.h"
#include "help_funtions.h"
#include "log.h"

#include <string>

ImageProcessing::ImageProcessing(std::string path_name, const bool sf)
: path_root{path_name}, save_frames{sf}, video_capture{cv::CAP_ANY} {
    // Check if we succeeded to open video capture
    if (!video_capture.isOpened()) {
        Logger::log(ERROR, __FILE__, "Image processing", "Unable to open camera");
        return;
    }
    video_capture.set(cv::CAP_PROP_FRAME_WIDTH, 320); // set frame size
    video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 240); // set frame size
    // video_capture.set(cv::CAP_PROP_AUTOFOCUS, 0); // turn the autofocus off

    mapy = get_transform_mtx(std::string{path_root + "/Matrices/mapy.txt"}, 320, 240);
    mapx = get_transform_mtx(std::string{path_root + "/Matrices/mapx.txt"}, 320, 240);
    mask = cv::imread(cv::samples::findFile(std::string{path_root + "mask.png"}));
}

ImageProcessing::~ImageProcessing() {
    video_capture.release();
    cv::destroyAllWindows();
}

image_proc_t ImageProcessing::process_next_frame() {
    const int R = 10;  // Measurement noise
    const float Q = 10;  // Process noise
    cv::Mat frame{};
    cv::Mat frame2;
    image_proc_t output{};
    // Get next frame
    video_capture.read(frame);


    cv::remap(frame, frame2, mapx, mapy, cv::INTER_LINEAR);
    // Remove inaccurate pixels in botton corners from fisheye+ipm with a mask
    frame2 = frame2 + mask;

    // Find lines and calculate angles and distances
    output = image_process(frame2, save_frames);
    std::cout<< output.status_code << std::endl;
    int angle_diff = output.angle_left - output.angle_right;

    if (save_frames) {
        cv::imwrite("out.jpg", frame2);
    }
    if (lateral_model == 1000) {
        lateral_model = static_cast<float> (output.lateral_position);
    }
    int left_diff = output.angle_left - static_cast<int>(pre_left);
    int right_diff = output.angle_right - static_cast<int>(pre_right);
    if (output.status_code == 0) {
        #define left_correct (1 << 0)
        #define right_correct (1 << 1)

        switch((abs(left_diff) < 20 ? left_correct : 0) | (abs(right_diff) < 20 ? right_correct : 0)) {
            case 0:
                output.status_code = 2;
                std::cout<<"No angle"<<std::endl;
                goto skip_kalman;
            case left_correct:
                output.angle_right = output.angle_left;
                output.status_code = 1;
                std::cout<<"Left angle"<<std::endl;
                goto skip_kalman;
            case right_correct:
                output.angle_left = output.angle_right;
                output.status_code = 1;
                std::cout<<"Right angle"<<std::endl;
                goto skip_kalman;
            case left_correct + right_correct:
                break;
            default: assert(false);   //something went wrong with the bits.
                std::cout<<"Something went wrong with bits"<<std::endl;
        }
        counter = 0;
        kalman(P, lateral_model, output.lateral_position, R);
        P = P + Q;
        pre_left = output.angle_left;
        pre_right = output.angle_right;
        }
    skip_kalman:
    output.lateral_position = static_cast<int> (lateral_model / 6.36); // skaling with 22 to get dist in cm

    Logger::log_img_data(output);
    std::cout<<angle_diff<<std::endl;
    std::cout<< output.status_code << " : " << output.lateral_position <<" : "<<
                output.angle_left <<" : "<< output.angle_right <<" : "<<
                output.stop_distance<<std::endl;
    return output;
}

// --------- TODO ----------

// For output.severity 
//  - if all is good (two sidelines detected), return 0
//  - if something is wrong (only one sideline detected for example), return 1
//  - if something is VERY wrong (no sidelines detected or only one for a
//    certain amount of time), return 2

// reset model if no sidelines found in xx cycles
