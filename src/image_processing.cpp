#include <string>
#include <chrono>
#include <thread>
#include <atomic>

#include "image_processing.h"
#include "help_funtions.h"
#include "log.h"

using namespace std;

ImageProcessing::ImageProcessing(std::string path_name, const bool sf)
: path_root{path_name}, save_frames{sf} {
    Logger::log(INFO, __FILE__, "Image Processing", "Initiate");

    mapy = get_transform_mtx(std::string{path_root + "/Matrices/mapy.txt"}, 320, 240);
    mapx = get_transform_mtx(std::string{path_root + "/Matrices/mapx.txt"}, 320, 240);
    mask = cv::imread(cv::samples::findFile(std::string{path_root + "mask.png"}));
    spawner_thread = new std::thread(&ImageProcessing::frame_spawner, this);
    processor_thread = new std::thread(&ImageProcessing::frame_processor, this);
}

ImageProcessing::~ImageProcessing() {
    // Frame processor could be blocked if frame spawner stops,
    // so stop processor first
    process_threads.store(false);
    processor_thread->join();
    delete processor_thread;

    spawn_threads.store(false);
    spawner_thread->join();
    delete spawner_thread;
    cv::destroyAllWindows();
}

void ImageProcessing::frame_spawner() {
    Logger::log(DEBUG, __FILE__, "frame_spawner", "Initiating");
    cv::VideoCapture video_capture(cv::CAP_ANY);
    if (!video_capture.isOpened()) {
        Logger::log(ERROR, __FILE__, "frame_spawner", "Unable to open camera");
    }
    video_capture.set(cv::CAP_PROP_FRAME_WIDTH, 320);  // set frame size
    video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 240);  // set frame size
    video_capture.set(cv::CAP_PROP_BUFFERSIZE, 3);  // set frame size

    cv::Mat frame{};
    while (spawn_threads.load()) {
        auto const created = chrono::high_resolution_clock::now();
        video_capture.read(frame);
        struct frame_package package{std::move(created), std::move(frame)};
        frame_buffer.store(package);
        auto const now = chrono::high_resolution_clock::now();
        float spawn_time = chrono::duration<float, std::milli>(now-package.created).count();
        Logger::log(DEBUG, __FILE__, "frame_spawner: Spawned new frame took (ms)", spawn_time);
    }
    Logger::log(DEBUG, __FILE__, "frame_spawner", "Closed");
}

void ImageProcessing::frame_processor() {
    Logger::log(DEBUG, __FILE__, "frame_processor", "Initiating");
    while (process_threads.load()) {
        auto const start = chrono::high_resolution_clock::now();
        struct frame_package package = frame_buffer.extract();
        image_proc_t data = process_next_frame(package.frame);
        auto const now = chrono::high_resolution_clock::now();
        float processing_time = chrono::duration<float, std::milli>(now-start).count();
        float total_time = chrono::duration<float, std::milli>(now-package.created).count();
        struct image_data_package out_package{std::move(now), std::move(data)};
        out_buffer.store(out_package);
        Logger::log(DEBUG, __FILE__, "Processing time (ms)", processing_time);
        Logger::log(DEBUG, __FILE__, "Total frame latency (ms)", total_time);
    }
    Logger::log(DEBUG, __FILE__, "frame_processor", "Closed");
}

image_proc_t ImageProcessing::get_next_image_data() {
    auto const start = chrono::high_resolution_clock::now();
    struct image_data_package package = out_buffer.extract();
    auto const now = chrono::high_resolution_clock::now();
    float wait_time = chrono::duration<float, std::milli>(now-start).count();
    Logger::log(DEBUG, __FILE__, "Waited on image data for (ms)", wait_time);
    float data_age = chrono::duration<float, std::milli>(now-package.sent).count();
    Logger::log(DEBUG, __FILE__, "Image data was this old (ms)", data_age);
    return package.data;
}

image_proc_t ImageProcessing::process_next_frame(cv::Mat &frame) {
    const int R_angles = 10;  // Measurement noise
    const float Q_angles = 100;  // Process noise
    const int R_lat = 10;  // Measurement noise
    const float Q_lat = 10;  // Process noise

    cv::Mat frame2{};
    image_proc_t output{};
    // Get next frame


    cv::remap(frame, frame2, mapx, mapy, cv::INTER_LINEAR);
    // Remove inaccurate pixels in botton corners from fisheye+ipm with a mask
    frame2 = frame2 + mask;

    // Find lines and calculate angles and distances
    output = image_process(frame2, (output.angle_left + output.angle_right)/2, save_frames);
    // int angle_diff = output.angle_left - output.angle_right;

    if (save_frames) {
        cv::imwrite("out.jpg", frame2);
    }
    if (lat_l_model == 1000) {
        lat_l_model = static_cast<float> (output.lateral_left);
    }
    if (lat_r_model == 1000) {
        lat_r_model = static_cast<float> (output.lateral_right);
    }
    if (ang_l_model == 1000) {
        ang_l_model = output.angle_left;
    }
    if (ang_r_model == 1000) {
        ang_r_model = output.angle_right;
    }
    int status = 0;

    int left_diff = output.angle_left - static_cast<int>(ang_l_model);
    int right_diff = output.angle_right - static_cast<int>(ang_r_model);
    std::cout<< (left_diff / sqrt(P_ang_l + R_angles)) << std::endl;
    std::cout<< (right_diff / sqrt(P_ang_r + R_angles)) << std::endl;
    float L_left = 0;
    float L_right = 0;
    if (abs(left_diff / sqrt(P_ang_l + R_angles)) > 0.8) {
        L_left = 1000;
    }
    kalman(P_ang_l, ang_l_model, output.angle_left, R_angles, L_left);
    P_ang_l = P_ang_l + Q_angles;

    if (abs(right_diff / sqrt(P_ang_r + R_angles) ) >0.8) {
        L_right = 1000;
    }
    kalman(P_ang_r, ang_r_model, output.angle_right, R_angles, L_right);
    P_ang_r = P_ang_r + Q_angles;


    kalman(P_lat_l, lat_l_model, output.lateral_left, R_lat, L_left);
    P_lat_l = P_lat_l + Q_lat;

    kalman(P_lat_r, lat_r_model, output.lateral_right, R_lat, L_right);
    P_lat_r = P_lat_r + Q_lat;
    // kalman(P_lat_r, lat_l_model, output.lateral_right, R_lat, L_right);
    // P_lat_l = P_lat_l + Q_lat;
        // status +=1;
    // if (abs(right_diff / sqrt(P_ang_r + R_angles) ) < 0.8) {

    //     kalman(P_ang_r, ang_r_model, output.angle_right, R_angles);
    //     P_ang_r = P_ang_r + Q_angles;

    //     kalman(P_lat_r, lat_r_model, output.lateral_right, R_lat);
    //     P_lat_r = P_lat_r + Q_lat;
    //     status +=2;
    // }
    // switch (status) {
    // case 0:  // No correct
    //     output.status_code = 1;
    //     output.angle_left = lat_l_model;
    //     output.angle_right = lat_r_model;

    //     cout << "No angle" << endl;
    //     break;
    // case 1:  // Left correct
    //     output.angle_right = lat_r_model;
    //     output.status_code = 1;
    //     cout << "No detected right angle" << endl;
    //     break;
    // case 2:  // Right correct
    //     output.angle_left = lat_l_model;
    //     output.status_code = 1;
    //     cout << "No detected right angle" << endl;
    //     break;
    // case 3:  // Both correct
    //     output.status_code = 0;
    //     break;
    // default:
    //     cout << "Something wrong with switch case" << endl;
    //     break;
    // }
    // if (output.status_code == 0) {
    //     if (left_counter >= 5) {
    //         pre_left = output.angle_left;
    //         left_counter = 0;
    //     }
    //     if (right_counter >= 5) {
    //         pre_right = output.angle_right;
    //         right_counter = 0;
    //     }
    //     #define left_correct (1 << 0)
    //     #define right_correct (1 << 1)
    //     switch ((abs(left_diff) < 20 ? left_correct : 0) | (abs(right_diff) < 20 ? right_correct : 0)) {
    //         case 0:
    //             left_counter++;
    //             right_counter++;
    //             output.status_code = 2;
    //             cout << "No angle" << endl;
    //             break;
    //         case left_correct:
    //             right_counter++;
    //             output.angle_right = output.angle_left;
    //             output.status_code = 1;
    //             cout << "No detected right angle" << endl;
    //             break;
    //         case right_correct:
    //             left_counter++;
    //             output.angle_left = output.angle_right;
    //             output.status_code = 1;
    //             cout << "No detected left angle" << endl;
    //             break;
    //         case left_correct + right_correct:
    //             kalman(P_lat_l, left_model, output.lateral_left, R);
    //             kalman(P_lat_r, right_model, output.lateral_right, R);
    //             P_lat_l = P_lat_l + Q;
    //             P_lat_r = P_lat_r + Q;

    //             pre_left = output.angle_left;
    //             pre_right = output.angle_right;
    //             left_counter = 0;
    //             right_counter = 0;
    //             break;
    //         default: assert(false);
    //             cout << "Something went wrong with bits" << endl;
    //     }
    // }
    output.lateral_left = static_cast<int> (-0.4432 * lat_l_model  + 24.4);
    output.lateral_right = static_cast<int> (0.46 * lat_r_model - 17.4);
    output.angle_left = ang_l_model;
    output.angle_right = ang_r_model;
    // output.lateral_left = static_cast<int> (-0.4432 * output.lateral_left + 24.4);
    // output.lateral_right = static_cast<int> (0.46 * output.lateral_right - 17.4);

    Logger::log_img_data(output);
    cout<< output.status_code << " : " << output.lateral_left << " : " << output.lateral_right << " : " 
                              << output.angle_left << " : " << output.angle_right << " : "
                              << output.stop_distance << endl;
    return output;
}
