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
    const int R = 10;  // Measurement noise
    const float Q = 10;  // Process noise
    cv::Mat frame2{};
    image_proc_t output{};
    // Get next frame


    cv::remap(frame, frame2, mapx, mapy, cv::INTER_LINEAR);
    // Remove inaccurate pixels in botton corners from fisheye+ipm with a mask
    frame2 = frame2 + mask;

    // Find lines and calculate angles and distances
    output = image_process(frame2, (output.angle_left + output.angle_right)/2, save_frames);
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

    output.lateral_position = static_cast<int> (0.2883 * lateral_model - 10.252);

    Logger::log_img_data(output);
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
