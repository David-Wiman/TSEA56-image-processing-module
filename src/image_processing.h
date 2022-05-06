#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <opencv2/opencv.hpp>

#include "help_funtions.h"
#include "raspi_common.h"
#include "replacing_buffer.h"

#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

struct frame_package {
    std::chrono::time_point<std::chrono::high_resolution_clock> created;
    cv::Mat frame;
}

class ImageProcessing {
public:
    ImageProcessing(std::string path_root, const bool save_frames);
    ~ImageProcessing();

    ImageProcessing(const ImageProcessing&) = delete;
    ImageProcessing operator=(const ImageProcessing&) = delete;

    image_proc_t get_next_image_data();

private:
    image_proc_t process_next_frame(cv::Mat &frame);
    void frame_spawner();
    void frame_processor();
    std::thread *spawner_thread = nullptr;
    std::thread *processor_thread = nullptr;
    std::atomic<bool> spawn_threads{true};
    std::atomic<bool> process_threads{true};
    ReplacingBuffer<frame_package> frame_buffer{};

    ReplacingBuffer<image_proc_t> out_buffer{};

    std::string path_root;
    bool const save_frames;
    float lateral_model = 1000;
    float P = 10;

    int pre_left = 1000;
    int pre_right = 1000;
    int left_counter = 0;
    int right_counter = 0;

    cv::Mat mapx{};
    cv::Mat mapy{};
    cv::Mat mask{};
};

#endif  // IMAGE_PROCESSING_H
