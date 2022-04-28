#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include "help_funtions.h"
#include "raspi_common.h"

class ImageProcessing {
public:
    explicit ImageProcessing(std::string path_root, const bool save_frames);
    ~ImageProcessing();

    image_proc_t process_next_frame();

private:
    std::string path_root;
    bool const save_frames;
    float lateral_model = 1000;
    cv::VideoCapture video_capture;
    float P = 10;

    cv::Mat mapx{};
    cv::Mat mapy{};
    cv::Mat mask{};
};

#endif  // IMAGE_PROCESSING_H
