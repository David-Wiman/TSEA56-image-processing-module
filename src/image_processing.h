#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>


#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include "help_funtions.h"
#include "raspi_common.h"

class ImageProcessing {
public:
    ImageProcessing(std::string path_root, const bool save_frames);
    ~ImageProcessing();

    image_proc_t process_next_frame(cv::Mat &frame);

private:
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
