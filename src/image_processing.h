#include <opencv2/opencv.hpp>
#include<iostream>
#include <fstream>

#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

typedef struct image_proc_t {
    bool success = 0;
    float lateral_position = 0;
    int stop_distance = 0;
    float angle_left = 0;
    float angle_right = 0;

} image_proc_t;

class ImageProcessing {
 public:
    explicit ImageProcessing(const bool visualize);
    ~ImageProcessing();

    image_proc_t process_next_frame();

 private:
    const bool visualize;
    float lateral_model = 100;
    cv::VideoCapture video_capture;
    float P = 10;

    cv::Mat mapx{};
    cv::Mat mapy{};
    image_proc_t output{};
    cv::Mat frame{};
};

#endif  // IMAGE_PROCESSING_H
