#include <opencv2/opencv.hpp>

#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

typedef struct image_processing_output {
    bool success = 0;
    int lateral_position = 0;
    int stop_distance = 0;
    float road_angle = 0;
} image_proc_t;

class ImageProcessing {
 public:
    ImageProcessing(const bool visualize);
    ~ImageProcessing();

    image_proc_t process_next_frame();

 private:
    const bool visualize;
    int lateral_model = 100;
    cv::VideoCapture video_capture;
    float P = 10;
    cv::Mat transformation_matrix{};

    image_proc_t output{};
    cv::Mat frame{};
};

#endif  // IMAGE_PROCESSING_H
