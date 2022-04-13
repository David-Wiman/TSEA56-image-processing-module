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
    int lateral_position = 100;
    int lateral_model = 0;
    int stop_distance = 0;
    float road_angle = 0;
    cv::VideoCapture video_capture;
    float P = 10;
    const int R = 10;  // Mesunet noise
    const float Q = 10;  // Process noise
    bool is_up{false};
    bool is_down{false};
    cv::Mat transformation_matrix{};

    image_proc_t output{};
    cv::Mat frame{};
    cv::Mat out{};
};

#endif  // IMAGE_PROCESSING_H
