#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

typedef struct image_processing_output {
    bool success;
    int lateral_position;
    int stop_distance;
    float angle;
} image_proc_t;

class ImageProcessing {
public:
    ImageProcessing(bool visualize=false, int lateral_position=100);
    ~ImageProcessing();

    image_proc_t process_next_frame();

private:    
    bool visualize;
    int lateral_position;
    cv::VideoCapture video_capture;
    bool is_up{false};
    bool is_down{false};

};

#endif  // IMAGE_PROCESSING_H
