#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

typedef struct image_processing_output {
    int lateral_position;
    int stop_distance;
} image_proc_t;

class ImageProcessing {
public:
    ImageProcessing(bool visualize=false);
    ~ImageProcessing();

    image_proc_t process_next_frame();

private:
    cv::VideoCapture video_capture;
};

#endif  // IMAGE_PROCESSING_H
