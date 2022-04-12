#include <opencv2/opencv.hpp>
//#include <opencv2/videoio.hpp>

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

    image_proc_t process_next_frame(cv::Mat frame);

private:
    bool visualize;
    int lateral_position;
    int stop_distance;
    int angle;
    cv::VideoCapture video_capture;
    bool is_up{false};
    bool is_down{false};
    image_proc_t output{0};
    cv::Mat frame{};
    cv::Mat out{};
};

class Process {
public:
  Process(const char* default_file);
  ~Process();
private:
  const char* default_file;
  cv::Mat src;

};

class Camera {
public:
  Camera();
  ~Camera();
  void start_camera();
private:
  cv::Mat frame;
  image_proc_t output{0};
};

#endif  // IMAGE_PROCESSING_H
