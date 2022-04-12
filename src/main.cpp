#include "help_funtions.h"
#include "image_processing.h"

int main() {
    cv::Mat src = cv::imread(cv::samples::findFile("./Reference/Left_turn.png"), cv::IMREAD_GRAYSCALE);
    int lateral_position;
    int stop_distance;
    cv::Mat out;

    image_process(src, true, lateral_position, stop_distance);
    cv::imwrite("Gray_Image.jpg", src);

    // int lateral_position = 100;
    // ImageProcessing imageprocessor(true, 100);
    // while (true) {
    //     imageprocessor.process_next_frame();
    // }
}