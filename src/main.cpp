#include "help_funtions.h"
#include "image_processing.h"

using namespace std;

int main() {
    // ------------- FOR TESTING -----------------
    // int lateral_position;
    // float angle;
    // int stop_distance;
    // cv::Mat src = cv::imread(cv::samples::findFile("./Reference/Left_turn.png"), cv::IMREAD_GRAYSCALE);
    // for (int i=0; i<10; i++){
    //     image_process(src, true, lateral_position, angle, stop_distance);
    // cv::imwrite("Gray_Image.jpg", src);

    // -------------- END TEST ---------------------

    // -------------- FOR CAR -----------------

    ImageProcessing imageprocessor(true);
    while (true) {
        imageprocessor.process_next_frame();
        if (cv::waitKey(10) > 0) break;
    }
    imageprocessor.~ImageProcessing();

    // -------------- END CAR ---------------
}
