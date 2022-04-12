#include "help_funtions.h"
#include "image_processing.h"

using namespace std;

int main() {
    Process process("./Reference/Left_turn.png");
    Camera camera;
    camera.start_camera();


    //cv::Mat src = cv::imread(cv::samples::findFile("./Reference/Left_turn.png"), cv::IMREAD_GRAYSCALE);
    // int lateral_position;
    // int stop_distance;
    // cv::Mat out;
    //cout << "hej1111111111111111111\n\n\n\n";

    // ImageProcessing img_proc(false, 100);
    // img_proc.process_next_frame();

    // image_process(src, true, lateral_position, stop_distance);
    // cv::imwrite("Gray_Image.jpg", src);

    //int lateral_position = 100;
    //ImageProcessing imageprocessor(true, 100);
    //while (true) {
    //    imageprocessor.process_next_frame();
    //}
}
