#include "help_funtions.h"
#include "image_processing.h"

using namespace std;

int main() {
    image_proc_t proccesed_img{};
    // ------------- FOR TESTING -----------------
    // int lateral_position;
    // float angle;
    // int stop_distance;
    // cout<<"start"<<endl;
    // cv::Mat src_out;
    // cv::Mat src = cv::imread(cv::samples::findFile("./Reference/Left_turn.png"));
    // cv::resize(src, src_out, cv::Size(), 0.5, 0.5);
    // for (int i=0; i<100; i++){
    //     image_process(src_out, true, lateral_position, angle, stop_distance);
    // }
    // cout<<"end"<<endl;
    // cv::imwrite("Gray_Image.jpg", src_out);
    
    // -------------- END TEST ---------------------

    // -------------- FOR CAR -----------------

    ImageProcessing imageprocessor(false);
    while (true) {
        proccesed_img = imageprocessor.process_next_frame();
        if (cv::waitKey(10) > 0) break;
    }
    imageprocessor.~ImageProcessing();

    // -------------- END CAR ---------------
}
