#include "help_funtions.h"
#include "image_processing.h"

using namespace std;

int main() {
    // ------------- FOR TESTING -----------------
    float lateral_position;
    float angle_left;
    float angle_right;
    int stop_distance;
    cv::Mat frame = cv::imread(cv::samples::findFile("reference.jpg"));
    cv::Mat frame2;
    cv::Mat mapy = get_transform_mtx("./Matrices/mapy.txt", 640, 480);
    cv::Mat mapx = get_transform_mtx("./Matrices/mapx.txt", 640, 480);
    cout<<"start"<<endl;
    cv::Mat src_out;
    cv::Mat src = cv::imread(cv::samples::findFile("./Reference/Left_turn.png"));
    // cv::resize(src, src_out, cv::Size(), 0.5, 0.5);

    cv::remap(src, src, mapx, mapy, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    // BORDER_CONSTANT BORDER_REPLICATE BORDER_REFLECT BORDER_WRAP BORDER_REFLECT_101 BORDER_TRANSPARENT 

    image_process(src, true, lateral_position, angle_left, angle_right, stop_distance);

    cout<<"end"<<endl;
    cv::imwrite("out.jpg", src);
    
    // -------------- END TEST ---------------------

    // -------------- FOR CAR -----------------

    // ImageProcessing imageprocessor(false);
    // while (true) {
    //     proccesed_img = imageprocessor.process_next_frame();
    //     if (cv::waitKey(10) > 0) break;
    // }
    // imageprocessor.~ImageProcessing();

    // -------------- END CAR ---------------
}
