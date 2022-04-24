#include "help_funtions.h"
#include "image_processing.h"

using namespace std;

    // ------------- FOR TESTING -----------------
void test() {
    float lateral_position;
    float angle_left;
    float angle_right;
    int stop_distance;

    // cv::Mat src = cv::imread(cv::samples::findFile("./ref_images_640_420/save_as_filename5.jpg"));
    // cv::Mat src = cv::imread(cv::samples::findFile("./Reference/Left_turn.png"));
    cv::Mat src = cv::imread(cv::samples::findFile("./Test_road/test.jpg"));
    cv::resize(src, src, cv::Size(640, 480));

    cv::Mat frame2;
    cv::Mat mapy = get_transform_mtx("./Matrices/mapy.txt", 640, 480);
    cv::Mat mapx = get_transform_mtx("./Matrices/mapx.txt", 640, 480);
    cout<<"start"<<endl;
    cv::Mat src_out;

    cv::remap(src, src, mapx, mapy, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    // BORDER_CONSTANT BORDER_REPLICATE BORDER_REFLECT BORDER_WRAP BORDER_REFLECT_101 BORDER_TRANSPARENT 

    cv::Mat mask = cv::imread(cv::samples::findFile("mask.png"));
    src = src + mask;
    // src.setTo(cv::Scalar(255, 255, 255), mask);
    // cv::Canny(src, src_out, 50, 200, 3);
    image_process(src, true, lateral_position, angle_left, angle_right, stop_distance);

    cout<<lateral_position <<":"<< angle_left <<":"<< angle_right <<":"<< stop_distance<<endl;
    cv::imwrite("out.jpg", src);
}
    // -------------- END TEST ---------------------


int main() {
    test();
    // -------------- FOR CAR -----------------

    // ImageProcessing imageprocessor(false);
    // while (true) {
    //     proccesed_img = imageprocessor.process_next_frame();
    //     if (cv::waitKey(10) > 0) break;
    // }
    // imageprocessor.~ImageProcessing();

    // -------------- END CAR ---------------
}
