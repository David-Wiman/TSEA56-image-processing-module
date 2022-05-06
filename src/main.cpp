#include "help_funtions.h"
#include "image_processing.h"
#include "log.h"

using namespace std;

   // ------------- FOR TESTING -----------------
void test() {



    cv::Mat frame2;
    cv::Mat mapy = get_transform_mtx("./Matrices/mapy.txt", 320, 240);
    cv::Mat mapx = get_transform_mtx("./Matrices/mapx.txt", 320, 240);
    cv::Mat canny;

    // cv::remap(src, src, mapx, mapy, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    // BORDER_CONSTANT BORDER_REPLICATE BORDER_REFLECT BORDER_WRAP BORDER_REFLECT_101 BORDER_TRANSPARENT 

    cv::Mat mask = cv::imread(cv::samples::findFile("mask.png"));
    // src = src + mask;
    // src.setTo(cv::Scalar(255, 255, 255), mask);
    cv::VideoCapture video_capture(0);
    video_capture.set(cv::CAP_PROP_FRAME_WIDTH, 320); // set frame size
    video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 240); // set frame size

    cv::Mat frame{};
    // cout<<cv::threshold(src, frame, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU)<<endl;

    // v = np.median(image)

    // lower = int(max(0, (1.0 - 0.33) * v))
    // upper = int(min(255, (1.0 + 0.33) * v))
    // edged = cv2.Canny(image, lower, upper)

    cout << "start" << endl;
    for (int i=1; i<1000; i++) {
        video_capture.read(frame);

        cv::remap(frame, frame2, mapx, mapy, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
        frame2 = frame2 + mask;

        cv::Canny(frame2, canny, 50, 200, 3);
        cv::imwrite("canny.jpg", canny);

        image_proc_t return_values = image_process(frame2, true);
        cout<< return_values.status_code << " : " << return_values.lateral_position <<":"<< return_values.angle_left <<":"<< return_values.angle_right <<":"<< return_values.stop_distance<<endl;
        cv::imwrite("out.jpg", frame2);
    }
    cout<<"end"<<endl;


}
    // -------------- END TEST ---------------------


int main() {
    ImageProcessing image_processor("./", true);
    Logger::init(true);
    while (true) {
        image_processor.get_next_image_data();
    }
}
