#ifndef SRC_HELP_FUNTIONS_H_
#define SRC_HELP_FUNTIONS_H_


#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

const float PI = 3.14159265359f;

typedef struct image_proc_t {
    bool status_code = 0;
    int lateral_position = 0;
    int stop_distance = 0;
    int angle_left = 0;
    int angle_right = 0;

} image_proc_t;


// private:
cv::Mat get_transform_mtx(std::string src, int x, int y);
void print_lines_on_image(std::vector<cv::Vec2f> const & lines, cv::Mat& image, cv::Scalar color);
void remove_negative_rho(std::vector<cv::Vec2f>& lines);
cv::Mat print_circles_on_image(std::vector<cv::Vec3f> circles, cv::Mat& image, cv::Scalar color);
bool comp_rho(cv::Vec2f const &line1, cv::Vec2f const &line2);
bool comp_rho_rev(cv::Vec2f const &line1, cv::Vec2f const &line2);
float angle_difference(float const angle_1, float const angle_2);
float circle_line_dist(cv::Vec3f circle, cv::Vec2f line);
bool circle_between_lines(cv::Vec2f line1, cv::Vec2f line2, cv::Vec3f circle);
float line_vertical_deviation(cv::Vec2f const &line);
bool line_is_horizontal(cv::Vec2f const &line);
bool lines_parallell(cv::Vec2f const &line_1, cv::Vec2f const &line_2);
bool lines_perpendicular(cv::Vec2f const &line_1, cv::Vec2f const &line_2);
float get_rho(cv::Vec2f const &line);
void classify_lines(std::vector<cv::Vec2f> &lines, std::vector<cv::Vec2f> &side_lines, std::vector<cv::Vec2f> &stop_lines);
cv::Mat perspective_transform_init();
void perspective_transform(cv::Mat& image, cv::Mat const &matrix);
void kalman(float &P, float &x_model, int z, float R);
int add_vote(cv::Vec2f line1, cv::Vec2f line2);
int sum_votes(std::vector<cv::Vec2f> lines);
float average_rho(std::vector<cv::Vec2f> const &lines);
float average_circle_coord(std::vector<cv::Vec3f> const &lines, int position);
float average_theta(std::vector<cv::Vec2f> const &lines);
cv::Vec2f average_line(std::vector<cv::Vec2f> const &lines);
cv::Vec3f average_circle(std::vector<cv::Vec3f> const &circles);
void get_unique_lines(std::vector<cv::Vec2f> &lines, float theta_margin, float rho_margin);
std::vector<cv::Vec3f> get_unique_circles(std::vector<cv::Vec3f>& circles);
image_proc_t get_lateral_position(std::vector<cv::Vec2f> &side_lines, float image_w, float image_h);
// float get_road_angle(std::vector<cv::Vec2f> side_lines);
int get_stop_line_distance(cv::Vec2f const &stop_line, float image_w, float image_h);
// public:
// void prefilter(int& lateral_position, int pre_lateral_position, bool& is_down, bool& is_up);
image_proc_t image_process(cv::Mat& imput_image, bool print_lines);


#endif  // SRC_HELP_FUNTIONS_H_
