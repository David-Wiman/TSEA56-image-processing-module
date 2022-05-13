#ifndef SRC_HELP_FUNTIONS_H_
#define SRC_HELP_FUNTIONS_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>

#include <opencv2/opencv.hpp>

#include "raspi_common.h"

const float PI = 3.14159265359f;

cv::Mat get_transform_mtx(std::string src, int x, int y);
void print_lines_on_image(std::vector<cv::Vec2f> const & lines, cv::Mat& image, cv::Scalar color);
cv::Mat print_circles_on_image(std::vector<cv::Vec3f> circles, cv::Mat& image, cv::Scalar color);
void remove_negative_rho(std::vector<cv::Vec2f>& lines);
bool comp_rho(cv::Vec2f const &line1, cv::Vec2f const &line2);
bool comp_rho_rev(cv::Vec2f const &line1, cv::Vec2f const &line2);
float angle_difference(float const angle_1, float const angle_2);
float line_vertical_deviation(cv::Vec2f const &line);
bool line_is_horizontal(cv::Vec2f const &line);
bool lines_parallell(cv::Vec2f const &line_1, cv::Vec2f const &line_2);
bool lines_perpendicular(cv::Vec2f const &line_1, cv::Vec2f const &line_2);
float get_rho(cv::Vec2f const &line);
void classify_lines(std::vector<cv::Vec2f> &lines, std::vector<cv::Vec2f> &side_lines, std::vector<cv::Vec2f> &stop_lines);
void kalman(float &P, float &x_model, int z, float R);
int add_vote(cv::Vec2f line1, cv::Vec2f line2);
int sum_votes(std::vector<cv::Vec2f> lines);
float average_rho(std::vector<cv::Vec2f> const &lines);
float average_theta(std::vector<cv::Vec2f> const &lines);
cv::Vec2f average_line(std::vector<cv::Vec2f> const &lines);
void get_unique_lines(std::vector<cv::Vec2f> &lines, float theta_margin, float rho_margin);

float average_circle_coord(std::vector<cv::Vec3f> const &lines, int position);
cv::Vec3f average_circle(std::vector<cv::Vec3f> const &circles);
std::vector<cv::Vec3f> get_unique_circles(std::vector<cv::Vec3f>& circles);
float circle_line_dist(cv::Vec3f circle, cv::Vec2f line);

image_proc_t get_lateral_position(std::vector<cv::Vec2f> &side_lines, float image_w, float image_h);
int get_stop_line_distance(cv::Vec2f const &stop_line, float image_w, float image_h);

image_proc_t image_process(cv::Mat& imput_image, int pre_angle, bool print_lines);


#endif  // SRC_HELP_FUNTIONS_H_
