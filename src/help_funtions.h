#ifndef HEMP_IMAGE_H
#define HEMP_IMAGE_H

#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>


// private:
    cv::Mat print_lines_on_image(std::vector<cv::Vec2f> lines, cv::Mat image, cv::Scalar color);
    std::vector<cv::Vec2f> remove_negative_rho(std::vector<cv::Vec2f> lines);
    cv::Mat print_circles_on_image(std::vector<cv::Vec3f> circles, cv::Mat image, cv::Scalar color);
    bool comp_rho(cv::Vec2f line1, cv::Vec2f line2);
    bool comp_rho_rev(cv::Vec2f line1, cv::Vec2f line2);
    float angle_difference(float angle_1, float angle_2);
    float circle_line_dist(cv::Vec3f circle, cv::Vec2f line);
    bool circle_between_lines(cv::Vec2f line1, cv::Vec2f line2, cv::Vec3f circle);
    float line_vertical_deviation(cv::Vec2f line);
    bool line_is_horizontal(cv::Vec2f line);
    bool lines_parallell(cv::Vec2f line_1, cv::Vec2f line_2);
    bool  lines_perpendicular(cv::Vec2f line_1, cv::Vec2f line_2);
    float get_rho(cv::Vec2f line);
    void classify_lines(std::vector<cv::Vec2f> lines, std::vector<cv::Vec2f> &side_lines, std::vector<cv::Vec2f> &stop_lines);
    cv::Mat perspective_transform(cv::Mat image);
    void kalman(float &P, int &x_model, int z, int R);
    int add_vote(cv::Vec2f line1, cv::Vec2f line2);
    int sum_votes(std::vector<cv::Vec2f> lines);
    float average_rho(std::vector<cv::Vec2f> lines);
    float average_circle_coord(std::vector<cv::Vec3f> lines, int position);
    float average_theta(std::vector<cv::Vec2f> lines);
    cv::Vec2f average_line(std::vector<cv::Vec2f> lines);
    cv::Vec3f average_circle(std::vector<cv::Vec3f> circles);
    std::vector<cv::Vec2f> get_unique_lines(std::vector<cv::Vec2f> lines, int theta_margin, int rho_margin);
    std::vector<cv::Vec3f> get_unique_circles(std::vector<cv::Vec3f> circles);
    float get_lateral_position(std::vector<cv::Vec2f> side_lines, int image_w, int image_h);
    int get_stop_line_distance(cv::Vec2f stop_line, int image_w, int image_h);
// public:
    void prefilter(int& lateral_position, int pre_lateral_position, bool& is_down, bool& is_up);
    int image_process(cv::Mat imput_image, cv::Mat &output_image, int &lateral_position, int &stop_distance);


#endif
