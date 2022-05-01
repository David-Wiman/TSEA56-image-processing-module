#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#include "help_funtions.h"

using std::cout;
using std::endl;
using std::vector;
using std::string;


// ### Help functions
cv::Mat get_transform_mtx(string src, int x, int y) {
    std::ifstream in(src);
    std::string element;
    std::cin.rdbuf(in.rdbuf());
    cv::Mat matrix = cv::Mat(y, x, CV_32FC1);

    for (int j=0; j<y; j++) {
        for (int i=0; i<x; i++) {
            std::cin >> element;
            matrix.at<float>(j,i) = std::stof(element);
        }
    }
    return matrix;
}


void print_lines_on_image(vector<cv::Vec2f> const &lines, cv::Mat& image, cv::Scalar color = cv::Scalar(255, 100, 15)) {
    for (size_t i = 0; i < lines.size(); i++) {
        float rho = lines[i][0];
        float theta = lines[i][1];
        cv::Point pt1, pt2;
        double cos_var = cos(theta);
        double sin_var = sin(theta);
        double x0 = cos_var*rho;
        double y0 = sin_var*rho;

        pt1.x = cvRound(x0 + 1000*(-sin_var));
        pt1.y = cvRound(y0 + 1000*(cos_var));
        pt2.x = cvRound(x0 - 1000*(-sin_var));
        pt2.y = cvRound(y0 - 1000*(cos_var));
        cv::line(image, pt1, pt2, color, 5, cv::LINE_AA);
    }
    return;
}

void remove_negative_rho(vector<cv::Vec2f>& lines) {
    for (unsigned int i = 0; i < lines.size(); i++) {
        if (lines[i][0] < 0) {
            lines[i][1] += PI;
            lines[i][0] = abs(lines[i][0]);
        }
    }
    return;
}


bool comp_rho(cv::Vec2f const & line1, cv::Vec2f const &line2) {
    return line1[0] > line2[0];
}
bool comp_rho_rev(cv::Vec2f const &line1, cv::Vec2f const &line2) {
    return line1[0] < line2[0];
}

float angle_difference(float const angle_1, float const angle_2) {
    float diff = fmod(abs(angle_1 - angle_2), PI);
    if (diff > PI/2) {
        diff = PI - diff;
    }
    return diff;
}

float line_vertical_deviation(cv::Vec2f const &line) {
    float dev = angle_difference(line[1], 0);
    return dev;
}

bool line_is_horizontal(cv::Vec2f const & line) {
    return angle_difference(line[1], PI/2) < 20*PI/180;
}

bool lines_parallell(cv::Vec2f const &line_1, cv::Vec2f const &line_2) {
    return abs(angle_difference(line_1[1], line_2[1])) < 20*PI/180;
}

bool lines_perpendicular(cv::Vec2f const &line_1, cv::Vec2f const &line_2) {
    return abs(angle_difference(line_1[1], line_2[1]) - PI/2) < 20*PI/180;
}

float get_rho(cv::Vec2f const &line) {
    float rho = abs(line[0]);
    return rho;
}

void classify_lines(vector<cv::Vec2f> &lines, vector<cv::Vec2f> &side_lines, vector<cv::Vec2f> &stop_lines) {
    if (lines.size() == 1) {
        side_lines = lines;
        return;
    } else {
        for (unsigned int i=0; i < lines.size(); i++) {
            if (abs(line_vertical_deviation(lines[i])) < PI/4) {
                side_lines.push_back(lines[i]);
            }
        }

    //  Add stop_lines that are perpendicular to side_line and closest to button.
        if (side_lines.size() >= 1) {
            sort(lines.begin(), lines.end(), comp_rho);
            for (unsigned int i=0; i < lines.size(); i++) {
                if (lines_perpendicular(lines[i], side_lines[0])) {
                    stop_lines.push_back(lines[i]);
                    return;
                }
            }
        }
    }
    return;
}

void kalman(float &P, float &x_model, int z, float R) {
    float K = P / (P+R);
    x_model = x_model + K*(static_cast<float>(z)-x_model);
    P = (1-K)*P;
}

float average_rho(vector<cv::Vec2f> const &lines) {
    float sum = 0;
    for (unsigned int i=0; i < lines.size(); i++) {
        sum += lines[i][0];
    }
    return sum/static_cast<float>(lines.size());
}

float average_theta(vector<cv::Vec2f> const &lines) {
    float x = 0, y = 0;
    for (unsigned int i=0; i < lines.size(); i++) {
        x += cos(lines[i][1]);
        y += sin(lines[i][1]);
    }
    return atan2(y, x);
}

cv::Vec2f average_line(vector<cv::Vec2f> const &lines) {
    cv::Vec2f line;
    float x = 0, y = 0;
    for (unsigned int i=0; i < lines.size(); i++) {
        line[0] += lines[i][0];  // rho
        x += cos(lines[i][1]);
        y += sin(lines[i][1]);
    }
    line[0] = line[0]/static_cast<float>(lines.size());
    line[1] = atan2(y, x);
    return line;
}

void get_unique_lines(vector<cv::Vec2f> &lines, float theta_margin = 5, float rho_margin = 60) {
    if (lines.size() <= 1) {
        return;
    }
    remove_negative_rho(lines);
    vector<vector<cv::Vec2f>> line_clusters;

    // Group similar lines
    vector<cv::Vec2f> line;
    line.push_back(lines[0]);
    line_clusters.push_back(line);
    bool sim = false;

    for (unsigned int i=0; i < lines.size(); i++) {
        sim = false;
        for (unsigned int j=0; j < line_clusters.size(); j++) {
            float delta_rho = average_rho(line_clusters[j]);
            float delta_theta = average_theta(line_clusters[j]);
            delta_rho = abs(delta_rho - lines[i][0]);
            delta_theta = angle_difference(delta_theta, lines[i][1]);
            if (delta_theta < theta_margin*PI/180 && delta_rho < rho_margin) {
                line_clusters[j].push_back(lines[i]);
                sim = true;
                break;
            }
        }
        if (sim == false) {
            vector<cv::Vec2f> line2;
            line2.push_back(lines[i]);
            line_clusters.push_back(line2);
        }
    }

    // Merge similar lines by averaging
    vector<cv::Vec2f> unique_lines;
    for (unsigned int i=0; i<line_clusters.size(); i++) {
        cv::Vec2f averaged_line = average_line(line_clusters[i]);
        unique_lines.push_back(averaged_line);
    }
    lines = unique_lines;
    return;
}


// ### Position calculation
image_proc_t get_lateral_position(vector<cv::Vec2f> &side_lines, float image_w, float image_h) {
    sort(side_lines.begin(), side_lines.end(), comp_rho_rev);
    float rho_l = side_lines[0][0];
    float angle_left = side_lines[0][1];
    float rho_r = side_lines[1][0];
    float angle_right = side_lines[1][1];

    // cout << "rho_l: " << rho_l << "\ttheta_l: " << theta_l << endl;
    // cout << "rho_r: " << rho_r << "\ttheta_r: "  << theta_r << endl;

    float x_l = (rho_l - image_h*sin(angle_left)) / cos(angle_left);
    float x_r = (rho_r - image_h*sin(angle_right)) / cos(angle_right);
    // cout <<  "x_l: " << x_l << "\tx_r: " << x_r << endl;

    float b_v = x_r - x_l;
    float b_vl = image_w/2 - x_l;
    float b_vr = x_r - image_w/2;
    // cout << "b_vl: " << b_vl << "\tb_vr: " << b_vr << endl;

    image_proc_t return_values{};
    return_values.lateral_position = static_cast<int>(b_vr*cos(angle_right) + b_v - b_vl*cos(angle_left)/2);

    // cout << "deviation: " <<  (theta_l+theta_r)/2 <<
    //       "\nl_lat left: " << b_v - b_vl*cos(theta_l) <<
    //       "\tl_lat right: " << b_vr*cos(theta_r);
    return_values.angle_left = static_cast<int>(180*angle_left/PI) % 180;
    if (return_values.angle_left > 90) {
        return_values.angle_left = return_values.angle_left - 180;
        cout<<"detected_left_angle > 90 degrees"<<endl;
    }
    
    return_values.angle_right = static_cast<int>(180*angle_right/PI) % 180;
    if (return_values.angle_right > 90) {
        return_values.angle_right = return_values.angle_right - 180;
        cout<<"detected_left_angle > 90 degrees"<<endl;
    }
    return return_values;
}

int get_stop_line_distance(cv::Vec2f const &stop_line, float image_w, float image_h) {
    float rho = stop_line[0];
    float theta = stop_line[1];
    float pixel_dist = image_h*sin(theta) + image_w/2*cos(theta) - rho;
    return cvRound(0.335*pixel_dist+8);
}

image_proc_t image_process(cv::Mat& image, bool print_lines) {
    cv::Mat edges, gray, gauss;
    vector<cv::Vec2f> lines, side_lines, stop_lines;
    image_proc_t return_values{};
    return_values.status_code = 2;
    float image_height = static_cast<float>(image.size().height);
    float image_width = static_cast<float>(image.size().width);

    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gauss, cv::Size(3, 3), 0, 0);
    cv::Canny(gauss, edges, 100, 180, 3);
    cv::HoughLines(edges, lines, 1, PI/180, 70, 0, 0);
    get_unique_lines(lines, 10, 58);
    classify_lines(lines, side_lines, stop_lines);

    if (side_lines.size() >= 2) {
        return_values = get_lateral_position(side_lines, image_width, image_height);
        return_values.status_code = 0;
    } else if (side_lines.size() == 1) {
        int angle = static_cast<int>(180*side_lines[0][1]/PI) % 180;
        if (angle > 90) {
            angle = angle - 180;
            cout<<"detected_angle > 90 degrees"<<endl;
        }
        return_values.angle_left = angle;
        return_values.angle_right = angle;
        return_values.status_code = 1;

    } else {
        return_values.status_code = 2;
    } 

    if (print_lines) {
        print_lines_on_image(side_lines, gauss, cv::Scalar(255, 100, 15));
        cv::imwrite("canny.jpg", edges);
    }

    if (stop_lines.size() != 0) {
        return_values.stop_distance = get_stop_line_distance(stop_lines[0], image_width, image_height);
        // cout << "l_s: " << stop_distance << endl;
        if (print_lines) {
            print_lines_on_image(stop_lines, gauss, cv::Scalar(0, 255, 0));
        }
    } else {
        return_values.stop_distance = -1;
    }
    return return_values;
}


