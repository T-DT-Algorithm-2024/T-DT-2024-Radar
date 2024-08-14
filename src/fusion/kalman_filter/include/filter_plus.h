#include "opencv2/opencv.hpp"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/types.hpp>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/time.hpp>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#pragma once


class Kalman_filter_plus {
private:
    cv::KalmanFilter KF;
public:
    float Distance(pcl::PointXY &a, pcl::PointXY &b) {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }
    float get_time() {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timer);
        // std::cout << duration.count()/1000.0 <<"ms"<< std::endl;
        return duration.count() / 1000.0;
    }
    float last_time = 0;
    std::chrono::steady_clock::time_point timer;//最后更新时间
    float delete_time = 2.0;//超时删除
    std::vector<std::pair<double, pcl::PointXY>> history;
    std::vector<std::pair<int ,int>> detect_history;//第一个是颜色，第二个是数字
    int max_history = 20;

    pcl::PointXY predict_point;
    float detect_r = 1;
    float car_speed = 2;
    float car_max_speed = 3.0;
    cv::Scalar color;
    bool has_updated = false;
    cv::Mat Q= cv::Mat::zeros(4, 4, CV_32F);
    cv::Mat R= cv::Mat::zeros(2, 2, CV_32F);
    float dt_=0.1f;
    float sigma_q_x=50.0f;//越小相信模型
    float sigma_q_y=50.0f;
    float sigma_r_x=0.1f;//越小相信观测
    float sigma_r_y=0.1f;


    Kalman_filter_plus(pcl::PointXY &input,rclcpp::Time time) {
        predict_point = input;
        history.push_back(std::make_pair(GetTimeByRosTime(time), input));
        timer = std::chrono::steady_clock::now();
        color = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);
        int stateSize = 4;
        int measSize = 2;
        int contrSize = 0;
        unsigned int type = CV_32F;
        KF.init(stateSize, measSize, contrSize, type);

        // 状态矩阵：[x, vx, y, vy]
        cv::Mat state(stateSize, 1, type);
        // 测量矩阵：[z_x, z_y]
        cv::Mat meas(measSize, 1, type);
        meas.at<float>(0) = input.x;
        meas.at<float>(1) = input.y;

        state.at<float>(0) = meas.at<float>(0);
        state.at<float>(2) = meas.at<float>(1);
        KF.statePost = state;
        KF.transitionMatrix = (cv::Mat_<float>(4, 4) <<
        1, dt_, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, dt_,
        0, 0, 0, 1);//F矩阵

        KF.measurementMatrix = (cv::Mat_<float>(2, 4) <<
        1, 0, 0, 0,
        0, 0, 1, 0);//H矩阵
        
        KF.processNoiseCov = (cv::Mat_<float>(4, 4) <<
        sigma_q_x*pow(dt_, 3) / 3, sigma_q_x*pow(dt_, 2) / 2, 0,0,
        sigma_q_x*pow(dt_, 2) / 2, sigma_q_x*pow(dt_, 1), 0,0,
        0, 0, sigma_q_y*pow(dt_, 3) / 3, sigma_q_y*pow(dt_, 2) / 2,
        0, 0, sigma_q_y*pow(dt_, 2) / 2, sigma_q_y*pow(dt_, 1));

        //越大越相信卡尔曼的预测值，收敛速度越快
        //过程噪声Q

        KF.measurementNoiseCov = (cv::Mat_<float>(2, 2) <<
        sigma_r_x, 0,
        0, sigma_r_y);
        //越大越不相信卡尔曼预测值
        //观测噪声R
        setIdentity(KF.errorCovPost, cv::Scalar::all(1));
        has_updated = true;
    }

    ~Kalman_filter_plus() {}

    int get_color() {
        if(detect_history.size() == 0) {
            return 1;
        }
        //比较队列中的颜色，返回最多的颜色
        int red = 0;
        int blue= 0;
        for(auto &color : detect_history) {
            if(color.first == 0) {
                blue++;
            }
            else {
                red++;
            }
        }
        if(red > blue) {
            return 2;
        }
        else {
            return 0;
        }
    }

    int get_number() {
    int color = get_color();
    std::map<int, int> number_map;
    for(auto &number : detect_history) {
        if(number.first == color) {
            number_map[number.second]++;
        }
    }

    // 初始化最大计数和对应的数字
    int max_count = 0;
    int max_number = 0; // 假设-1为无效数字，或根据实际情况调整
    for(auto &entry : number_map) {
        if(entry.second > max_count) {
            max_count = entry.second;
            max_number = entry.first;
        }
    }
        return max_number; // 返回出现次数最多的数字
    }

    void update(pcl::PointXY &input, rclcpp::Time time) {
        cv::Mat meas = cv::Mat::zeros(2, 1, CV_32F);
        meas.at<float>(0) = input.x;
        meas.at<float>(1) = input.y;
        KF.correct(meas);
        predict_point.x = KF.statePost.at<float>(0);
        predict_point.y = KF.statePost.at<float>(2);
        has_updated = true;
        last_time = 0;
        auto temp_point = input;
        history.push_back(std::make_pair(GetTimeByRosTime(time), temp_point));
        if(history.size() > max_history){
            history.erase(history.begin());
        }
    }

    void update_predict_point() {//基于最后的点和速度方向和车的速度，以及预测下一个点
        dt_ = get_time();
        timer = std::chrono::steady_clock::now();
        auto result = KF.predict();
        last_time += dt_;
        predict_point.x = result.at<float>(0);
        predict_point.y = result.at<float>(2);
    }

    bool match(pcl::PointXY &input) {
        if(Distance(predict_point, input) < car_max_speed * dt_+detect_r){
            return true;
        }
        else {
            return false;
        }
    }

    void camera_match(rclcpp::Time &time, pcl::PointXY &input,int color,int number) {
        const double TIME_THRESHOLD = 1.0f;
        double input_time = GetTimeByRosTime(time);
        double differ_time = 1000;
        pcl::PointXY match_point;
        for(auto &point : history) {
            // std::cout<<"compare"<<point.first<<"and"<<input_time<<std::endl;
            auto differ = abs(point.first - input_time);
            // std::cout<<"differ"<<differ<<std::endl;
            if(differ < differ_time) {
                differ_time = differ;
                match_point = point.second;
            }
        }
        if(differ_time>TIME_THRESHOLD) {
            return ;
        }//首先找到离相机取帧时间戳最近的点

        
        if(Distance(match_point, input) < detect_r){
            std::cout<<"match success"<<std::endl;
            detect_history.push_back(std::make_pair(color, number));
            if(detect_history.size() > max_history){
                detect_history.erase(detect_history.begin());
            }
        }//如果距离小于检测半径，认为匹配成功
    }
    static double GetTimeByRosTime(
        rclcpp::Time& ros_time) {
        double ros_time_value =ros_time.nanoseconds()/1e9;
        // std::cout<<"ros_time_value"<<ros_time_value<<std::endl;
        return ros_time_value;
    }
}; 
