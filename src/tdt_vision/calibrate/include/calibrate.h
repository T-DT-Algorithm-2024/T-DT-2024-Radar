#ifndef RADAR_CALIBRATE_H
#define RADAR_CALIBRATE_H

#include <memory>
#include <opencv2/photo.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "radar_utils.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace tdt_radar {
    static cv::Mat cvimage_;
    static std::vector<cv::Point2f> pick_points;
    static int EnemyColor;//蓝色0 红色2 按BGR来
    static cv::Mat camera_matrix;
    static cv::Mat dist_coeffs;
    static cv::Mat rvec;
    static cv::Mat tvec;
    static bool is_calibrating = false;


    class Calibrate final : public rclcpp::Node {
    public:
        std::vector<cv::Point3f> real_points;
        cv::Point3f self_R0TL   = cv::Point3f(8.67 , -5.715, 0.120 + 0.3);
        cv::Point3f self_R0TR   = cv::Point3f(8.67 , -5.715 - 0.4, 0.120 + 0.3);
        cv::Point3f self_Tower = cv::Point3f(11.1865, -12.419, 1.003+0.118);
        cv::Point3f enemy_Base  = cv::Point3f(26.153, -7.5, 1.043+0.2);
        cv::Point3f enemy_Tower =cv::Point3f(16.64, -2.4215, 1.331+0.118);
        //切记前哨站和基地有底座
        
        void change_outmatrix(double x,double y,double z);

        void publish_tf();
        
        explicit Calibrate(const rclcpp::NodeOptions &options);

        void callback(const sensor_msgs::msg::Image::SharedPtr msg);

        void compressed_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

        void ChessboardCallback(const sensor_msgs::msg::Image::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub;

        void solve();

        parser *parser_;

        std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

        geometry_msgs::msg::TransformStamped transformStamped;

    };
    void mousecallback(int event, int x, int y, int flags, void *userdata);

}  // namespace tdt_radar
#endif