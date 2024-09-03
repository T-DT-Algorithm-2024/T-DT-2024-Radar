#ifndef RADAR_RESOLVE_H
#define RADAR_RESOLVE_H

#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "radar_utils.h"
#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "opencv2/opencv.hpp"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "vision_interface/msg/detect_result.hpp"
#include "vision_interface/msg/radar2_sentry.hpp"
#include <vision_interface/msg/match_info.hpp>
namespace tdt_radar {

class Resolve final : public rclcpp::Node {
 public:
  explicit Resolve(const rclcpp::NodeOptions& options);
  void callback(const std::shared_ptr<geometry_msgs::msg::Vector3> msg);
  void DetectCallback(const vision_interface::msg::DetectResult::SharedPtr msg);
  void MatchInfoCallback(const vision_interface::msg::MatchInfo::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr point_sub;
  rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr detect_sub;
  rclcpp::Subscription<vision_interface::msg::MatchInfo>::SharedPtr match_info_sub;
  parser* parser_;
  int EnemyColor=1;
  cv::Mat minimap;
  int markers[6];
  int16_t match_time;
  uint8_t robot_hp[16];

 private:
  // pub PointXYZRGBA
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
  rclcpp::Publisher<vision_interface::msg::DetectResult>::SharedPtr pub_radar;
};

class map_car{
  public:
  float x;
  float y;
  int id;
};
}  // namespace tdt_radar

#endif