#ifndef obstacle_detector_HPP
#define obstacle_detector_HPP

#include <chrono>
#include <cmath>  // M_PIやsin, cosを使うのに必要
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>  // SharedPtr用
#include <optional>  // has_value用
#include <sensor_msgs/msg/laser_scan.hpp>

class ObstacleDetector : public rclcpp::Node
{
  ObstacleDetector();
  void process();

  // コールバック関数
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  
  // 関数
  void scan_obstacle();
  bool is_ignore_scan();

  // 変数
  std::optional<sensor_msgs::msg::LaserScan> laserscan_;

  // Pub & Sub
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr obstacle_points_pub_;
};

#endif  // b_obstacle_detector_hpp