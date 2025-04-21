#ifndef OBSTACLE_DETECTOR_HPP
#define OBSTACLE_DETECTOR_HPP

#include <chrono>
#include <cmath>  // M_PIやsin, cosを使うのに必要
#include <rclcpp/rclcpp.hpp>
#include <functional>  // bind & placeholders
#include <memory>  // SharedPtr用
#include <optional>  // has_value用
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

class ObstacleDetector : public rclcpp::Node
{
  public:
      ObstacleDetector();
      void process();

      // コールバック関数
      void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  
      // 関数
      void scan_obstacle();
      bool is_ignore_scan(float distance, float angle);
  
  private:
      // 変数
      int hz_;
      int laser_step_;
      std::string robot_frame_;
      float ignore_dist_;
      std::optional<sensor_msgs::msg::LaserScan> laserscan_;

      // Pub & Sub
      rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
      rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_array_pub_;
};

#endif  // b_obstacle_detector_hpp