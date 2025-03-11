#ifndef obstacle_detector_HPP
#define obstacle_detector_HPP

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <sensor_msgs/msg/laser_scan.hpp>

class ObstacleDetector : public rclcpp::Node
{
  ObstacleDetector();
  void scan_obstacle(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

#endif  // b_obstacle_detector_hpp