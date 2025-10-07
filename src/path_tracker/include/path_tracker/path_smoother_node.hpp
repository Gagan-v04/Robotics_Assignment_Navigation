#ifndef PATH_SMOOTHER_NODE_HPP_
#define PATH_SMOOTHER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>

class PathSmootherNode : public rclcpp::Node
{
public:
  PathSmootherNode();

private:
  void publish_path_once();

  /**
   * @brief Generates a smooth path using Cubic Spline interpolation.
   * @param waypoints The input vector of coarse 2D waypoints.
   * @param num_points The number of points to generate for the smoothed path.
   * @return A nav_msgs::msg::Path containing the dense, smoothed path.
   */
  nav_msgs::msg::Path generate_spline_path(const std::vector<geometry_msgs::msg::Point>& waypoints, int num_points);

  // ROS 2 components
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr original_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_pub_;

  // Member variables
  std::vector<geometry_msgs::msg::Point> coarse_waypoints_;
  nav_msgs::msg::Path original_path_msg_;
  bool path_published_;  // Flag to ensure single publish
};

#endif // PATH_SMOOTHER_NODE_HPP_