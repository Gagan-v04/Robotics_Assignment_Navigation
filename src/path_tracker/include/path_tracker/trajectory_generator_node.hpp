#ifndef TRAJECTORY_GENERATOR_NODE_HPP_
#define TRAJECTORY_GENERATOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>

/**
 * @brief Structure to hold a time-stamped trajectory point
 */
struct TrajectoryPoint
{
  double x;
  double y;
  double t;           // Time in seconds
  double v;           // Linear velocity (m/s)
  double yaw;         // Heading angle (radians)
  double curvature;   // Path curvature (1/m)
};

/**
 * @brief Node that generates time-parameterized trajectories from smoothed paths
 * 
 * This node subscribes to the smoothed path and generates a trajectory with:
 * - Time stamps
 * - Velocity profiles (trapezoidal)
 * - Heading angles
 * - Curvature information
 */
class TrajectoryGeneratorNode : public rclcpp::Node
{
public:
  TrajectoryGeneratorNode();

private:
  /**
   * @brief Callback for receiving smoothed path
   */
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);

  /**
   * @brief Generate time-parameterized trajectory from path
   * @param path Input smoothed path
   * @return Vector of trajectory points with time stamps and velocities
   */
  std::vector<TrajectoryPoint> generate_trajectory(const nav_msgs::msg::Path& path);

  /**
   * @brief Apply trapezoidal velocity profile to trajectory
   * @param trajectory Input trajectory points
   * @param v_max Maximum velocity (m/s)
   * @param a_max Maximum acceleration (m/s²)
   * @param a_max_decel Maximum deceleration (m/s²)
   */
  void apply_trapezoidal_profile(
    std::vector<TrajectoryPoint>& trajectory,
    double v_max,
    double a_max,
    double a_max_decel);

  /**
   * @brief Calculate curvature at each point in the trajectory
   * @param trajectory Input/output trajectory points
   */
  void calculate_curvature(std::vector<TrajectoryPoint>& trajectory);

  /**
   * @brief Timer callback to publish trajectory for visualization
   */
  void timer_callback();

  /**
   * @brief Convert trajectory to Path message for visualization
   */
  nav_msgs::msg::Path trajectory_to_path(const std::vector<TrajectoryPoint>& trajectory);

  /**
   * @brief Publish trajectory as Odometry messages for visualization
   */
  void publish_trajectory_odom();

  // ROS 2 components
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr trajectory_odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Trajectory parameters
  double v_max_;          // Maximum velocity (m/s)
  double a_max_;          // Maximum acceleration (m/s²)
  double a_max_decel_;    // Maximum deceleration (m/s²)
  double dt_;             // Time step for trajectory sampling (s)

  // Stored trajectory
  std::vector<TrajectoryPoint> current_trajectory_;
  bool trajectory_ready_;
};

#endif // TRAJECTORY_GENERATOR_NODE_HPP_