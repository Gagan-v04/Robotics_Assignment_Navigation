#ifndef TRAJECTORY_TRACKER_NODE_HPP_
#define TRAJECTORY_TRACKER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/float64.hpp" // +++ RQT PLOT INTEGRATION +++
#include <vector>
#include <cmath>

struct RobotState
{
  double x;
  double y;
  double theta;
  double v;
  double omega;
};

struct TrajectoryPoint
{
  double x;
  double y;
  double t;
  double v;
  double yaw;
};

class TrajectoryTrackerNode : public rclcpp::Node
{
public:
  TrajectoryTrackerNode();

private:
  void trajectory_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void control_loop();
  double pure_pursuit_control(const RobotState& current_state, const TrajectoryPoint& lookahead_point);
  TrajectoryPoint find_lookahead_point(const RobotState& current_state);
  size_t find_closest_point(const RobotState& current_state);
  void simulate_robot(double v_cmd, double omega_cmd, double dt);
  void publish_odometry();
  void publish_markers();
  bool is_tracking_complete();
  void calculate_tracking_errors();
  double normalize_angle(double angle);

  // ROS 2 components
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // +++ RQT PLOT INTEGRATION +++
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr accel_pub_;

  // Controller parameters
  double lookahead_distance_;
  double max_linear_vel_;
  double max_angular_vel_;
  double position_tolerance_;
  double control_frequency_;
  double k_v_;
  double k_omega_;
  bool use_gazebo_odom_;

  // Robot state
  RobotState robot_state_;
  bool trajectory_received_;
  std::vector<TrajectoryPoint> trajectory_;
  size_t current_trajectory_idx_;
  
  // +++ RQT PLOT INTEGRATION +++
  double prev_linear_velocity_;

  // Tracking metrics
  double total_path_error_;
  double max_path_error_;
  size_t error_sample_count_;
  rclcpp::Time start_time_;
  bool tracking_started_;
  bool tracking_complete_;

  // Simulation time
  double sim_time_;
};

#endif // TRAJECTORY_TRACKER_NODE_HPP_