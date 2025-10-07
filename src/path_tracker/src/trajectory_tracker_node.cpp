#include "path_tracker/trajectory_tracker_node.hpp"
#include "std_msgs/msg/float64.hpp" // +++ RQT PLOT INTEGRATION +++
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <limits>
#include <algorithm>

TrajectoryTrackerNode::TrajectoryTrackerNode()
  : Node("trajectory_tracker_node"),
    trajectory_received_(false),
    current_trajectory_idx_(0),
    total_path_error_(0.0),
    max_path_error_(0.0),
    error_sample_count_(0),
    tracking_started_(false),
    tracking_complete_(false),
    sim_time_(0.0),
    prev_linear_velocity_(0.0) // +++ RQT PLOT INTEGRATION +++
{
  this->declare_parameter("lookahead_distance", 0.5);
  this->declare_parameter("max_linear_vel", 0.5);
  this->declare_parameter("max_angular_vel", 1.0);
  this->declare_parameter("position_tolerance", 0.15);
  this->declare_parameter("control_frequency", 50.0);
  this->declare_parameter("k_v", 1.0);
  this->declare_parameter("k_omega", 2.0);
  this->declare_parameter("start_x", 0.0);
  this->declare_parameter("start_y", 0.0);
  this->declare_parameter("start_theta", 0.0);
  this->declare_parameter("use_gazebo_odom", false);

  lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
  max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
  max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
  position_tolerance_ = this->get_parameter("position_tolerance").as_double();
  control_frequency_ = this->get_parameter("control_frequency").as_double();
  k_v_ = this->get_parameter("k_v").as_double();
  k_omega_ = this->get_parameter("k_omega").as_double();
  use_gazebo_odom_ = this->get_parameter("use_gazebo_odom").as_bool();

  robot_state_.x = this->get_parameter("start_x").as_double();
  robot_state_.y = this->get_parameter("start_y").as_double();
  robot_state_.theta = this->get_parameter("start_theta").as_double();
  robot_state_.v = 0.0;
  robot_state_.omega = 0.0;

  trajectory_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/trajectory", 10,
    std::bind(&TrajectoryTrackerNode::trajectory_callback, this, std::placeholders::_1));

  if (use_gazebo_odom_) {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&TrajectoryTrackerNode::odom_callback, this, std::placeholders::_1));
  }
  
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot_odom", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/tracking_marker", 10);
  target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/lookahead_target", 10);

  // +++ RQT PLOT INTEGRATION +++
  accel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/robot_acceleration", 10);

  auto period = std::chrono::duration<double>(1.0 / control_frequency_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&TrajectoryTrackerNode::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "Trajectory Tracker Node started");
}

void TrajectoryTrackerNode::trajectory_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (tracking_complete_) {
    RCLCPP_DEBUG(this->get_logger(), "Tracking already complete. Ignoring new trajectory.");
    return;
  }
  if (msg->poses.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Received trajectory with less than 2 points");
    return;
  }
  if (trajectory_received_) {
    RCLCPP_DEBUG(this->get_logger(), "Already have a trajectory. Ignoring new one.");
    return;
  }

  trajectory_.clear();
  for (const auto& pose : msg->poses) {
    TrajectoryPoint pt;
    pt.x = pose.pose.position.x;
    pt.y = pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(pose.pose.orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, pt.yaw);
    pt.v = max_linear_vel_;
    trajectory_.push_back(pt);
  }

  trajectory_received_ = true;
  current_trajectory_idx_ = 0;
  tracking_started_ = false;
  tracking_complete_ = false;
  total_path_error_ = 0.0;
  max_path_error_ = 0.0;
  error_sample_count_ = 0;
  sim_time_ = 0.0;
  prev_linear_velocity_ = 0.0; // Reset for new trajectory

  RCLCPP_INFO(this->get_logger(), "Received trajectory with %zu points", trajectory_.size());
}

void TrajectoryTrackerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_state_.x = msg->pose.pose.position.x;
  robot_state_.y = msg->pose.pose.position.y;
  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  tf2::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, robot_state_.theta);
  robot_state_.v = msg->twist.twist.linear.x;
  robot_state_.omega = msg->twist.twist.angular.z;
}

void TrajectoryTrackerNode::control_loop()
{
  if (!trajectory_received_ || trajectory_.empty() || tracking_complete_) {
    return;
  }

  if (!tracking_started_) {
    tracking_started_ = true;
    start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Starting trajectory tracking!");
  }

  if (is_tracking_complete()) {
    tracking_complete_ = true;
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
    control_timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "===========================================");
    RCLCPP_INFO(this->get_logger(), "Trajectory tracking complete!");
    if (error_sample_count_ > 0) {
        RCLCPP_INFO(this->get_logger(), "Average tracking error: %.3f m", total_path_error_ / error_sample_count_);
    }
    RCLCPP_INFO(this->get_logger(), "Maximum tracking error: %.3f m", max_path_error_);
    RCLCPP_INFO(this->get_logger(), "Total time: %.2f s", sim_time_);
    RCLCPP_INFO(this->get_logger(), "Robot has returned to start position!");
    RCLCPP_INFO(this->get_logger(), "===========================================");
    return;
  }

  TrajectoryPoint lookahead_point = find_lookahead_point(robot_state_);
  double omega_cmd = pure_pursuit_control(robot_state_, lookahead_point);
  
  double dist_to_goal = std::sqrt(std::pow(trajectory_.back().x - robot_state_.x, 2) + std::pow(trajectory_.back().y - robot_state_.y, 2));
  double v_cmd = max_linear_vel_;
  if (dist_to_goal < 1.0) {
    v_cmd = max_linear_vel_ * dist_to_goal;
    v_cmd = std::max(v_cmd, 0.05);
  }

  v_cmd = std::clamp(v_cmd, 0.0, max_linear_vel_);
  omega_cmd = std::clamp(omega_cmd, -max_angular_vel_, max_angular_vel_);

  if (!use_gazebo_odom_) {
    simulate_robot(v_cmd, omega_cmd, 1.0 / control_frequency_);
  }
  sim_time_ += 1.0 / control_frequency_;

  // This is placed after the state update (either from odom_callback or simulate_robot)
  double dt = 1.0 / control_frequency_;
  if (dt > 1e-9) {
      double current_linear_velocity = robot_state_.v;
      double linear_acceleration = (current_linear_velocity - prev_linear_velocity_) / dt;
      
      auto accel_msg = std_msgs::msg::Float64();
      accel_msg.data = linear_acceleration;
      accel_pub_->publish(accel_msg);

      prev_linear_velocity_ = current_linear_velocity;
  }

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = v_cmd;
  cmd_vel.angular.z = omega_cmd;
  cmd_vel_pub_->publish(cmd_vel);

  publish_odometry();
  publish_markers();

  geometry_msgs::msg::PoseStamped target_msg;
  target_msg.header.frame_id = "map";
  target_msg.header.stamp = this->now();
  target_msg.pose.position.x = lookahead_point.x;
  target_msg.pose.position.y = lookahead_point.y;
  target_pub_->publish(target_msg);
  
  calculate_tracking_errors();
}

double TrajectoryTrackerNode::pure_pursuit_control(const RobotState& current_state, const TrajectoryPoint& lookahead_point)
{
  double dx = lookahead_point.x - current_state.x;
  double dy = lookahead_point.y - current_state.y;
  double alpha = std::atan2(dy, dx) - current_state.theta;
  alpha = normalize_angle(alpha);
  double ld = std::sqrt(dx * dx + dy * dy);
  if (ld < 0.01) { return 0.0; }
  double curvature = 2.0 * std::sin(alpha) / ld;
  double current_v = std::max(0.1, robot_state_.v);
  double omega = k_omega_ * curvature * current_v;
  return omega;
}

TrajectoryPoint TrajectoryTrackerNode::find_lookahead_point(const RobotState& current_state)
{
  size_t closest_idx = find_closest_point(current_state);
  for (size_t i = closest_idx; i < trajectory_.size(); ++i) {
    double dist = std::sqrt(std::pow(trajectory_[i].x - current_state.x, 2) + std::pow(trajectory_[i].y - current_state.y, 2));
    if (dist >= lookahead_distance_) {
      current_trajectory_idx_ = i;
      return trajectory_[i];
    }
  }
  current_trajectory_idx_ = trajectory_.size() - 1;
  return trajectory_.back();
}

size_t TrajectoryTrackerNode::find_closest_point(const RobotState& current_state)
{
  double min_dist_sq = std::numeric_limits<double>::max();
  size_t closest_idx = current_trajectory_idx_;
  size_t search_start = current_trajectory_idx_;
  size_t search_end = std::min(current_trajectory_idx_ + 50, trajectory_.size());
  for (size_t i = search_start; i < search_end; ++i) {
    double dx = trajectory_[i].x - current_state.x;
    double dy = trajectory_[i].y - current_state.y;
    double dist_sq = dx * dx + dy * dy;
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_idx = i;
    }
  }
  return closest_idx;
}

void TrajectoryTrackerNode::simulate_robot(double v_cmd, double omega_cmd, double dt)
{
  robot_state_.v = v_cmd;
  robot_state_.omega = omega_cmd;
  robot_state_.x += v_cmd * std::cos(robot_state_.theta) * dt;
  robot_state_.y += v_cmd * std::sin(robot_state_.theta) * dt;
  robot_state_.theta += omega_cmd * dt;
  robot_state_.theta = normalize_angle(robot_state_.theta);
}

void TrajectoryTrackerNode::publish_odometry()
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "map";
  odom.header.stamp = this->now();
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = robot_state_.x;
  odom.pose.pose.position.y = robot_state_.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, robot_state_.theta);
  odom.pose.pose.orientation = tf2::toMsg(q);
  odom.twist.twist.linear.x = robot_state_.v;
  odom.twist.twist.angular.z = robot_state_.omega;
  odom_pub_->publish(odom);
}

void TrajectoryTrackerNode::publish_markers()
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = "robot_position";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = robot_state_.x;
  marker.pose.position.y = robot_state_.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, robot_state_.theta);
  marker.pose.orientation = tf2::toMsg(q);
  marker.scale.x = 0.3;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker_pub_->publish(marker);
}

bool TrajectoryTrackerNode::is_tracking_complete()
{
  if (trajectory_.empty()) return false;

  double dist_to_final = std::sqrt(std::pow(trajectory_.back().x - robot_state_.x, 2) + std::pow(trajectory_.back().y - robot_state_.y, 2));
  bool near_final_waypoint = dist_to_final < position_tolerance_;
  bool progressed_through_path = current_trajectory_idx_ >= (trajectory_.size() * 0.95);
  bool sufficient_time_elapsed = sim_time_ > 2.0;

  return near_final_waypoint && progressed_through_path && sufficient_time_elapsed;
}

void TrajectoryTrackerNode::calculate_tracking_errors()
{
  if (trajectory_.empty()) return;
  size_t closest_idx = find_closest_point(robot_state_);
  double dx = trajectory_[closest_idx].x - robot_state_.x;
  double dy = trajectory_[closest_idx].y - robot_state_.y;
  double error = std::sqrt(dx * dx + dy * dy);
  total_path_error_ += error;
  error_sample_count_++;
  if (error > max_path_error_) {
    max_path_error_ = error;
  }
}

double TrajectoryTrackerNode::normalize_angle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryTrackerNode>());
  rclcpp::shutdown();
  return 0;
}