#include "path_tracker/trajectory_generator_node.hpp"
#include <cmath>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

TrajectoryGeneratorNode::TrajectoryGeneratorNode() 
  : Node("trajectory_generator_node"),
    trajectory_ready_(false)
{
  // Declare and get parameters
  this->declare_parameter("v_max", 0.5);        // m/s
  this->declare_parameter("a_max", 0.3);        // m/s²
  this->declare_parameter("a_max_decel", 0.4);  // m/s²
  this->declare_parameter("dt", 0.1);           // seconds

  v_max_ = this->get_parameter("v_max").as_double();
  a_max_ = this->get_parameter("a_max").as_double();
  a_max_decel_ = this->get_parameter("a_max_decel").as_double();
  dt_ = this->get_parameter("dt").as_double();

  // Subscribe to smoothed path
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/smoothed_path", 10,
    std::bind(&TrajectoryGeneratorNode::path_callback, this, std::placeholders::_1));

  // Publishers
  trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory", 10);
  trajectory_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/trajectory_odom", 10);

  // Timer for publishing trajectory visualization
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&TrajectoryGeneratorNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Trajectory Generator Node started");
  RCLCPP_INFO(this->get_logger(), "Parameters: v_max=%.2f m/s, a_max=%.2f m/s², dt=%.2f s",
              v_max_, a_max_, dt_);
}

void TrajectoryGeneratorNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Received path with less than 2 points");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Received path with %zu points, generating trajectory...",
              msg->poses.size());

  // Generate trajectory from path
  current_trajectory_ = generate_trajectory(*msg);
  trajectory_ready_ = true;

  RCLCPP_INFO(this->get_logger(), "Generated trajectory with %zu points, total time: %.2f s",
              current_trajectory_.size(),
              current_trajectory_.empty() ? 0.0 : current_trajectory_.back().t);
}

std::vector<TrajectoryPoint> TrajectoryGeneratorNode::generate_trajectory(
  const nav_msgs::msg::Path& path)
{
  std::vector<TrajectoryPoint> trajectory;

  if (path.poses.size() < 2) {
    return trajectory;
  }

  // Step 1: Calculate path lengths and heading angles
  std::vector<double> cumulative_distance;
  cumulative_distance.push_back(0.0);

  for (size_t i = 0; i < path.poses.size(); ++i) {
    TrajectoryPoint pt;
    pt.x = path.poses[i].pose.position.x;
    pt.y = path.poses[i].pose.position.y;
    pt.t = 0.0;  // Will be assigned later
    pt.v = 0.0;  // Will be assigned later
    pt.curvature = 0.0;

    // Calculate heading angle
    if (i < path.poses.size() - 1) {
      double dx = path.poses[i + 1].pose.position.x - pt.x;
      double dy = path.poses[i + 1].pose.position.y - pt.y;
      pt.yaw = std::atan2(dy, dx);

      // Calculate cumulative distance
      double dist = std::sqrt(dx * dx + dy * dy);
      cumulative_distance.push_back(cumulative_distance.back() + dist);
    } else {
      // Last point: use previous heading
      pt.yaw = trajectory.back().yaw;
      cumulative_distance.push_back(cumulative_distance.back());
    }

    trajectory.push_back(pt);
  }

  // Step 2: Calculate curvature
  calculate_curvature(trajectory);

  // Step 3: Apply velocity profile (trapezoidal)
  apply_trapezoidal_profile(trajectory, v_max_, a_max_, a_max_decel_);

  return trajectory;
}

void TrajectoryGeneratorNode::calculate_curvature(std::vector<TrajectoryPoint>& trajectory)
{
  if (trajectory.size() < 3) {
    return;
  }

  // Use finite differences to calculate curvature
  for (size_t i = 1; i < trajectory.size() - 1; ++i) {
    double dx1 = trajectory[i].x - trajectory[i - 1].x;
    double dy1 = trajectory[i].y - trajectory[i - 1].y;
    double dx2 = trajectory[i + 1].x - trajectory[i].x;
    double dy2 = trajectory[i + 1].y - trajectory[i].y;

    double ds1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    double ds2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

    if (ds1 < 1e-6 || ds2 < 1e-6) {
      trajectory[i].curvature = 0.0;
      continue;
    }

    // Calculate change in heading angle
    double theta1 = std::atan2(dy1, dx1);
    double theta2 = std::atan2(dy2, dx2);
    double dtheta = theta2 - theta1;

    // Normalize angle difference to [-pi, pi]
    while (dtheta > M_PI) dtheta -= 2 * M_PI;
    while (dtheta < -M_PI) dtheta += 2 * M_PI;

    double ds_avg = (ds1 + ds2) / 2.0;
    trajectory[i].curvature = dtheta / ds_avg;
  }

  // Boundary points
  trajectory[0].curvature = trajectory[1].curvature;
  trajectory.back().curvature = trajectory[trajectory.size() - 2].curvature;
}

void TrajectoryGeneratorNode::apply_trapezoidal_profile(
  std::vector<TrajectoryPoint>& trajectory,
  double v_max,
  double a_max,
  double a_max_decel)
{
  if (trajectory.size() < 2) {
    return;
  }

  // Step 1: Calculate distances between consecutive points
  std::vector<double> distances;
  double total_distance = 0.0;

  for (size_t i = 0; i < trajectory.size() - 1; ++i) {
    double dx = trajectory[i + 1].x - trajectory[i].x;
    double dy = trajectory[i + 1].y - trajectory[i].y;
    double dist = std::sqrt(dx * dx + dy * dy);
    distances.push_back(dist);
    total_distance += dist;
  }

  // Step 2: Forward pass - apply acceleration limit
  trajectory[0].v = 0.0;  // Start from rest
  trajectory[0].t = 0.0;

  for (size_t i = 0; i < trajectory.size() - 1; ++i) {
    double ds = distances[i];
    
    // Velocity limit based on curvature (slower on curves)
    double v_curve_limit = v_max;
    if (std::abs(trajectory[i].curvature) > 1e-3) {
      v_curve_limit = std::min(v_max, std::sqrt(0.5 / std::abs(trajectory[i].curvature)));
    }

    // Apply acceleration constraint
    double v_next_max = std::sqrt(trajectory[i].v * trajectory[i].v + 2 * a_max * ds);
    v_next_max = std::min(v_next_max, v_curve_limit);

    trajectory[i + 1].v = v_next_max;
  }

  // Step 3: Backward pass - apply deceleration limit and ensure we stop at the end
  trajectory.back().v = 0.0;  // End at rest

  for (int i = trajectory.size() - 2; i >= 0; --i) {
    double ds = distances[i];
    
    // Apply deceleration constraint
    double v_max_from_next = std::sqrt(trajectory[i + 1].v * trajectory[i + 1].v + 2 * a_max_decel * ds);
    trajectory[i].v = std::min(trajectory[i].v, v_max_from_next);
  }

  // Step 4: Calculate time stamps based on velocities
  for (size_t i = 0; i < trajectory.size() - 1; ++i) {
    double ds = distances[i];
    double v_avg = (trajectory[i].v + trajectory[i + 1].v) / 2.0;
    
    if (v_avg < 1e-6) {
      v_avg = 0.01;  // Prevent division by zero
    }
    
    double dt = ds / v_avg;
    trajectory[i + 1].t = trajectory[i].t + dt;
  }

  auto max_v_it = std::max_element(trajectory.begin(), trajectory.end(),
    [](const TrajectoryPoint& a, const TrajectoryPoint& b) { return a.v < b.v; });
  double max_v = (max_v_it != trajectory.end()) ? max_v_it->v : 0.0;
  
  RCLCPP_INFO(this->get_logger(), "Trajectory stats: max_v=%.2f m/s, total_time=%.2f s, distance=%.2f m",
              max_v, trajectory.back().t, total_distance);
}

void TrajectoryGeneratorNode::timer_callback()
{
  if (!trajectory_ready_ || current_trajectory_.empty()) {
    return;
  }

  // Publish trajectory as Path for visualization
  auto path_msg = trajectory_to_path(current_trajectory_);
  trajectory_pub_->publish(path_msg);

  // Publish first point as odometry (for debugging/visualization)
  publish_trajectory_odom();
}

nav_msgs::msg::Path TrajectoryGeneratorNode::trajectory_to_path(
  const std::vector<TrajectoryPoint>& trajectory)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp = this->now();

  for (const auto& pt : trajectory) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.position.x = pt.x;
    pose.pose.position.y = pt.y;
    pose.pose.position.z = 0.0;

    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, pt.yaw);
    pose.pose.orientation = tf2::toMsg(q);

    path_msg.poses.push_back(pose);
  }

  return path_msg;
}

void TrajectoryGeneratorNode::publish_trajectory_odom()
{
  if (current_trajectory_.empty()) {
    return;
  }

  // Publish a sample odometry message for visualization
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "map";
  odom.header.stamp = this->now();
  odom.child_frame_id = "base_link";

  const auto& pt = current_trajectory_[0];
  odom.pose.pose.position.x = pt.x;
  odom.pose.pose.position.y = pt.y;
  odom.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, pt.yaw);
  odom.pose.pose.orientation = tf2::toMsg(q);

  odom.twist.twist.linear.x = pt.v;
  odom.twist.twist.angular.z = pt.curvature * pt.v;

  trajectory_odom_pub_->publish(odom);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}