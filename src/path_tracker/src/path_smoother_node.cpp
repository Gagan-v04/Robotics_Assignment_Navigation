#include "path_tracker/path_smoother_node.hpp"
#include <Eigen/Dense>
#include <cmath>

// Constructor for the PathSmootherNode class
PathSmootherNode::PathSmootherNode() : Node("path_smoother_node"), path_published_(false)
{
  // This makes the publishers "latch" the last message for late subscribers like RViz.
  rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
  qos_profile.transient_local();

  // +++ FIX: Apply the new QoS profile to the publishers +++
  original_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/original_path", qos_profile);
  smoothed_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/smoothed_path", qos_profile);

  // --- Load waypoints from ROS2 parameters ---
  this->declare_parameter<std::vector<double>>("waypoints", std::vector<double>());
  std::vector<double> waypoints_flat = this->get_parameter("waypoints").as_double_array();

  if (waypoints_flat.size() % 2 != 0) {
    RCLCPP_ERROR(this->get_logger(), "Waypoints parameter has an odd number of values. Must be pairs of [x, y].");
    return;
  }

  for (size_t i = 0; i < waypoints_flat.size(); i += 2) {
    geometry_msgs::msg::Point pt;
    pt.x = waypoints_flat[i];
    pt.y = waypoints_flat[i+1];
    pt.z = 0.0;
    coarse_waypoints_.push_back(pt);
  }

  if (coarse_waypoints_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No waypoints were provided. The robot will not move.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu waypoints.", coarse_waypoints_.size());
  }
  
  // Convert coarse waypoints to a Path message for visualization
  original_path_msg_.header.frame_id = "map";
  for (const auto& waypoint : coarse_waypoints_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position = waypoint;
    original_path_msg_.poses.push_back(pose);
  }

  // Use a one-shot timer to publish the path only once after a short delay
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&PathSmootherNode::publish_path_once, this));
  
  RCLCPP_INFO(this->get_logger(), "Path Smoother Node initialized. Will publish path once after 1 second.");
}

void PathSmootherNode::publish_path_once()
{
  // Double-check with a member variable to ensure we never publish twice
  if (path_published_) {
    return;
  }
  
  path_published_ = true;
  
  // Set the timestamp for the path messages
  original_path_msg_.header.stamp = this->now();
  
  // Generate the smoothed path from the coarse waypoints
  nav_msgs::msg::Path smoothed_path = generate_spline_path(coarse_waypoints_, 100);
  smoothed_path.header.stamp = this->now();

  // Publish both the original and smoothed paths
  original_path_pub_->publish(original_path_msg_);
  smoothed_path_pub_->publish(smoothed_path);

  RCLCPP_INFO(this->get_logger(), "Path published successfully. This will only happen once.");
  
  // Cancel the timer to ensure it never fires again
  timer_->cancel();
  timer_.reset();  // Release the timer resources
}

// Core function to generate the cubic spline path using natural cubic splines
nav_msgs::msg::Path PathSmootherNode::generate_spline_path(
    const std::vector<geometry_msgs::msg::Point>& waypoints, int num_points)
{
  nav_msgs::msg::Path smoothed_path;
  smoothed_path.header.frame_id = "map";

  if (waypoints.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Not enough waypoints for spline generation (need at least 2).");
    return smoothed_path;
  }

  // Filter out duplicate or near-duplicate waypoints
  const double MIN_DISTANCE = 1e-6;
  std::vector<geometry_msgs::msg::Point> filtered_waypoints;
  filtered_waypoints.push_back(waypoints[0]);
  
  for (size_t i = 1; i < waypoints.size(); ++i) {
    double dist = std::sqrt(
      std::pow(waypoints[i].x - filtered_waypoints.back().x, 2) + 
      std::pow(waypoints[i].y - filtered_waypoints.back().y, 2)
    );
    if (dist > MIN_DISTANCE) {
      filtered_waypoints.push_back(waypoints[i]);
    }
  }

  if (filtered_waypoints.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "After filtering, not enough unique waypoints remain.");
    return smoothed_path;
  }

  // Special case: if only 2 waypoints, use linear interpolation
  if (filtered_waypoints.size() == 2) {
    for (int i = 0; i < num_points; ++i) {
      double t = static_cast<double>(i) / (num_points - 1);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = filtered_waypoints[0].x + t * (filtered_waypoints[1].x - filtered_waypoints[0].x);
      pose.pose.position.y = filtered_waypoints[0].y + t * (filtered_waypoints[1].y - filtered_waypoints[0].y);
      pose.pose.position.z = 0;
      smoothed_path.poses.push_back(pose);
    }
    return smoothed_path;
  }

  int n = filtered_waypoints.size();
  
  // Extract x and y coordinates
  std::vector<double> x_vals(n), y_vals(n);
  for (int i = 0; i < n; ++i) {
    x_vals[i] = filtered_waypoints[i].x;
    y_vals[i] = filtered_waypoints[i].y;
  }

  // Calculate cumulative chord lengths as parameters
  std::vector<double> t(n);
  t[0] = 0.0;
  for (int i = 1; i < n; ++i) {
    double dx = x_vals[i] - x_vals[i-1];
    double dy = y_vals[i] - y_vals[i-1];
    t[i] = t[i-1] + std::sqrt(dx*dx + dy*dy);
  }
  double total_length = t[n-1];

  // Compute spline coefficients for x and y separately
  auto compute_spline_coeffs = [&](const std::vector<double>& vals) -> 
      std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>> {
    
    std::vector<double> a = vals;
    std::vector<double> b(n-1), d(n-1), c(n);
    
    // Solve for c using Thomas algorithm (tridiagonal matrix)
    std::vector<double> h(n-1);
    for (int i = 0; i < n-1; ++i) {
      h[i] = t[i+1] - t[i];
    }
    
    std::vector<double> alpha(n-1);
    for (int i = 1; i < n-1; ++i) {
      alpha[i] = 3.0/h[i] * (a[i+1] - a[i]) - 3.0/h[i-1] * (a[i] - a[i-1]);
    }
    
    std::vector<double> l(n), mu(n), z(n);
    l[0] = 1.0;
    mu[0] = 0.0;
    z[0] = 0.0;
    
    for (int i = 1; i < n-1; ++i) {
      l[i] = 2.0 * (t[i+1] - t[i-1]) - h[i-1] * mu[i-1];
      mu[i] = h[i] / l[i];
      z[i] = (alpha[i] - h[i-1] * z[i-1]) / l[i];
    }
    
    l[n-1] = 1.0;
    z[n-1] = 0.0;
    c[n-1] = 0.0;
    
    for (int j = n-2; j >= 0; --j) {
      c[j] = z[j] - mu[j] * c[j+1];
      b[j] = (a[j+1] - a[j]) / h[j] - h[j] * (c[j+1] + 2.0*c[j]) / 3.0;
      d[j] = (c[j+1] - c[j]) / (3.0 * h[j]);
    }
    
    return {a, b, c, d};
  };

  auto [a_x, b_x, c_x, d_x] = compute_spline_coeffs(x_vals);
  auto [a_y, b_y, c_y, d_y] = compute_spline_coeffs(y_vals);

  // Generate interpolated points
  for (int i = 0; i < num_points; ++i) {
    double param = (static_cast<double>(i) / (num_points - 1)) * total_length;
    
    // Find the segment
    int j = 0;
    for (int k = 0; k < n-1; ++k) {
      if (param <= t[k+1]) {
        j = k;
        break;
      }
      if (k == n-2) j = n-2;
    }
    
    double dt = param - t[j];
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = a_x[j] + b_x[j]*dt + c_x[j]*dt*dt + d_x[j]*dt*dt*dt;
    pose.pose.position.y = a_y[j] + b_y[j]*dt + c_y[j]*dt*dt + d_y[j]*dt*dt*dt;
    pose.pose.position.z = 0;
    smoothed_path.poses.push_back(pose);
  }

  return smoothed_path;
}

// Main function to spin up the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathSmootherNode>());
  rclcpp::shutdown();
  return 0;
}