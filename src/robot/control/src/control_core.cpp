#include "control_core.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;

namespace robot
{

ControlCore::ControlCore(rclcpp::Node* node)
  : logger_(node->get_logger())
{
  // Declare parameters with defaults
  node->declare_parameter("lookahead_distance", 1.0);
  node->declare_parameter("goal_tolerance", 0.2);
  node->declare_parameter("linear_speed", 0.5);

  // Retrieve values
  node->get_parameter("lookahead_distance", lookahead_distance_);
  node->get_parameter("goal_tolerance", goal_tolerance_);
  node->get_parameter("linear_speed", linear_speed_);

  // Subscribers
  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlCore::pathCallback, this, _1));

  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlCore::odomCallback, this, _1));

  // Publisher
  cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer to periodically call the control loop
  timer_ = node->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ControlCore::controlLoop, this));
}

// Callback to update the current path when a new path message is received
void ControlCore::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = msg;
}

// Callback to update the robot's odometry when a new odometry message is received
void ControlCore::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_odom_ = msg;
}

// Main control loop to compute and publish velocity commands
void ControlCore::controlLoop() {
  // Ensure both path and odometry data are available
  if (!current_path_ || !robot_odom_) return;

  // Find a lookahead point on the path
  auto target = findLookaheadPoint();
  if (!target) {
    RCLCPP_INFO(logger_, "No valid lookahead point found, stopping.");
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    return;
  }

  // Extract the robot's current position and orientation
  auto robot_pos = robot_odom_->pose.pose.position;
  double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

  // Extract the target position from the lookahead point
  auto target_pos = target->pose.position;

  // Compute the difference in position between the robot and the target
  double dx = target_pos.x - robot_pos.x;
  double dy = target_pos.y - robot_pos.y;

  // Transform the target point to the robot's local frame
  double local_x = std::cos(-robot_yaw) * dx - std::sin(-robot_yaw) * dy;
  double local_y = std::sin(-robot_yaw) * dx + std::cos(-robot_yaw) * dy;

  // Compute the curvature for Pure Pursuit control
  double curvature = (2.0 * local_y) / (lookahead_distance_ * lookahead_distance_);

  // Create and populate the velocity command message
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear_speed_;  
  cmd.angular.z = linear_speed_ * curvature;  

  // Check if the robot has reached the goal
  auto last = current_path_->poses.back().pose.position;
  double dist_to_goal = computeDistance(robot_pos, last);
  if (dist_to_goal < goal_tolerance_) {
    RCLCPP_INFO(logger_, "Goal reached, stopping robot.");
    cmd.linear.x = 0.0;  // Stop linear motion
    cmd.angular.z = 0.0; // Stop angular motion
  }

  // Publish the velocity command
  cmd_vel_pub_->publish(cmd);
}

// Finds the lookahead point on the path for the robot to follow
std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint() {
  if (!current_path_) return std::nullopt;  /
  if (current_path_->poses.empty()) return std::nullopt; 

  auto robot_pos = robot_odom_->pose.pose.position;

  // Iterate through the path to find the first point beyond the lookahead distance
  for (auto &pose : current_path_->poses) {
    double dist = computeDistance(robot_pos, pose.pose.position);
    if (dist >= lookahead_distance_) {
      return pose; // Return the first valid lookahead point
    }
  }

  return current_path_->poses.back();  // Fallback to the last pose if no valid point is found
}

// Computes the distance between two points
double ControlCore::computeDistance(const geometry_msgs::msg::Point &a,
                                    const geometry_msgs::msg::Point &b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

// Extracts the yaw from a quaternion
double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion &q) {
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);  // Convert quaternion to roll, pitch, and yaw
  return yaw;  
}
}
