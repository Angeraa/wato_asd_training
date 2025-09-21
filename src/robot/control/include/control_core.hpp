#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <optional>

namespace robot
{

class ControlCore {
public:
  // Initializes the ControlCore object with a ROS2 node
  ControlCore(rclcpp::Node* node);

private:
  // Callback for receiving and processing the path messages
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  // Callback for receiving and processing odometry messages
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Main control loop that runs periodically to compute and publish commands
  void controlLoop();

  // Finds the lookahead point on the path for the robot to follow
  std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();

  // Computes the distance between two points
  double computeDistance(const geometry_msgs::msg::Point &a,
                         const geometry_msgs::msg::Point &b);

  // Extracts the yaw from a quaternion
  double extractYaw(const geometry_msgs::msg::Quaternion &q);

  // Subscription to the path topic
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  // Subscription to the odometry topic
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publisher for velocity commands (cmd_vel)
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Timer for periodically triggering the control loop
  rclcpp::TimerBase::SharedPtr timer_;

  // Stores the current path received from the path topic
  nav_msgs::msg::Path::SharedPtr current_path_;

  // Stores the current odometry data of the robot
  nav_msgs::msg::Odometry::SharedPtr robot_odom_;

  // Control parameters
  double lookahead_distance_;
  double goal_tolerance_;
  double linear_speed_;

  rclcpp::Logger logger_;
};

}

#endif
