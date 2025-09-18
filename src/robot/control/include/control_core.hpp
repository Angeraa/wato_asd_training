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
  ControlCore(rclcpp::Node* node);

private:
  // Callbacks
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlLoop();

  // Helpers
  std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();
  double computeDistance(const geometry_msgs::msg::Point &a,
                         const geometry_msgs::msg::Point &b);
  double extractYaw(const geometry_msgs::msg::Quaternion &q);

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Data
  nav_msgs::msg::Path::SharedPtr current_path_;
  nav_msgs::msg::Odometry::SharedPtr robot_odom_;

  // Parameters
  double lookahead_distance_;
  double goal_tolerance_;
  double linear_speed_;

  rclcpp::Logger logger_;
};

}

#endif
