#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

// PlannerNode: Implements a simple A*-based path planner as a ROS2 node
class PlannerNode : public rclcpp::Node
{
public:
  // Constructor: sets up subscribers, publisher, and timer
  PlannerNode();

private:
  // State machine for planner behavior
  enum class State
  {
    WAITING_FOR_GOAL,               // Waiting for a new goal
    WAITING_FOR_ROBOT_TO_REACH_GOAL // Waiting for robot to reach the goal
  };
  State state_; // Current state

  // ROS2 Subscribers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_msg_;      // Subscribes to /map
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_goal_; // Subscribes to /goal_point
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;          // Subscribes to /odom/filtered

  // ROS2 Publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; // Publishes to /path

  // Timer for periodic replanning and goal checking
  rclcpp::TimerBase::SharedPtr timer_;

  // Latest map, goal, and robot pose
  nav_msgs::msg::OccupancyGrid current_map_; // Latest occupancy grid
  geometry_msgs::msg::PointStamped goal_;    // Latest goal point
  geometry_msgs::msg::Pose robot_pose_;      // Latest robot pose
  bool goal_received_ = false;               // Flag: has a goal been received?

  // Callback for /map topic
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  // Callback for /goal_point topic
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  // Callback for /odom/filtered topic
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  // Timer callback for periodic checks
  void timerCallback();
  // Returns true if robot is close enough to the goal
  bool goalReached();
  // Plans a path using A* and publishes it
  void planPath();
};

#endif // PLANNER_NODE_HPP_
#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();

private:
  robot::PlannerCore planner_;
};

#endif
