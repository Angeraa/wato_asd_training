#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    // Callbacks
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Helpers
    void updateMap();
    void integrateCostmapIntoMap();


    robot::MapMemoryCore map_memory_;

    // Parameters
    nav_msgs::msg::OccupancyGrid global_map_;
    double last_x, last_y;
    double distance_threshold_;
    double size_m_;
    double resolution_;
    int width_, height_;

    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
    bool should_update_map_ = false;
    bool costmap_updated_ = false;
    
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex costmap_mutex_;  // Protect costmap
    std::mutex odom_mutex_; // Protect odometry
};

#endif 
