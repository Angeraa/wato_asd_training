#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
 
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
 
  private:
    // Callbacks
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    // Helpers
    void initializeDistanceCache();
    void clearCostmap();
    void inflateObstacles(int radius);
    void publishCostmap();

    // Data
    std::vector<std::vector<int>> costmap_data_;
    std::vector<std::vector<int>> inflated_map_;
    std::vector<std::vector<double>> distance_cache_;

    // Parameters
    double resolution_;
    double inverse_resolution_;
    double size_m_;
    int inflation_radius_;
    int width_, height_;
    double update_rate_;
    
    robot::CostmapCore costmap_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid grid_msg_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    std::mutex scan_mutex_;  // Protect last_scan_ from race conditions
};
 
#endif