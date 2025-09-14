#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
 
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
 
  private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void inflateObstacles(int radius);

    void publishCostmap();


    std::vector<std::vector<int>> costmap_data_;

    double resolution_;
    double size_m_;
    int width_, height_;

    robot::CostmapCore costmap_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    nav_msgs::msg::OccupancyGrid grid_msg_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

    rclcpp::TimerBase::SharedPtr timer_;
};
 
#endif