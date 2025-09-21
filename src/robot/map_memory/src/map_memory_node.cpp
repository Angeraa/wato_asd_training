#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())), last_x(0.0), last_y(0.0), distance_threshold(5.0)
{

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&MapMemoryNode::updateMap, this));

  resolution_ = 0.05;
  size_m_ = 50;
  width_ = static_cast<int>(size_m_ / resolution_);
  height_ = static_cast<int>(size_m_ / resolution_);

  global_map_.info.resolution = resolution_;
  global_map_.info.width = width_;
  global_map_.info.height = height_;
  global_map_.info.origin.position.x = -size_m_ / 2.0 + resolution_ / 2.0;
  global_map_.info.origin.position.y = -size_m_ / 2.0 + resolution_ / 2.0;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.header.frame_id = "sim_world";
  global_map_.data.resize(width_ * height_, 0);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = msg;

  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));

  if (distance >= distance_threshold)
  {
    last_x = x;
    last_y = y;
    should_update_map_ = true;
  }
}

void MapMemoryNode::updateMap()
{
  if (should_update_map_ && costmap_updated_)
  {
    integrateCostmapIntoMap();
    map_pub_->publish(global_map_);
    should_update_map_ = false;
  }
}

void MapMemoryNode::integrateCostmapIntoMap()
{
  if (!costmap_updated_ || !current_odom_)
  {
    RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for costmap and odometry data...");
    return;
  }

  double robot_x = current_odom_->pose.pose.position.x;
  double robot_y = current_odom_->pose.pose.position.y;
  auto &q = current_odom_->pose.pose.orientation;
  double yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  double costmap_origin_x = latest_costmap_.info.origin.position.x;
  double costmap_origin_y = latest_costmap_.info.origin.position.y;

  double costmap_origin_in_map_x = robot_x +
                                   (costmap_origin_x * cos(yaw) - costmap_origin_y * sin(yaw));
  double costmap_origin_in_map_y = robot_y +
                                   (costmap_origin_x * sin(yaw) + costmap_origin_y * cos(yaw));

  double global_origin_x = global_map_.info.origin.position.x;
  double global_origin_y = global_map_.info.origin.position.y;

  int global_start_x = static_cast<int>((costmap_origin_in_map_x - global_origin_x) / global_map_.info.resolution);
  int global_start_y = static_cast<int>((costmap_origin_in_map_y - global_origin_y) / global_map_.info.resolution);

  // Same resolutions for now but just in case
  double resolution_ratio = latest_costmap_.info.resolution / global_map_.info.resolution;

  for (int local_y = 0; local_y < static_cast<int>(latest_costmap_.info.height); ++local_y)
  {
    for (int local_x = 0; local_x < static_cast<int>(latest_costmap_.info.width); ++local_x)
    {

      double cell_x_robot = local_x * latest_costmap_.info.resolution + costmap_origin_x;
      double cell_y_robot = local_y * latest_costmap_.info.resolution + costmap_origin_y;

      double cell_x_map = robot_x + (cell_x_robot * cos(yaw) - cell_y_robot * sin(yaw));
      double cell_y_map = robot_y + (cell_x_robot * sin(yaw) + cell_y_robot * cos(yaw));

      int global_x = static_cast<int>((cell_x_map - global_origin_x) / global_map_.info.resolution);
      int global_y = static_cast<int>((cell_y_map - global_origin_y) / global_map_.info.resolution);

      if (global_x >= 0 && global_x < static_cast<int>(global_map_.info.width) &&
          global_y >= 0 && global_y < static_cast<int>(global_map_.info.height))
      {

        int local_index = local_y * latest_costmap_.info.width + local_x;
        int8_t costmap_value = latest_costmap_.data[local_index];

        int global_index = global_y * global_map_.info.width + global_x;

        if (costmap_value != -1)
        {
          global_map_.data[global_index] = std::max(global_map_.data[global_index], costmap_value);
        }
      }
    }
  }

  global_map_.header.stamp = this->now();

  RCLCPP_DEBUG(this->get_logger(), "Integrated costmap into global map at robot pose (%.2f, %.2f, %.2f rad)",
               robot_x, robot_y, yaw);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
