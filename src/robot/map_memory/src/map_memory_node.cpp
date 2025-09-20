#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())), last_x(0.0), last_y(0.0) {
  // Subscribers and Publishers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Timer for periodic map updates every second
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&MapMemoryNode::updateMap, this));

  // Initialize parameters
  this->declare_parameter("resolution", 0.05);
  this->declare_parameter("size_m", 50.0);
  this->declare_parameter("distance_threshold", 5.0);

  this->get_parameter("resolution", resolution_);
  this->get_parameter("size_m", size_m_);
  this->get_parameter("distance_threshold", distance_threshold_);

  width_ = static_cast<int>(size_m_ / resolution_);
  height_ = static_cast<int>(size_m_ / resolution_);
  
  // Initialize the global map
  global_map_.info.resolution = resolution_;
  global_map_.info.width = width_;
  global_map_.info.height = height_;
  global_map_.info.origin.position.x = -size_m_ / 2.0 + resolution_ / 2.0;
  global_map_.info.origin.position.y = -size_m_ / 2.0 + resolution_ / 2.0;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.header.frame_id = "sim_world";
  global_map_.data.resize(width_ * height_, 0);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  latest_costmap_ = msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_ = msg;
  
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));

  // Update if moved more than threshold
  if (distance >= distance_threshold_) {
    last_x = x;
    last_y = y;
    should_update_map_ = true;
  }
}

void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_updated_) {
    integrateCostmapIntoMap();
    map_pub_->publish(global_map_);
    should_update_map_ = false;
  }
}

void MapMemoryNode::integrateCostmapIntoMap() {
  nav_msgs::msg::OccupancyGrid::SharedPtr current_costmap;
  nav_msgs::msg::Odometry::SharedPtr current_odom;

  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!latest_costmap_) return;
    current_costmap = latest_costmap_;
  }

  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!latest_odom_) return;
    current_odom = latest_odom_;
  }

  if (!costmap_updated_ || !current_costmap || !current_odom) {
    RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for costmap or odom data...");
    return;
  }

  // Get robot pose from odometry
  double robot_x = current_odom->pose.pose.position.x;
  double robot_y = current_odom->pose.pose.position.y;
  auto& q = current_odom->pose.pose.orientation;
  double yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // Costmap origin in robot frame
  double costmap_origin_x = current_costmap->info.origin.position.x;
  double costmap_origin_y = current_costmap->info.origin.position.y;

  // Global map origin
  double global_origin_x = global_map_.info.origin.position.x;
  double global_origin_y = global_map_.info.origin.position.y;
  
  // Integrate costmap into global map
  for (int local_y = 0; local_y < static_cast<int>(current_costmap->info.height); ++local_y) {
    for (int local_x = 0; local_x < static_cast<int>(current_costmap->info.width); ++local_x) {
      
      double cell_x_robot = local_x * current_costmap->info.resolution + costmap_origin_x;
      double cell_y_robot = local_y * current_costmap->info.resolution + costmap_origin_y;
      
      double cell_x_map = robot_x + (cell_x_robot * cos(yaw) - cell_y_robot * sin(yaw));
      double cell_y_map = robot_y + (cell_x_robot * sin(yaw) + cell_y_robot * cos(yaw));
      

      double exact_x = (cell_x_map - global_origin_x) / global_map_.info.resolution;
      double exact_y = (cell_y_map - global_origin_y) / global_map_.info.resolution;

      int global_x = static_cast<int>(std::round(exact_x));
      int global_y = static_cast<int>(std::round(exact_y));

      // Calculate the local index and costmap value
      int local_index = local_y * current_costmap->info.width + local_x;
      int8_t costmap_value = current_costmap->data[local_index];
      
      // Only process valid costmap values
      if (costmap_value != -1) {
        // Update the primary cell and surrounding cells to reduce holes
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            int target_x = global_x + dx;
            int target_y = global_y + dy;
            
            // Check if the target coordinates are within bounds
            if (target_x >= 0 && target_x < static_cast<int>(global_map_.info.width) &&
                target_y >= 0 && target_y < static_cast<int>(global_map_.info.height)) {
              
              int global_index = target_y * global_map_.info.width + target_x;
              
              // Apply full value to center cell, reduced value to surrounding cells
              int8_t update_value;
              if (dx == 0 && dy == 0) {
                // Center cell gets full value
                update_value = costmap_value;
              } else {
                // Surrounding cells get reduced value to maintain accuracy
                update_value = static_cast<int8_t>(costmap_value * 0.7);
              }
              
              // Update the global map with the highest occupancy
              global_map_.data[global_index] = std::max(global_map_.data[global_index], update_value);
            }
          }
        }
      }
    }
  }
  
  global_map_.header.stamp = this->now();
  
  RCLCPP_DEBUG(this->get_logger(), "Integrated costmap into global map at robot pose (%.2f, %.2f, %.2f rad)", 
              robot_x, robot_y, yaw);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
