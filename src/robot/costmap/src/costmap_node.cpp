#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
    
  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  resolution_ = 0.05;  // 10 cm per cell
  size_m_ = 50.0;
  width_ = static_cast<int>(size_m_ / resolution_);
  height_ = static_cast<int>(size_m_ / resolution_);

  grid_msg_.info.resolution = resolution_;
  grid_msg_.info.width = width_;
  grid_msg_.info.height = height_;
  grid_msg_.info.origin.orientation.w = 1.0;

  grid_msg_.info.origin.position.x = -size_m_ / 2.0 + resolution_ / 2.0;
  grid_msg_.info.origin.position.y = -size_m_ / 2.0 + resolution_ / 2.0;
  grid_msg_.header.frame_id = "robot/chassis/lidar";
  grid_msg_.data.resize(width_ * height_, -1); // Unknown space
  
  costmap_data_.assign(height_, std::vector<int>(width_, 0)); // Initialize with 0 (free space)
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received laser scan with %zu ranges", msg->ranges.size());
  last_scan_ = msg;
  publishCostmap();
}

void CostmapNode::publishCostmap() {
  if (!last_scan_) return;
  RCLCPP_DEBUG(this->get_logger(), "Publishing costmap");

  for (auto & row : costmap_data_) {
    for (auto & cell : row) {
      cell = 0;
    }
  }

  for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
    double range = last_scan_->ranges[i];
    double angle = -(last_scan_->angle_min + i * last_scan_->angle_increment);
    
    if (range < last_scan_->range_max && range > last_scan_->range_min && 
        !std::isnan(range) && !std::isinf(range)) {
      
      double obstacle_x = range * cos(angle);
      double obstacle_y = range * sin(angle);
      
      double x_exact = (obstacle_x - grid_msg_.info.origin.position.x) / resolution_;
      double y_exact = (-obstacle_y - grid_msg_.info.origin.position.y) / resolution_;
      
      int x = static_cast<int>(std::round(x_exact));
      int y = static_cast<int>(std::round(y_exact));
      
      if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        costmap_data_[y][x] = std::max(costmap_data_[y][x], 100);
      }
    }
  }

  inflateObstacles(1);

  for(int y = 0; y < height_; ++y) {
    for(int x = 0; x < width_; ++x) {
      grid_msg_.data[y * width_ + x] = costmap_data_[y][x];
    }
  }

  grid_msg_.header.stamp = this->now();
  grid_pub_->publish(grid_msg_);
}

void CostmapNode::inflateObstacles(int radius_m) {
  std::vector<std::vector<int>> inflated_map = costmap_data_;
  int radius = static_cast<int>(radius_m / resolution_);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (costmap_data_[y][x] == 100) { // If occupied
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dx = -radius; dx <= radius; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
              int cost = static_cast<int>(100 * (1.0 - sqrt(dx*dx + dy*dy) / radius));
              inflated_map[ny][nx] = std::max(inflated_map[ny][nx], cost);
            }
          }
        }
      }
    }
  }
  costmap_data_.swap(inflated_map);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}