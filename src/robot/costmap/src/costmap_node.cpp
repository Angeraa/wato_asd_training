#include <chrono>
#include <memory>
#include <algorithm>
#include <cmath>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Subscribers and Publishers
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
    
  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  // Initialize parameters
  this->declare_parameter("resolution", 0.05);
  this->declare_parameter("size_m", 50.0);
  this->declare_parameter("inflation_radius", 1);
  this->declare_parameter("update_rate", 20.0);

  this->get_parameter("resolution", resolution_);
  this->get_parameter("size_m", size_m_);
  this->get_parameter("inflation_radius", inflation_radius_);
  this->get_parameter("update_rate", update_rate_);
  
  width_ = static_cast<int>(size_m_ / resolution_);
  height_ = static_cast<int>(size_m_ / resolution_);
  inverse_resolution_ = 1.0 / resolution_;

  // Create timer for periodic costmap updates
  auto timer_period = std::chrono::duration<double>(1.0 / update_rate_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
    std::bind(&CostmapNode::timerCallback, this));

  // Set up the occupancy grid message
  grid_msg_.info.resolution = resolution_;
  grid_msg_.info.width = width_;
  grid_msg_.info.height = height_;
  grid_msg_.info.origin.orientation.w = 1.0;

  grid_msg_.info.origin.position.x = -size_m_ / 2.0 + resolution_ / 2.0;
  grid_msg_.info.origin.position.y = -size_m_ / 2.0 + resolution_ / 2.0;
  grid_msg_.header.frame_id = "robot/chassis/lidar";
  grid_msg_.data.resize(width_ * height_, -1); // Unknown space
  
  // Allocate memory for costmap and inflated map
  costmap_data_.assign(height_, std::vector<int>(width_, 0));
  inflated_map_.assign(height_, std::vector<int>(width_, 0));
  
  // Initialize distance cache for inflation
  initializeDistanceCache();
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received laser scan with %zu ranges", msg->ranges.size());
  
  // Thread-safe update of the latest scan data
  std::lock_guard<std::mutex> lock(scan_mutex_);
  last_scan_ = msg;
}

void CostmapNode::timerCallback() {
  publishCostmap();
}

void CostmapNode::publishCostmap() {
  // Thread-safe access to scan data
  sensor_msgs::msg::LaserScan::SharedPtr current_scan;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    if (!last_scan_) return;
    current_scan = last_scan_;
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Publishing costmap");

  // Clear costmap data
  clearCostmap();

  // Pre-compute trigonometric values for better performance
  const size_t num_ranges = current_scan->ranges.size();
  const double angle_min = current_scan->angle_min;
  const double angle_increment = current_scan->angle_increment;
  const double range_min = current_scan->range_min;
  const double range_max = current_scan->range_max;
  const double origin_x = grid_msg_.info.origin.position.x;
  const double origin_y = grid_msg_.info.origin.position.y;

  // Populate costmap based on laser scan
  for (size_t i = 0; i < num_ranges; ++i) {
    const double range = current_scan->ranges[i];
    
    if (range < range_max && range > range_min &&
        !std::isnan(range) && !std::isinf(range)) {
      
      // Convert to grid coordinates
      const double angle = -(angle_min + i * angle_increment);
      const double obstacle_x = range * cos(angle);
      const double obstacle_y = range * sin(angle);
      
      const int x = static_cast<int>(std::round((obstacle_x - origin_x) * inverse_resolution_));
      const int y = static_cast<int>(std::round((-obstacle_y - origin_y) * inverse_resolution_));
      
      // Mark the cell as occupied if within bounds
      if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        costmap_data_[y][x] = 100;
      }
    }
  }

  // Inflate obstacles
  inflateObstacles(inflation_radius_);

  // Update the occupancy grid message data
  for(int y = 0; y < height_; ++y) {
    for(int x = 0; x < width_; ++x) {
      grid_msg_.data[y * width_ + x] = costmap_data_[y][x];
    }
  }

  grid_msg_.header.stamp = this->now();
  grid_pub_->publish(grid_msg_);
}

void CostmapNode::initializeDistanceCache() {
  // Pre-compute distances for inflation radius to avoid sqrt calls during inflation
  int radius = static_cast<int>(inflation_radius_ * inverse_resolution_);
  distance_cache_.assign(2 * radius + 1, std::vector<double>(2 * radius + 1, 0.0));
  
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      distance_cache_[dy + radius][dx + radius] = sqrt(dx*dx + dy*dy);
    }
  }
}

void CostmapNode::clearCostmap() {
  for (auto & row : costmap_data_) {
    std::fill(row.begin(), row.end(), 0);
  }
}

void CostmapNode::inflateObstacles(int radius_m) {
  // Copy current costmap to inflated map
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      inflated_map_[y][x] = costmap_data_[y][x];
    }
  }
  
  int radius = static_cast<int>(radius_m * inverse_resolution_);
  const double inv_radius = 1.0 / radius;
  
  // Only process cells that have obstacles
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (costmap_data_[y][x] == 100) { // If occupied
        const int min_x = std::max(0, x - radius);
        const int max_x = std::min(width_ - 1, x + radius);
        const int min_y = std::max(0, y - radius);
        const int max_y = std::min(height_ - 1, y + radius);
        
        for (int ny = min_y; ny <= max_y; ++ny) {
          const int dy = ny - y;
          for (int nx = min_x; nx <= max_x; ++nx) {
            const int dx = nx - x;
            
            // Use pre-computed distance if available, otherwise calculate
            double distance;
            if (abs(dx) <= radius && abs(dy) <= radius) {
              distance = distance_cache_[dy + radius][dx + radius];
            } else {
              distance = sqrt(dx*dx + dy*dy);
            }
            
            if (distance <= radius) {
              // Calculate cost based on distance
              int cost = static_cast<int>(100 * (1.0 - distance * inv_radius));
              
              // Clamp cost between 0 and 100
              cost = std::max(0, std::min(100, cost));
              
              // Update the inflated map if the new cost is higher
              inflated_map_[ny][nx] = std::max(inflated_map_[ny][nx], cost);
            }
          }
        }
      }
    }
  }
  costmap_data_.swap(inflated_map_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}