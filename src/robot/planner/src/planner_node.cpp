#include "planner_node.hpp"
#include <queue>
#include <vector>
#include <unordered_map>
#include <algorithm>

// Constructor for PlannerNode
// Sets up all ROS2 communication (subscribers, publisher, timer)
// Constructor for PlannerNode
// Sets up all ROS2 communication (subscribers, publisher, timer)
PlannerNode::PlannerNode() : Node("planner")
{
  // Subscriber for the occupancy grid map (used for path planning)
  sub_msg_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  // Subscriber for the goal point (where the robot should go)
  sub_goal_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  // Subscriber for the robot's odometry (current position)
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher for the planned path
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer for periodic replanning and goal checking
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

// Callback for map updates
// Stores the latest map message and triggers replanning if needed
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Map received!");
  current_map_ = *msg;
  // Only replan if we are currently trying to reach a goal
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    planPath();
  }
}

// Callback for receiving a new goal
// Stores the goal and triggers planning
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal_msg)
{
  RCLCPP_INFO(this->get_logger(), "Goal received! x=%.2f y=%.2f frame=%s", goal_msg->point.x, goal_msg->point.y, goal_msg->header.frame_id.c_str());
  goal_ = *goal_msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}
void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  robot_pose_ = odom->pose.pose;
}
void PlannerNode::timerCallback()
{
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    if (goalReached())
    {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      planPath();
    }
  }
}

bool PlannerNode::goalReached()
{
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}

// A* path planner helpers
struct CellIndex
{
  int x, y;
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
  bool operator==(const CellIndex &other) const { return x == other.x && y == other.y; }
  bool operator!=(const CellIndex &other) const { return x != other.x || y != other.y; }
};
namespace std
{
  template <>
  struct hash<CellIndex>
  {
    std::size_t operator()(const CellIndex &idx) const
    {
      return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
  };
}

struct AStarNode
{
  CellIndex idx;
  double f;
  AStarNode(const CellIndex &i, double ff) : idx(i), f(ff) {}
  bool operator<(const AStarNode &other) const { return f > other.f; } // min-heap
};

// Helper: Convert world coordinates to grid index
static CellIndex worldToGrid(const nav_msgs::msg::OccupancyGrid &map, double x, double y)
{
  int gx = static_cast<int>((x - map.info.origin.position.x) / map.info.resolution);
  int gy = static_cast<int>((y - map.info.origin.position.y) / map.info.resolution);
  return CellIndex(gx, gy);
}

// Helper: Convert grid index to world coordinates
static geometry_msgs::msg::PoseStamped gridToWorld(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &idx)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = map.info.origin.position.x + idx.x * map.info.resolution;
  pose.pose.position.y = map.info.origin.position.y + idx.y * map.info.resolution;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

// Helper: Check if cell is valid and free
static bool isValidCell(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &idx)
{
  int w = map.info.width, h = map.info.height;
  if (idx.x < 0 || idx.y < 0 || idx.x >= w || idx.y >= h)
    return false;
  int i = idx.y * w + idx.x;
  int8_t val = map.data[i];
  // Treat unknown or high cost as obstacle
  return val >= 0 && val < 90;
}

void PlannerNode::planPath()
{
  if (!goal_received_ || current_map_.data.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }
  auto start = worldToGrid(current_map_, robot_pose_.position.x, robot_pose_.position.y);
  auto goal = worldToGrid(current_map_, goal_.point.x, goal_.point.y);
  int w = current_map_.info.width, h = current_map_.info.height;
  if (!isValidCell(current_map_, start))
  {
    RCLCPP_WARN(this->get_logger(), "Start is in obstacle or out of bounds!");
    return;
  }
  if (!isValidCell(current_map_, goal))
  {
    RCLCPP_WARN(this->get_logger(), "Goal is in obstacle or out of bounds!");
    return;
  }
  std::priority_queue<AStarNode> open;
  std::unordered_map<CellIndex, double> gScore;
  std::unordered_map<CellIndex, CellIndex> came_from;
  auto heuristic = [](const CellIndex &a, const CellIndex &b)
  {
    double dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  };
  gScore[start] = 0.0;
  open.push(AStarNode(start, heuristic(start, goal)));
  std::vector<std::pair<int, int>> directions = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
  bool found = false;
  while (!open.empty())
  {
    auto curr = open.top().idx;
    open.pop();
    if (curr == goal)
    {
      found = true;
      break;
    }
    for (const auto &d : directions)
    {
      CellIndex next(curr.x + d.first, curr.y + d.second);
      if (!isValidCell(current_map_, next))
        continue;
      double step = (d.first == 0 || d.second == 0) ? 1.0 : std::sqrt(2.0);
      int i = next.y * w + next.x;
      double penalty = current_map_.data[i] / 25.0; // scale cost
      double tentative_g = gScore[curr] + step + penalty;
      if (!gScore.count(next) || tentative_g < gScore[next])
      {
        gScore[next] = tentative_g;
        double f = tentative_g + heuristic(next, goal);
        open.push(AStarNode(next, f));
        came_from[next] = curr;
      }
    }
  }
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "map";
  if (found)
  {
    std::vector<CellIndex> rev_path;
    for (CellIndex at = goal; at != start; at = came_from[at])
      rev_path.push_back(at);
    rev_path.push_back(start);
    std::reverse(rev_path.begin(), rev_path.end());
    for (const auto &idx : rev_path)
      path.poses.push_back(gridToWorld(current_map_, idx));
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "No path found!");
  }
  path_pub_->publish(path);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
