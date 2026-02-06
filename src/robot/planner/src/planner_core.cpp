#include "planner_core.hpp"
#include <algorithm>
#include <limits>

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger &logger)
    : logger_(logger), state_(State::WAITING_FOR_GOAL) {}

void PlannerCore::updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
}

void PlannerCore::updateGoal(
    const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
}

void PlannerCore::updatePose(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

bool PlannerCore::checkGoalReached() {
  if (!goal_received_)
    return false;

  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  double distance_to_goal = std::sqrt(dx * dx + dy * dy);

  return distance_to_goal < 0.5;
}

std::optional<nav_msgs::msg::Path> PlannerCore::planPath() {
  if (!goal_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(logger_, "Path planning aborted: missing map data or goal");
    return std::nullopt;
  }

  nav_msgs::msg::Path planned_path;
  planned_path.header.stamp = rclcpp::Clock().now();
  planned_path.header.frame_id = current_map_.header.frame_id;

  CellIndex robot_cell =
      worldToGrid(robot_pose_.position.x, robot_pose_.position.y);
  CellIndex target_cell = worldToGrid(goal_.point.x, goal_.point.y);

  auto &orientation = robot_pose_.orientation;
  double sin_yaw_cos_pitch =
      2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
  double cos_yaw_cos_pitch = 1.0 - 2.0 * (orientation.y * orientation.y +
                                          orientation.z * orientation.z);
  double robot_heading = std::atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch);

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> frontier;
  std::unordered_map<CellIndex, double, CellIndexHash> cost_so_far;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> parent_map;

  bool path_found = false;
  check_bound_ratio_ = 1.0;
  cost_threshold_ = 50; 

  const int MAX_PLANNING_ATTEMPTS = 5;
  int attempt_number = 0;

  while (!path_found && attempt_number < MAX_PLANNING_ATTEMPTS) {

    bool start_position_valid = isValid(robot_cell, robot_heading);

    if (!start_position_valid) {
      int cell_index = robot_cell.y * current_map_.info.width + robot_cell.x;
      if (cell_index >= 0 && cell_index < (int)current_map_.data.size()) {
        int8_t center_cost = current_map_.data[cell_index];
        if (center_cost < cost_threshold_) {
          start_position_valid = true;
          RCLCPP_DEBUG(logger_, "Robot footprint partially obstructed, but "
                                "center is safe - proceeding");
        }
      }
    }

    if (!start_position_valid) {
      cost_threshold_ += 10; 
      attempt_number++;
      RCLCPP_DEBUG(logger_,
                   "Planning attempt %d: relaxing cost threshold to %d",
                   attempt_number, cost_threshold_);
      continue;
    }

    if (!isValid(target_cell, robot_heading)) {
      cost_threshold_ += 10;
      attempt_number++;
      RCLCPP_DEBUG(logger_, "Goal position invalid, relaxing constraints");
      continue;
    }

    while (!frontier.empty())
      frontier.pop();
    cost_so_far.clear();
    parent_map.clear();

    cost_so_far[robot_cell] = 0.0;
    frontier.emplace(robot_cell, heuristic(robot_cell, target_cell));

    while (!frontier.empty()) {
      AStarNode current_node = frontier.top();
      frontier.pop();

      if (current_node.index == target_cell) {
        path_found = true;
        break;
      }

      std::vector<CellIndex> adjacent_cells = getNeighbors(current_node.index);
      for (const auto &neighbor_cell : adjacent_cells) {
        double movement_cost =
            std::sqrt(std::pow(neighbor_cell.x - current_node.index.x, 2) +
                      std::pow(neighbor_cell.y - current_node.index.y, 2));

        int neighbor_index =
            neighbor_cell.y * current_map_.info.width + neighbor_cell.x;
        int8_t obstacle_cost = current_map_.data[neighbor_index];
        double safety_penalty = 0.0;
        if (obstacle_cost > 0) {
          safety_penalty = static_cast<double>(obstacle_cost) * 0.05;
        }

        double new_cost =
            cost_so_far[current_node.index] + movement_cost + safety_penalty;

        if (cost_so_far.find(neighbor_cell) == cost_so_far.end() ||
            new_cost < cost_so_far[neighbor_cell]) {
          parent_map[neighbor_cell] = current_node.index;
          cost_so_far[neighbor_cell] = new_cost;
          double priority = new_cost + heuristic(neighbor_cell, target_cell);
          frontier.emplace(neighbor_cell, priority);
        }
      }
    }

    if (!path_found) {
      cost_threshold_ += 10;
      attempt_number++;
      RCLCPP_DEBUG(logger_, "No path found on attempt %d, relaxing constraints",
                   attempt_number);
    }
  }

  if (path_found) {
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    CellIndex current_cell = target_cell;

    while (current_cell != robot_cell) {
      geometry_msgs::msg::PoseStamped waypoint;
      waypoint.header = planned_path.header;

      double world_x, world_y;
      gridToWorld(current_cell, world_x, world_y);
      waypoint.pose.position.x = world_x;
      waypoint.pose.position.y = world_y;
      waypoint.pose.orientation.w = 1.0; 

      waypoints.push_back(waypoint);
      current_cell = parent_map[current_cell];
    }

    geometry_msgs::msg::PoseStamped start_waypoint;
    start_waypoint.header = planned_path.header;
    start_waypoint.pose = robot_pose_;
    waypoints.push_back(start_waypoint);

    std::reverse(waypoints.begin(), waypoints.end());
    planned_path.poses = waypoints;

    RCLCPP_INFO(logger_, "Successfully planned path with %zu waypoints",
                waypoints.size());
    return planned_path;
  } else {
    RCLCPP_WARN(logger_, "Failed to find valid path after %d attempts",
                MAX_PLANNING_ATTEMPTS);
    return std::nullopt;
  }
}


/**
 * @brief Heuristic function for A* (Euclidean distance)
 * Estimates the cost from cell 'a' to cell 'b'
 */
double PlannerCore::heuristic(const CellIndex &a, const CellIndex &b) {
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

/**
 * @brief Get all valid neighboring cells (8-connected grid)
 * Returns cells that are within bounds and collision-free
 */
std::vector<CellIndex> PlannerCore::getNeighbors(const CellIndex &current) {
  std::vector<CellIndex> valid_neighbors;

  const int direction_offsets[8][2] = {
      {0, 1},   
      {1, 1},   
      {1, 0},   
      {1, -1},  
      {0, -1},  
      {-1, -1}, 
      {-1, 0},  
      {-1, 1}   
  };

  for (int i = 0; i < 8; ++i) {
    int delta_x = direction_offsets[i][0];
    int delta_y = direction_offsets[i][1];
    CellIndex neighbor_cell(current.x + delta_x, current.y + delta_y);

    double movement_heading = std::atan2(delta_y, delta_x);

    if (isValid(neighbor_cell, movement_heading)) {
      valid_neighbors.push_back(neighbor_cell);
    }
  }

  return valid_neighbors;
}

/**
 * @brief Check if a cell is valid for the robot to occupy
 * Validates both map bounds and collision with obstacles
 * Considers robot's full footprint, not just center point
 */
bool PlannerCore::isValid(const CellIndex &cell, double heading) {
  if (cell.x < 0 || cell.x >= (int)current_map_.info.width || cell.y < 0 ||
      cell.y >= (int)current_map_.info.height) {
    return false;
  }

  const double ROBOT_LENGTH = 1.5; 
  const double ROBOT_WIDTH = 1.0;  

  double sampling_step = current_map_.info.resolution;
  if (sampling_step <= 0.001) {
    sampling_step = 0.1; 
  }

  double center_x, center_y;
  gridToWorld(cell, center_x, center_y);

  for (double longitudinal = 0.0;
       longitudinal <= ROBOT_LENGTH + sampling_step / 2.0;
       longitudinal += sampling_step) {
    for (double lateral = -ROBOT_WIDTH / 2.0;
         lateral <= ROBOT_WIDTH / 2.0 + sampling_step / 2.0;
         lateral += sampling_step) {

      double offset_x = (longitudinal - ROBOT_LENGTH / 2.0);
      double offset_y = lateral;

      double world_x = center_x + std::cos(heading) * offset_x -
                       std::sin(heading) * offset_y;
      double world_y = center_y + std::sin(heading) * offset_x +
                       std::cos(heading) * offset_y;

      CellIndex footprint_cell = worldToGrid(world_x, world_y);

      if (footprint_cell.x < 0 ||
          footprint_cell.x >= (int)current_map_.info.width ||
          footprint_cell.y < 0 ||
          footprint_cell.y >= (int)current_map_.info.height) {
        return false; 
      }

      int map_index =
          footprint_cell.y * current_map_.info.width + footprint_cell.x;
      int8_t occupancy_value = current_map_.data[map_index];
      if (occupancy_value >= cost_threshold_) {
        return false; 
      }
    }
  }

  return true; 
}

/**
 * @brief Convert world coordinates to grid cell indices
 */
CellIndex PlannerCore::worldToGrid(double world_x, double world_y) {
  double origin_x = current_map_.info.origin.position.x;
  double origin_y = current_map_.info.origin.position.y;
  double resolution = current_map_.info.resolution;

  int grid_x = static_cast<int>((world_x - origin_x) / resolution);
  int grid_y = static_cast<int>((world_y - origin_y) / resolution);

  return CellIndex(grid_x, grid_y);
}

/**
 * @brief Convert grid cell indices to world coordinates (cell center)
 */
void PlannerCore::gridToWorld(const CellIndex &cell, double &world_x,
                              double &world_y) {
  double origin_x = current_map_.info.origin.position.x;
  double origin_y = current_map_.info.origin.position.y;
  double resolution = current_map_.info.resolution;

  world_x = origin_x + (cell.x + 0.5) * resolution;
  world_y = origin_y + (cell.y + 0.5) * resolution;
}

} 
