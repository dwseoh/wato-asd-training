#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <optional>

namespace robot
{

// supporting structures

// 2d grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// hash for cellindex
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// a* search node
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// priority queue comparator (min-heap)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    return a.f_score > b.f_score;
  }
};

// planner core logic class

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // state machine definitions
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };

    // state update methods
    void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void updateGoal(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void updatePose(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // core planning logic
    bool checkGoalReached();
    std::optional<nav_msgs::msg::Path> planPath();

    State getState() const { return state_; }
    void setState(State s) { state_ = s; }

  private:
    rclcpp::Logger logger_;
    State state_;

    // internal data storage
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;
    bool goal_received_ = false;

    // configuration parameters
    double check_bound_ratio_ = 1.0;
    int8_t cost_threshold_ = 50;

    // internal a* helpers
    double heuristic(const CellIndex& a, const CellIndex& b);
    std::vector<CellIndex> getNeighbors(const CellIndex& current);
    bool isValid(const CellIndex& idx, double yaw);
    CellIndex worldToGrid(double wx, double wy);
    void gridToWorld(const CellIndex& idx, double& wx, double& wy);
};

}  // namespace robot

#endif  // PLANNER_CORE_HPP_
