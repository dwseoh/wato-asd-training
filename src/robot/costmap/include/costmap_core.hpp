#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    // process scan into costmap
    nav_msgs::msg::OccupancyGrid processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  private:
    void initializeCostmap();
    void convertToGrid(double range, double angle, int &grid_x, int &grid_y);
    void markObstacle(int grid_x, int grid_y);
    void inflateObstacles();

    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid costmap_;
    std::vector<std::pair<int, int>> obstacles_;
    
    // map parameters
    double resolution_{0.05};
    int width_{200};
    int height_{200};
    double origin_x_{-5.0};
    double origin_y_{-5.0};
    double inflation_radius_{1.5}; // inflate to avoid being stuck
};

}

#endif  