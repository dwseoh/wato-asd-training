#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <mutex>

namespace robot
{

class MapMemoryCore {
public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void updateCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void updateOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    bool shouldUpdateMap();
    void updateMap();
    nav_msgs::msg::OccupancyGrid getGlobalMap();

private:
    rclcpp::Logger logger_;
    
    // internal data structures
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;

    // robot state tracking
    double last_x;
    double last_y;
    double current_x;
    double current_y;
    double current_yaw;

    // configuration parameters
    double distance_threshold;

    // internal flags
    bool costmap_received_;
    bool should_update_map_;
    bool map_initialized_;

    // synchronization
    std::mutex data_mutex_;

    // helper methods
    void initGlobalMap(const nav_msgs::msg::MapMetaData& costmap_info);
    void integrateCostmap();
};

}  // namespace robot

#endif  // MAP_MEMORY_CORE_HPP_
