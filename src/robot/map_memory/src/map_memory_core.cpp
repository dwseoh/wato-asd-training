#include "map_memory_core.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger), 
    last_x(0.0), last_y(0.0), 
    current_x(0.0), current_y(0.0), current_yaw(0.0),
    distance_threshold(1.5),
    costmap_received_(false), should_update_map_(false), map_initialized_(false) 
{
}

void MapMemoryCore::updateCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_costmap_ = *msg;
    costmap_received_ = true;
    
    if (!map_initialized_) {
        initGlobalMap(msg->info);
    }
}

void MapMemoryCore::updateOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    current_x = x;
    current_y = y;
    current_yaw = yaw;

    double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
    if (distance >= distance_threshold) {
        last_x = x;
        last_y = y;
        should_update_map_ = true;
        RCLCPP_INFO(logger_, "Distance threshold reached: %.2f m. Flagged for update.", distance);
    }
}

bool MapMemoryCore::shouldUpdateMap() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return should_update_map_ && costmap_received_ && map_initialized_;
}

void MapMemoryCore::updateMap() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (should_update_map_ && costmap_received_) {
        integrateCostmap();
        should_update_map_ = false;
    }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return global_map_;
}

void MapMemoryCore::initGlobalMap(const nav_msgs::msg::MapMetaData& costmap_info) {
    float map_width_meters = 100.0;
    float map_height_meters = 100.0;
    
    global_map_.header.frame_id = "sim_world"; 
    global_map_.info.resolution = 0.1; 
    global_map_.info.width = static_cast<uint32_t>(map_width_meters / global_map_.info.resolution);
    global_map_.info.height = static_cast<uint32_t>(map_height_meters / global_map_.info.resolution);
    
    global_map_.info.origin.position.x = -map_width_meters / 2.0;
    global_map_.info.origin.position.y = -map_height_meters / 2.0;
    global_map_.info.origin.position.z = 0.0;
    global_map_.info.origin.orientation.w = 1.0;

    global_map_.data.assign(global_map_.info.width * global_map_.info.height, -1);
    
    map_initialized_ = true;
    RCLCPP_INFO(logger_, "Global Map Initialized: %dx%d", global_map_.info.width, global_map_.info.height);
}

void MapMemoryCore::integrateCostmap() {
    int width = latest_costmap_.info.width;
    int height = latest_costmap_.info.height;
    double res = latest_costmap_.info.resolution;
    double origin_x = latest_costmap_.info.origin.position.x;
    double origin_y = latest_costmap_.info.origin.position.y;

    int g_width = global_map_.info.width;
    int g_height = global_map_.info.height;
    double g_res = global_map_.info.resolution;
    double g_origin_x = global_map_.info.origin.position.x;
    double g_origin_y = global_map_.info.origin.position.y;
    
    double cos_yaw = std::cos(current_yaw);
    double sin_yaw = std::sin(current_yaw);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int i = x + y * width;
            int8_t cell_value = latest_costmap_.data[i];
            
            if (cell_value == -1) continue; 

            double lx = origin_x + (x + 0.5) * res;
            double ly = origin_y + (y + 0.5) * res;
            
            double gx = current_x + (lx * cos_yaw - ly * sin_yaw);
            double gy = current_y + (lx * sin_yaw + ly * cos_yaw);
            
            int gx_idx = static_cast<int>((gx - g_origin_x) / g_res);
            int gy_idx = static_cast<int>((gy - g_origin_y) / g_res);
            
            if (gx_idx >= 0 && gx_idx < g_width && gy_idx >= 0 && gy_idx < g_height) {
                int g_i = gx_idx + gy_idx * g_width;
                global_map_.data[g_i] = cell_value;
            }
        }
    }
}

} 
