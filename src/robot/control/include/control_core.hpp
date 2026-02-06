#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <optional>

namespace robot
{

class ControlCore {
  public:
    // constructor with logger initialization
    ControlCore(const rclcpp::Logger& logger);

    void updatePath(const nav_msgs::msg::Path::SharedPtr msg);
    void updateOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);

    geometry_msgs::msg::Twist computeVelocityCommand();

  private:
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();
    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);

    rclcpp::Logger logger_;

    // robot data state
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;

    // control parameters
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
};

} 

#endif 
