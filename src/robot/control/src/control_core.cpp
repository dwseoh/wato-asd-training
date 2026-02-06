#include "control_core.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {
    lookahead_distance_ = 0.4; 
    goal_tolerance_ = 0.5;
    linear_speed_ = 0.5;
}

void ControlCore::updatePath(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = msg;
}

void ControlCore::updateOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_odom_ = msg;
}

geometry_msgs::msg::Twist ControlCore::computeVelocityCommand() {
  geometry_msgs::msg::Twist cmd_vel;

  if (!current_path_ || !robot_odom_) {
    return cmd_vel;
  }

  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
      return cmd_vel;
  }

  double robot_x = robot_odom_->pose.pose.position.x;
  double robot_y = robot_odom_->pose.pose.position.y;
  double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

  double target_x = lookahead_point->pose.position.x;
  double target_y = lookahead_point->pose.position.y;

  double dx = target_x - robot_x;
  double dy = target_y - robot_y;

  
  double sin_yaw = std::sin(robot_yaw);
  double cos_yaw = std::cos(robot_yaw);
  
  double xt =  dx * cos_yaw + dy * sin_yaw; 
  double yt = -dx * sin_yaw + dy * cos_yaw;

  if (xt < 0) {
      
      double sign = (yt >= 0) ? 1.0 : -1.0;
      cmd_vel.linear.x = 0.0; 
      cmd_vel.angular.z = sign * 1.0; 
      return cmd_vel;
  }

  double distance_sq = dx*dx + dy*dy;
  
  if (distance_sq < 0.001) {
      return cmd_vel;
  }

  double angular_velocity = (2.0 * yt / distance_sq) * linear_speed_;

  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = angular_velocity;

  return cmd_vel;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint() {
  if (!current_path_ || current_path_->poses.empty()) {
    return std::nullopt;
  }

  geometry_msgs::msg::Point robot_pos = robot_odom_->pose.pose.position;

  size_t closest_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::max();
  
  for (size_t i = 0; i < current_path_->poses.size(); ++i) {
      double dist = computeDistance(robot_pos, current_path_->poses[i].pose.position);
      double dist_sq = dist*dist;
      if (dist_sq < min_dist_sq) {
          min_dist_sq = dist_sq;
          closest_idx = i;
      }
  }

  for (size_t i = closest_idx; i < current_path_->poses.size(); ++i) {
    double dist = computeDistance(robot_pos, current_path_->poses[i].pose.position);
    if (dist >= lookahead_distance_) {
      return current_path_->poses[i];
    }
  }

  const auto& last_pose = current_path_->poses.back();
  double dist_to_goal = computeDistance(robot_pos, last_pose.pose.position);
  if (dist_to_goal > goal_tolerance_) {
      return last_pose;
  }

  return std::nullopt; 
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx*dx + dy*dy);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  tf2::Quaternion tf_quat;
  tf2::fromMsg(quat, tf_quat);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
  return yaw;
}

}
