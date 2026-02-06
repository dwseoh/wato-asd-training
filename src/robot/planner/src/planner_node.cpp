#include "planner_node.hpp"

PlannerNode::PlannerNode() 
: Node("planner_node"), planner_(this->get_logger())
{
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Planner Node initialized.");
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    planner_.updateMap(msg);
    if (planner_.getState() == robot::PlannerCore::State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        auto path = planner_.planPath();
        if (path) {
            path_pub_->publish(*path);
        }
    }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received new goal.");
    planner_.updateGoal(msg);
    auto path = planner_.planPath();
    if (path) {
        path_pub_->publish(*path);
    }
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    planner_.updatePose(msg);
}

void PlannerNode::timerCallback()
{
    if (planner_.getState() == robot::PlannerCore::State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        if (planner_.checkGoalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            planner_.setState(robot::PlannerCore::State::WAITING_FOR_GOAL);
        } else {
            auto path = planner_.planPath();
            if (path) {
                path_pub_->publish(*path);
            }
        }
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
