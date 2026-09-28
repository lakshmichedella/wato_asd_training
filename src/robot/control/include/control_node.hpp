#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "control_core.hpp"

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>
#include <cmath>
#include <optional>

class ControlNode : public rclcpp::Node
{
public:
    ControlNode() : Node("pure_pursuit_controller"), control_(robot::ControlCore(this->get_logger()))
    {
        // Initialize parameters
        lookahead_distance_ = 1.0; // Lookahead distance
        goal_tolerance_ = 0.1;     // Distance to consider the goal reached
        linear_speed_ = 0.5;       // Constant forward speed

        // Subscribers and Publishers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg)
            { current_path_ = msg; });

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            { robot_odom_ = msg; });

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), [this]()
            { controlLoop(); });
    }

    virtual ~ControlNode() override = default;

private:
    void controlLoop()
    {
        // Skip control if no path or odometry data is available
        if (!current_path_ || !robot_odom_)
        {
            return;
        }

        // Find the lookahead point
        auto lookahead_point = findLookaheadPoint();
        if (!lookahead_point)
        {
            return; // No valid lookahead point found
        }

        // Compute velocity command
        auto cmd_vel = computeVelocity(*lookahead_point);

        // Publish the velocity command
        cmd_vel_pub_->publish(cmd_vel);
    }

    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint()
    {
        if (!current_path_ || current_path_->poses.empty() || !robot_odom_)
        {
            return std::nullopt;
        }

        const auto &poses = current_path_->poses;
        const auto &position = robot_odom_->pose.pose.position;

        for (const auto &pose : poses)
        {
            if (computeDistance(position, pose.pose.position) >= lookahead_distance_)
            {
                return pose;
            }
        }
        return poses.back();
    }

    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target)
    {
        geometry_msgs::msg::Twist cmd_vel;
        const auto &pose = robot_odom_->pose.pose;
        const double robot_yaw = extractYaw(pose.orientation);
        const double dx = target.pose.position.x - pose.position.x;
        const double dy = target.pose.position.y - pose.position.y;
        const double distance = std::hypot(dx, dy);
        if (distance <= goal_tolerance_)
        {
            return cmd_vel;
        }

        const double target_heading = std::atan2(dy, dx);
        const double heading_error = std::atan2(std::sin(target_heading - robot_yaw),
                                                std::cos(target_heading - robot_yaw));
        constexpr double max_angular_speed = 1.5;
        if (std::abs(heading_error) > 1.2)
        {
            cmd_vel.angular.z = std::clamp(2.0 * heading_error,
                                           -max_angular_speed, max_angular_speed);
            return cmd_vel;
        }

        const double local_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
        const double curvature = 2.0 * local_y / (distance * distance);
        cmd_vel.linear.x = linear_speed_ * std::max(0.0, std::cos(heading_error));
        cmd_vel.angular.z = std::clamp(cmd_vel.linear.x * curvature,
                                       -max_angular_speed, max_angular_speed);
        return cmd_vel;
    }

    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
    {
        return std::hypot(a.x - b.x, a.y - b.y);
    }

    double extractYaw(const geometry_msgs::msg::Quaternion &quat)
    {
        return std::atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                          1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
    }

    // Subscribers and Publishers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Data
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;

    // Parameters
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
    robot::ControlCore control_;
};

#endif
