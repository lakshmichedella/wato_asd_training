#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

class PlannerNode : public rclcpp::Node
{
public:
    PlannerNode() : Node("planner_node"), state_(State::WAITING_FOR_GOAL), planner_(robot::PlannerCore(this->get_logger()))
    {
        // Subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

        // Publisher
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
    }

private:
    enum class State
    {
        WAITING_FOR_GOAL,
        WAITING_FOR_ROBOT_TO_REACH_GOAL
    };
    State state_;

    // Subscribers and Publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data Storage
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;

    bool goal_received_ = false;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        current_map_ = *msg;
        if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
        {
            planPath();
        }
    }

    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        goal_ = *msg;
        goal_received_ = true;
        state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
        planPath();
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_pose_ = msg->pose.pose;
    }

    void timerCallback()
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

    bool goalReached()
    {
        double dx = goal_.point.x - robot_pose_.position.x;
        double dy = goal_.point.y - robot_pose_.position.y;
        return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
    }

    void planPath()
    {
        if (!goal_received_ || current_map_.data.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
            return;
        }

        const auto width = current_map_.info.width;
        const auto height = current_map_.info.height;
        const double resolution = current_map_.info.resolution;
        const size_t cell_count = static_cast<size_t>(width) * height;
        if (width == 0 || height == 0 || resolution <= 0.0 || current_map_.data.size() < cell_count)
        {
            RCLCPP_WARN(this->get_logger(), "Cannot plan path: Invalid occupancy grid.");
            return;
        }

        // OccupancyGrid origin is the world pose of the grid's bottom-left corner.
        const auto &origin = current_map_.info.origin;
        const auto &q = origin.orientation;
        const double origin_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                             1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        const double cos_yaw = std::cos(origin_yaw);
        const double sin_yaw = std::sin(origin_yaw);
        const auto world_to_grid = [&](double wx, double wy, int &gx, int &gy) {
            const double dx = wx - origin.position.x;
            const double dy = wy - origin.position.y;
            const double local_x = cos_yaw * dx + sin_yaw * dy;
            const double local_y = -sin_yaw * dx + cos_yaw * dy;
            gx = static_cast<int>(std::floor(local_x / resolution));
            gy = static_cast<int>(std::floor(local_y / resolution));
            return gx >= 0 && gx < static_cast<int>(width) &&
                   gy >= 0 && gy < static_cast<int>(height);
        };

        int start_x, start_y, goal_x, goal_y;
        if (!world_to_grid(robot_pose_.position.x, robot_pose_.position.y, start_x, start_y))
        {
            RCLCPP_WARN(this->get_logger(), "Cannot plan path: Robot is outside the map.");
            return;
        }
        if (!world_to_grid(goal_.point.x, goal_.point.y, goal_x, goal_y))
        {
            RCLCPP_WARN(this->get_logger(), "Cannot plan path: Goal is outside the map.");
            return;
        }

        const auto index_of = [width](int x, int y) {
            return static_cast<size_t>(y) * width + static_cast<size_t>(x);
        };
        const auto traversable = [&](int x, int y, bool is_start) {
            if (x < 0 || y < 0 || x >= static_cast<int>(width) || y >= static_cast<int>(height))
            {
                return false;
            }
            // Unknown cells can be explored with an added cost; occupied and inflated cells are blocked.
            return is_start || current_map_.data[index_of(x, y)] < 50;
        };

        if (!traversable(goal_x, goal_y, false))
        {
            RCLCPP_WARN(this->get_logger(), "Cannot plan path: Goal lies in an occupied cell.");
            return;
        }

        struct QueueEntry
        {
            double f_score;
            double g_score;
            size_t index;
        };
        const auto compare_entries = [](const QueueEntry &a, const QueueEntry &b) {
            return a.f_score > b.f_score;
        };
        std::priority_queue<QueueEntry, std::vector<QueueEntry>, decltype(compare_entries)> open(compare_entries);

        const size_t start = index_of(start_x, start_y);
        const size_t goal = index_of(goal_x, goal_y);
        std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
        std::vector<size_t> came_from(cell_count, cell_count);
        std::vector<bool> closed(cell_count, false);
        const auto heuristic = [goal_x, goal_y](int x, int y) {
            return std::hypot(static_cast<double>(goal_x - x), static_cast<double>(goal_y - y));
        };
        g_score[start] = 0.0;
        open.push({heuristic(start_x, start_y), 0.0, start});

        constexpr std::array<std::pair<int, int>, 8> directions{{
            {1, 0}, {-1, 0}, {0, 1}, {0, -1},
            {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};
        bool found = false;
        while (!open.empty())
        {
            const QueueEntry current = open.top();
            open.pop();
            if (closed[current.index] || current.g_score > g_score[current.index])
            {
                continue;
            }
            if (current.index == goal)
            {
                found = true;
                break;
            }
            closed[current.index] = true;

            const int x = static_cast<int>(current.index % width);
            const int y = static_cast<int>(current.index / width);
            for (const auto &[dx, dy] : directions)
            {
                const int nx = x + dx;
                const int ny = y + dy;
                if (!traversable(nx, ny, false))
                {
                    continue;
                }
                // Prevent diagonal movement through the corner of an obstacle.
                if (dx != 0 && dy != 0 &&
                    (!traversable(x + dx, y, false) || !traversable(x, y + dy, false)))
                {
                    continue;
                }

                const size_t next = index_of(nx, ny);
                if (closed[next])
                {
                    continue;
                }
                const int8_t value = current_map_.data[next];
                const double distance = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
                const double cell_penalty = value < 0 ? 2.0 : 1.0 + static_cast<double>(value) / 100.0;
                const double tentative_g = current.g_score + distance * cell_penalty;
                if (tentative_g < g_score[next])
                {
                    came_from[next] = current.index;
                    g_score[next] = tentative_g;
                    open.push({tentative_g + heuristic(nx, ny), tentative_g, next});
                }
            }
        }

        nav_msgs::msg::Path path;
        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = current_map_.header.frame_id.empty() ? "map" : current_map_.header.frame_id;

        if (!found)
        {
            RCLCPP_WARN(this->get_logger(), "No path found from robot to goal.");
            path_pub_->publish(path);
            return;
        }

        std::vector<size_t> path_cells;
        for (size_t cell = goal; cell != cell_count; cell = came_from[cell])
        {
            path_cells.push_back(cell);
            if (cell == start)
            {
                break;
            }
        }
        if (path_cells.empty() || path_cells.back() != start)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to reconstruct path.");
            path_pub_->publish(path);
            return;
        }
        std::reverse(path_cells.begin(), path_cells.end());

        path.poses.reserve(path_cells.size());
        for (size_t i = 0; i < path_cells.size(); ++i)
        {
            const size_t cell = path_cells[i];
            const int x = static_cast<int>(cell % width);
            const int y = static_cast<int>(cell / width);
            const double local_x = (x + 0.5) * resolution;
            const double local_y = (y + 0.5) * resolution;

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = origin.position.x + cos_yaw * local_x - sin_yaw * local_y;
            pose.pose.position.y = origin.position.y + sin_yaw * local_x + cos_yaw * local_y;
            pose.pose.position.z = 0.0;

            double yaw = 0.0;
            if (i + 1 < path_cells.size())
            {
                const int next_x = static_cast<int>(path_cells[i + 1] % width);
                const int next_y = static_cast<int>(path_cells[i + 1] / width);
                yaw = std::atan2(next_y - y, next_x - x) + origin_yaw;
            }
            else if (i > 0)
            {
                const int previous_x = static_cast<int>(path_cells[i - 1] % width);
                const int previous_y = static_cast<int>(path_cells[i - 1] / width);
                yaw = std::atan2(y - previous_y, x - previous_x) + origin_yaw;
            }
            pose.pose.orientation.z = std::sin(yaw / 2.0);
            pose.pose.orientation.w = std::cos(yaw / 2.0);
            path.poses.push_back(pose);
        }

        path_pub_->publish(path);
    }

    robot::PlannerCore planner_;
};

#endif
