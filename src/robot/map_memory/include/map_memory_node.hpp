#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <chrono>
#include <cmath>
#include <functional>

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node
{
public:
    MapMemoryNode() : Node("mapping_node"), last_x(0.0), last_y(0.0), distance_threshold(0.1)
    {
        global_map_.header.frame_id = "odom";
        global_map_.info.resolution = 0.1;
        global_map_.info.width = 200;
        global_map_.info.height = 200;
        global_map_.info.origin.position.x = -10.0;
        global_map_.info.origin.position.y = -10.0;
        global_map_.info.origin.orientation.w = 1.0;

        // FIX: Initialize all 40,000 cells to -1 (Unknown space)
        global_map_.data.resize(200 * 200, -1);
        // Initialize subscribers
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

        // Initialize publisher
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

        // Initialize timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
    }

private:
    // Subscribers and Publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Global map and robot position
    nav_msgs::msg::OccupancyGrid global_map_;
    double last_x, last_y;
    const double distance_threshold;
    bool costmap_updated_ = false;

    // Callback for costmap updates
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        // Store the latest costmap
        latest_costmap_ = *msg;
        costmap_updated_ = true;
    }

    // Callback for odometry updates
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        // Compute distance traveled
        double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
        if (distance >= distance_threshold)
        {
            last_x = x;
            last_y = y;
            should_update_map_ = true;
        }
    }

    // Timer-based map update
    void updateMap()
    {
        if (should_update_map_ && costmap_updated_)
        {
            integrateCostmap();
            map_pub_->publish(global_map_);
            should_update_map_ = false;
        }
    }

    // Integrate the latest costmap into the global map
    void integrateCostmap()
    {
        int local_w = latest_costmap_.info.width;
        int local_h = latest_costmap_.info.height;
        const double local_res = latest_costmap_.info.resolution;

        int global_w = global_map_.info.width;
        int global_h = global_map_.info.height;
        double global_res = global_map_.info.resolution;

        double global_origin_x = global_map_.info.origin.position.x;
        double global_origin_y = global_map_.info.origin.position.y;
        double local_origin_x = latest_costmap_.info.origin.position.x;
        double local_origin_y = latest_costmap_.info.origin.position.y;

        if (local_res <= 0.0 || global_res <= 0.0 ||
            latest_costmap_.data.size() < static_cast<size_t>(local_w) * local_h)
        {
            RCLCPP_WARN(this->get_logger(), "Ignoring invalid costmap metadata or data length.");
            return;
        }

        for (int ly = 0; ly < local_h; ++ly)
        {
            for (int lx = 0; lx < local_w; ++lx)
            {
                int local_flat_idx = ly * local_w + lx;
                int8_t cell_value = latest_costmap_.data[local_flat_idx];

                // OccupancyGrid reserves -1 for unknown; all known observations replace old data.
                if (cell_value < 0)
                {
                    continue;
                }

                // Transform the local cell center into world coordinates.
                double lx_m = local_origin_x + (lx + 0.5) * local_res;
                double ly_m = local_origin_y + (ly + 0.5) * local_res;
                double gx_m = last_x + lx_m;
                double gy_m = last_y + ly_m;

                int gx_idx = static_cast<int>(std::floor((gx_m - global_origin_x) / global_res));
                int gy_idx = static_cast<int>(std::floor((gy_m - global_origin_y) / global_res));

                if (gx_idx >= 0 && gx_idx < global_w && gy_idx >= 0 && gy_idx < global_h)
                {
                    int global_flat_idx = gy_idx * global_w + gx_idx;
                    // Known observations replace older data; unknown cells were skipped above.
                    global_map_.data[global_flat_idx] = cell_value;
                }
            }
        }
    }

    // Flags
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool should_update_map_ = true;
};

#endif
