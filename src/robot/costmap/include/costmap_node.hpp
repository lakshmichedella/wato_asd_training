#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"  // <-- Added for LaserScan
#include "nav_msgs/msg/occupancy_grid.hpp" // <-- Added for OccupancyGrid
#include "std_msgs/msg/header.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node
{
public:
    CostmapNode();

    // Place callback function here
    void publishMessage();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void apply_inflation(std::vector<std::vector<float>> &costmap);
    void publish_occupancy_grid(const std_msgs::msg::Header &scan_header, const std::vector<std::vector<float>> &costmap);

private:
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;

    int grid_size_;  // 30x30 cells
    double resolution_; // 0.1 meters per cell (total 3m x 3m grid)
    double origin_x_;   // 1.5 meters
    double origin_y_;   // 1.5 meters
};

#endif
