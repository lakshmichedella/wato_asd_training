#include <chrono>
#include <memory>

#include "costmap_node.hpp"
using std::placeholders::_1;

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
    // 1. SAFE REQUIREMENT: Initialize parameters FIRST before starting data feeds
    grid_size_ = 30;   // 30x30 cells
    resolution_ = 0.1; // 0.1 meters per cell (total 3m x 3m grid)

    // Origin places the robot in the exact center of the grid (index 15, 15)
    origin_x_ = (grid_size_ * resolution_) / 2.0; // 1.5 meters
    origin_y_ = (grid_size_ * resolution_) / 2.0; // 1.5 meters

    // 2. Initialize the publishers and subscribers after variables are safe
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

    // NOTE: If you still see nothing after compiling this, swap out the line below with:
    // sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", rclcpp::SensorDataQoS(), std::bind(&CostmapNode::laserCallback, this, _1));
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Lidar Subscriber Node has been initialized.");
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    // Explicit constants inside the scope to prevent hidden header rounding bugs
    const int MAP_SIZE = 30;
    const double MAP_RES = 0.1;
    const double OFFSET_X = 1.5;
    const double OFFSET_Y = 1.5;

    float min_angle = scan->angle_min;
    float angle_step = scan->angle_increment;

    // Create a fresh clean 30x30 local costmap matrix frame
    std::vector<std::vector<float>> costmap(MAP_SIZE, std::vector<float>(MAP_SIZE, 0.0f));

    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        double range = scan->ranges[i];

        // Loosened filter: explicitly check standard boundaries
        if (std::isnan(range) || std::isinf(range) || range <= scan->range_min || range >= scan->range_max)
        {
            continue;
        }

        double angle = min_angle + (static_cast<double>(i) * angle_step);

        // Calculate continuous Cartesian coordinates in meters
        double x_meters = range * std::cos(angle);
        double y_meters = range * std::sin(angle);

        // Convert meters to discrete integer grid indices
        int grid_x = static_cast<int>(std::floor((x_meters + OFFSET_X) / MAP_RES));
        int grid_y = static_cast<int>(std::floor((y_meters + OFFSET_Y) / MAP_RES));

        // Boundary verification shield
        if (grid_x >= 0 && grid_x < MAP_SIZE && grid_y >= 0 && grid_y < MAP_SIZE)
        {
            costmap[grid_y][grid_x] = 100.0f;
        }
    }

    // Apply the 1-meter decay inflation layer bounds
    apply_inflation(costmap);

    // Forward the compiled map out to Foxglove Studio
    publish_occupancy_grid(scan->header, costmap);
}

void CostmapNode::apply_inflation(std::vector<std::vector<float>> &costmap)
{
    // Hardcoded local constants to match the callback geometry
    const int MAP_SIZE = 30;
    const double MAP_RES = 0.1;
    const float inflation_radius = 1.0f; // 1 meter radius
    const float max_cost = 100.0f;       // Peak cost at the obstacle core

    // Create a copy of the raw grid to safely read base obstacle locations
    std::vector<std::vector<float>> raw_obstacles = costmap;

    // Convert 1.0m radius into cell units: 1.0m / 0.1m = 10 cells in each direction
    int cell_radius = static_cast<int>(std::ceil(inflation_radius / MAP_RES));

    // Loop through every cell in the 30x30 grid to look for core obstacles
    for (int y = 0; y < MAP_SIZE; ++y)
    {
        for (int x = 0; x < MAP_SIZE; ++x)
        {
            // If this cell is a raw obstacle core
            if (raw_obstacles[y][x] == 100.0f)
            {
                // Search a bounding box of surrounding neighbor cells
                for (int ny = y - cell_radius; ny <= y + cell_radius; ++ny)
                {
                    for (int nx = x - cell_radius; nx <= x + cell_radius; ++nx)
                    {
                        // Ensure neighbor cell stays within the 30x30 array bounds
                        if (nx >= 0 && nx < MAP_SIZE && ny >= 0 && ny < MAP_SIZE)
                        {
                            // Calculate Euclidean distance in meters
                            float dx = (nx - x) * MAP_RES;
                            float dy = (ny - y) * MAP_RES;
                            float distance = std::sqrt(dx * dx + dy * dy);

                            // Do not assign a cost to cells beyond the inflation radius
                            if (distance <= inflation_radius)
                            {
                                // Linear decay cost formula matching your requirements
                                float calculated_cost = max_cost * (1.0f - (distance / inflation_radius));

                                // Only assign if calculated cost is higher than the cell's current value
                                if (calculated_cost > costmap[ny][nx])
                                {
                                    costmap[ny][nx] = calculated_cost;
                                }
                            }
                        }
                    }
                }
            } // end obstacle check
        }
    }
}

void CostmapNode::publish_occupancy_grid(const std_msgs::msg::Header &scan_header, const std::vector<std::vector<float>> &costmap)
{
    auto grid_msg = nav_msgs::msg::OccupancyGrid();

    // Match the message timestamp and frame to your incoming scanner coordinates
    grid_msg.header.stamp = scan_header.stamp;
    grid_msg.header.frame_id = scan_header.frame_id; // Usually "laser_frame" or "robot"

    // Configure metadata details
    grid_msg.info.resolution = resolution_;
    grid_msg.info.width = grid_size_;
    grid_msg.info.height = grid_size_;

    // Shift origin map boundary configuration so (0,0) is centered relative to physical metrics
    grid_msg.info.origin.position.x = -origin_x_;
    grid_msg.info.origin.position.y = -origin_y_;
    grid_msg.info.origin.position.z = 0.0;
    grid_msg.info.origin.orientation.w = 1.0; // Identity quaternion assignment
    // Flatten the 2D vector matrix array layout down into a 1D row-major array sequence
    grid_msg.data.resize(grid_size_ * grid_size_);
    for (int y = 0; y < grid_size_; ++y)
    {
        for (int x = 0; x < grid_size_; ++x)
        {
            int index = y * grid_size_ + x;

            // OccupancyGrid message values are explicit signed integers (0-100)
            grid_msg.data[index] = static_cast<int8_t>(costmap[y][x]);
        }
    }

    costmap_pub_->publish(grid_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}