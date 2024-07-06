#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

class MinRangeNode : public rclcpp::Node
{
public:
    MinRangeNode() : Node("min_range_node")
    {
        // Subscriber to the LiDAR scan topic
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&MinRangeNode::scan_callback, this, std::placeholders::_1));

        // Publisher for the visualization marker
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("min_range_marker", 10);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Find the minimum range
        float min_range = std::numeric_limits<float>::infinity();
        int min_index = -1;
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            if (msg->ranges[i] < min_range && msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max)
            {
                min_range = msg->ranges[i];
                min_index = i;
            }
        }

        // Calculate the position of the minimum range point
        if (min_index != -1)
        {
            float angle = msg->angle_min + min_index * msg->angle_increment;
            float x = min_range * cos(angle);
            float y = min_range * sin(angle);

            // Create a marker message to visualize the minimum range
            auto marker_msg = visualization_msgs::msg::Marker();
            marker_msg.header.frame_id = msg->header.frame_id;
            marker_msg.header.stamp = this->get_clock()->now();
            marker_msg.ns = "min_range";
            marker_msg.id = 0;
            marker_msg.type = visualization_msgs::msg::Marker::ARROW;
            marker_msg.action = visualization_msgs::msg::Marker::ADD;

            // Set the start and end points of the arrow
            marker_msg.points.resize(2);
            marker_msg.points[0].x = 0.0;  // Start point at the origin
            marker_msg.points[0].y = 0.0;
            marker_msg.points[0].z = 0.0;
            marker_msg.points[1].x = x;    // End point at the minimum range point
            marker_msg.points[1].y = y;
            marker_msg.points[1].z = 0.0;

            marker_msg.scale.x = 0.05;  // Shaft diameter
            marker_msg.scale.y = 0.1;   // Head diameter
            marker_msg.scale.z = 0.1;   // Head length

            marker_msg.color.a = 1.0;
            marker_msg.color.r = 1.0;
            marker_msg.color.g = 0.0;
            marker_msg.color.b = 0.0;

            // Publish the marker
            marker_publisher_->publish(marker_msg);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinRangeNode>());
    rclcpp::shutdown();
    return 0;
}
