#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>

using namespace std;
typedef vector<vector<float>> float_mat;

class cRVIZ
{
public:
    uint ws_x, ws_y;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_pub;
    visualization_msgs::msg::MarkerArray rviz_marker_array;

    // Pass the node pointer so the class can use the ROS 2 clock and logger
    void init_rviz(int ws_size_x, int ws_size_y, float_mat ws, rclcpp::Node* node);
    void update_rviz(float_mat ws, rclcpp::Node* node);

private:
    void init_marker(visualization_msgs::msg::Marker& marker, int id, float x, float y, float z, float r, float g, float b, float a, rclcpp::Node* node);
};
