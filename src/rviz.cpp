#include "con_plan_exec_pkg/rviz.h"

void cRVIZ::init_marker(visualization_msgs::msg::Marker& marker, int id, float x, float y, float z, float r, float g, float b, float a, rclcpp::Node* node)
{
    marker.header.frame_id = "map"; // Standard ROS 2 fixed frame
    marker.header.stamp = node->get_clock()->now();
    marker.ns = "grid_workspace";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Scale represents the grid cell
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.1; // Flat grid

    // Position
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;

    // Color
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
}

void cRVIZ::init_rviz(int ws_size_x, int ws_size_y, float_mat ws, rclcpp::Node* node)
{
    ws_x = ws_size_x;
    ws_y = ws_size_y;
    rviz_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("/concpp_node/grid_markers", 10);
    
    update_rviz(ws, node); // Draw the initial state
}

void cRVIZ::update_rviz(float_mat ws, rclcpp::Node* node)
{
    rviz_marker_array.markers.clear();
    int marker_id = 0;

    for (uint i = 0; i < ws_x; i++)
    {
        for (uint j = 0; j < ws_y; j++)
        {
            visualization_msgs::msg::Marker m;
            float val = ws[i][j];

            if (val == 0.0) // Obstacle (Red)
                init_marker(m, marker_id++, i, j, 0, 1.0, 0.0, 0.0, 0.8, node);
            else if (val == 0.5) // Unexplored (Yellow/Transparent)
                init_marker(m, marker_id++, i, j, 0, 1.0, 1.0, 0.0, 0.3, node);
            else if (val == 1.0) // Covered (Green)
                init_marker(m, marker_id++, i, j, 0, 0.0, 1.0, 0.0, 0.8, node);
            else if (val > 1.0) // Robot current location (Blue)
                init_marker(m, marker_id++, i, j, 0.2, 0.0, 0.0, 1.0, 1.0, node);

            if (val != -1) {
                rviz_marker_array.markers.push_back(m);
            }
        }
    }
    rviz_pub->publish(rviz_marker_array);
}
