#include "ttbot_gui/ros_worker.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

RosWorker::RosWorker() {
    node_ = rclcpp::Node::make_shared("gui_node");
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/mpc_path", 10);
}

void RosWorker::publishPath(const std::vector<std::pair<double, double>>& points) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = node_->now();
    path_msg.header.frame_id = "map"; 

    for (const auto& pt : points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = pt.first;
        pose.pose.position.y = pt.second;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }
    path_pub_->publish(path_msg);
    RCLCPP_INFO(node_->get_logger(), "Published Path: %zu points", points.size());
}