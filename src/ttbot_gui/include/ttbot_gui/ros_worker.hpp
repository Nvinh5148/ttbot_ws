#ifndef ROS_WORKER_HPP
#define ROS_WORKER_HPP

#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vector>
#include <utility>

class RosWorker : public QObject {
    Q_OBJECT
public:
    RosWorker();
    void publishPath(const std::vector<std::pair<double, double>>& points);
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};
#endif