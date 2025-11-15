#ifndef ACKERMANN_CONTROLLER_HPP
#define ACKERMANN_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>  
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>



class AckermannController : public rclcpp::Node
{
public:
    AckermannController(const std::string & name);

private:
    void velCallback(const geometry_msgs::msg::TwistStamped & msg);
    // callback đọc encoder / joint_states
    void jointCallback(const sensor_msgs::msg::JointState & msg);



    // Sub nhận /cmd_vel
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;

    // Sub nhận /joint_states (từ ROS2 control / encoder)
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

    // Xuất ra ROS2 Control:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rear_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;



    double wheel_radius_;
    double wheel_base_;



    // ===== hỗ trợ tính tốc độ  =====
    double rear_left_prev_pos_;
    double rear_right_prev_pos_;
    rclcpp::Time prev_time_;


    // ===== TRẠNG THÁI ODOMETRY (mô hình xe đạp) =====
    double x_;      // m
    double y_;      // m
    double theta_;  // rad
    
    // message odometry để publish
    nav_msgs::msg::Odometry odom_msg_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    geometry_msgs::msg::TransformStamped transform_stamped_;


};

#endif
