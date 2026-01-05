#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <cmath>
#include <vector>

class PathPublisher : public rclcpp::Node
{
public:
    PathPublisher() : Node("path_publisher")
    {
        // Khai báo tham số (có thể chỉnh trong launch file)
        this->declare_parameter("frame_id", "map"); 
        
        // --- CẤU HÌNH QOS (QUAN TRỌNG) ---
        // Transient Local: Giúp tin nhắn được lưu lại cho các node vào sau (như Rviz) đọc được
        // Dù ta chỉ publish 1 lần, Rviz vẫn sẽ hiện đường đi.
        rclcpp::QoS qos_profile(10);
        qos_profile.transient_local(); 
        qos_profile.reliable();

        // Tạo Publisher
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_path", qos_profile);

        // Tạo dữ liệu đường đi (Đi thẳng -> Rẽ phải)
        generatePath();

        // Tạo Timer chạy 1 lần sau 1 giây (để đảm bảo kết nối ổn định rồi mới gửi)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PathPublisher::timerCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "Path Publisher Initialized. Waiting to publish ONCE...");
    }

private:
    void generatePath()
    {
        std::string frame_id = this->get_parameter("frame_id").as_string();
        path_msg_.header.frame_id = frame_id;

        // --- PHẦN 1: ĐI THẲNG 10 MÉT ---
        // Từ (0,0) đến (10,0)
        double straight_length = 10.0;
        int straight_points = 200; // Số lượng điểm càng lớn đường càng mịn

        for (int i = 0; i < straight_points; ++i) {
            double x = (double)i / (straight_points - 1) * straight_length;
            double y = 0.0;
            addPointToPath(x, y);
        }

        // --- PHẦN 2: RẼ PHẢI (CUNG TRÒN) ---
        // Điểm nối tiếp là (10, 0)
        // Ta muốn rẽ phải 90 độ với bán kính R = 5m
        // Tâm quay sẽ nằm tại (10, -5) -> Vì bên Phải là trục Y âm
        
        double radius = 5.0; 
        int turn_points = 100;
        
        double center_x = straight_length; // 10.0
        double center_y = -radius;         // -5.0

        for (int i = 0; i < turn_points; ++i) {
            // Góc t chạy từ 90 độ (PI/2) về 0 độ
            // Tại 90 độ: x = 10 + 0, y = -5 + 5 = 0 (Khớp điểm cuối đoạn thẳng)
            // Tại 0 độ:  x = 10 + 5, y = -5 + 0 = -5 (Đã rẽ xong)
            double angle_start = M_PI / 2.0;
            double angle_end = 0.0;
            
            double t = angle_start - ((double)i / (turn_points - 1)) * (angle_start - angle_end);

            double x = center_x + radius * std::cos(t);
            double y = center_y + radius * std::sin(t);

            addPointToPath(x, y);
        }

        RCLCPP_INFO(this->get_logger(), "Path Created: Straight 10m -> Turn Right. Total points: %zu", path_msg_.poses.size());
    }

    void addPointToPath(double x, double y)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = path_msg_.header.frame_id;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        
        // Orientation mặc định (MPC sẽ tự tính hướng dựa trên vector giữa các điểm)
        pose.pose.orientation.w = 1.0; 
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;

        path_msg_.poses.push_back(pose);
    }

    void timerCallback()
    {
        if (!path_msg_.poses.empty()) {
            // Cập nhật thời gian thực tế lúc gửi
            path_msg_.header.stamp = this->now();
            
            // Gửi Path đi
            path_pub_->publish(path_msg_);
            
            RCLCPP_INFO(this->get_logger(), "Path PUBLISHED successfully! (Straight -> Right)");
        } else {
            RCLCPP_WARN(this->get_logger(), "Path is empty, nothing to publish.");
        }

        // --- QUAN TRỌNG: HỦY TIMER ĐỂ KHÔNG GỬI LẠI ---
        timer_->cancel();
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path path_msg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}