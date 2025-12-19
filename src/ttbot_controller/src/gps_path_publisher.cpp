#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>

const double EARTH_RADIUS = 6378137.0;

class GpsPathPublisher : public rclcpp::Node
{
public:
    GpsPathPublisher() : Node("gps_path_publisher")
    {
        this->declare_parameter("path_file", "path_gps.csv"); 
        this->declare_parameter("frame_id", "map");
        this->declare_parameter("origin_lat", 10.7769);
        this->declare_parameter("origin_lon", 106.7009);

        path_filename_ = this->get_parameter("path_file").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        origin_lat_ = this->get_parameter("origin_lat").as_double();
        origin_lon_ = this->get_parameter("origin_lon").as_double();

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpc_path", 10);
        
        click_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10,
            std::bind(&GpsPathPublisher::clickCallback, this, std::placeholders::_1));

        loadAndConvertPath();

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GpsPathPublisher::timerCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "GPS Path Publisher Started. Origin: [%.4f, %.4f]", origin_lat_, origin_lon_);
    }

private:
    void latLonToXY(double lat, double lon, double &x, double &y)
    {
        double dLat = (lat - origin_lat_) * M_PI / 180.0;
        double dLon = (lon - origin_lon_) * M_PI / 180.0;
        double lat0_rad = origin_lat_ * M_PI / 180.0;

        x = EARTH_RADIUS * dLon * std::cos(lat0_rad);
        y = EARTH_RADIUS * dLat;
    }

    void xyToLatLon(double x, double y, double &lat, double &lon)
    {
        double lat0_rad = origin_lat_ * M_PI / 180.0;
        double dLat = y / EARTH_RADIUS;
        double dLon = x / (EARTH_RADIUS * std::cos(lat0_rad));

        lat = origin_lat_ + (dLat * 180.0 / M_PI);
        lon = origin_lon_ + (dLon * 180.0 / M_PI);
    }

    void clickCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose.position = msg->point;
        pose.pose.orientation.w = 1.0;
        path_msg_.poses.push_back(pose);

        double lat, lon;
        xyToLatLon(msg->point.x, msg->point.y, lat, lon);
        savePointToFile(lat, lon);
        
        RCLCPP_INFO(this->get_logger(), "Added Point: X=%.2f Y=%.2f -> GPS: %.6f %.6f", msg->point.x, msg->point.y, lat, lon);
        
        path_msg_.header.stamp = this->now();
        path_pub_->publish(path_msg_);
    }

    void savePointToFile(double lat, double lon)
    {
        std::string pkg_share = ament_index_cpp::get_package_share_directory("ttbot_controller");
        std::string full_path = pkg_share + "/path/" + path_filename_;
        
        std::ofstream file(full_path, std::ios::app); // Append mode
        if (file.is_open()) {
            file << std::fixed << std::setprecision(7) << lat << ", " << lon << "\n";
            file.close();
        }
    }

    void loadAndConvertPath()
    {
        path_msg_.header.frame_id = frame_id_;
        std::string pkg_share = ament_index_cpp::get_package_share_directory("ttbot_controller");
        std::string full_path = pkg_share + "/path/" + path_filename_;
        
        std::ifstream file(full_path);
        if (!file.is_open()) return; // Không có file cũng không sao

        std::string line;
        while (std::getline(file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string val;
            std::vector<double> row;
            while (std::getline(ss, val, ',')) {
                try { row.push_back(std::stod(val)); } catch (...) {}
            }
            if (row.size() >= 2) {
                double x, y;
                latLonToXY(row[0], row[1], x, y);
                
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = frame_id_;
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.orientation.w = 1.0;
                path_msg_.poses.push_back(pose);
            }
        }
        file.close();
    }

    void timerCallback()
    {
        path_msg_.header.stamp = this->now();
        path_pub_->publish(path_msg_);
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path path_msg_;
    
    std::string path_filename_;
    std::string frame_id_;
    double origin_lat_, origin_lon_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsPathPublisher>());
    rclcpp::shutdown();
    return 0;
}