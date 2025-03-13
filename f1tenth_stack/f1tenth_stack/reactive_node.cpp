#include <string>
#include <vector>
#include <cmath>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class ReactiveFollowGap : public rclcpp::Node {
public:
    ReactiveFollowGap() : Node("reactive_follow_gap") {
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, _1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ReactiveFollowGap::publish_drive_command, this));
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/ackermann_cmd";
    std::vector<double> processed_lidar;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    double last_steering_angle = 0.0;
    double last_speed = 2.0;

    double clamp_value(double value, double min_val, double max_val) {
        return std::max(min_val, std::min(value, max_val));
    }

    void preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        processed_lidar.clear();
        for (double range : scan_msg->ranges) {
            processed_lidar.push_back(range > 3.0 ? 0.0 : range);
        }
    }

    int find_closest_point() {
        int closest_idx = 0;
        double min_distance = processed_lidar[0];
        for (size_t i = 1; i < processed_lidar.size(); i++) {
            if (processed_lidar[i] > 0 && processed_lidar[i] < min_distance) {
                min_distance = processed_lidar[i];
                closest_idx = i;
            }
        }
        return closest_idx;
    }

    void eliminate_bubble(int closest_idx, double bubble_radius, double angle_increment) {
        int bubble_size = static_cast<int>(bubble_radius / angle_increment);
        for (int i = std::max(0, closest_idx - bubble_size); i < std::min((int)processed_lidar.size(), closest_idx + bubble_size); i++) {
            processed_lidar[i] = 0;
        }
    }

    std::pair<int, int> find_max_gap() {
        int max_start = 0, max_length = 0;
        int current_start = 0, current_length = 0;

        for (size_t i = 0; i < processed_lidar.size(); i++) {
            if (processed_lidar[i] > 0.5) {
                current_length++;
                if (current_length > max_length) {
                    max_length = current_length;
                    max_start = current_start;
                }
            } else {
                current_start = i + 1;
                current_length = 0;
            }
        }
        return std::make_pair(max_start, max_length);
    }

    int find_best_point(int start_idx, int gap_length) {
        int best_idx = start_idx;
        double max_distance = 0;
        for (int i = start_idx; i < start_idx + gap_length; i++) {
            if (processed_lidar[i] > max_distance) {
                max_distance = processed_lidar[i];
                best_idx = i;
            }
        }
        return best_idx;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        preprocess_lidar(scan_msg);
        int closest_idx = find_closest_point();
        eliminate_bubble(closest_idx, 0.3, scan_msg->angle_increment);

        auto gap = find_max_gap();
        int best_idx = find_best_point(gap.first, gap.second);

        last_steering_angle = scan_msg->angle_min + best_idx * scan_msg->angle_increment;
        last_steering_angle = clamp_value(last_steering_angle, -0.4, 0.4);
    }

    void publish_drive_command() {
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = last_steering_angle;
        drive_msg.drive.speed = last_speed;
        publisher_->publish(drive_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
