#include <memory>
#include <string>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono;

/* This class converts <geometry_msgs::msg::Twist> messages into <ackermann_msgs::msg::AckermannDriveStamped> */

class TwistToAckermann : public rclcpp::Node
{
  public:
    TwistToAckermann()
    : Node("twist_to_ackermann")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&TwistToAckermann::topic_callback, this, _1));
      
      publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/drive", 10);
    }

  private:
    // Assumes base_link to be located at the center of rotation
    // Wheelbase in meters
    float convert_trans_rot_vel_to_steering_angle(float v, float omega, float wheelbase) const
    {
      if (omega == 0 || v == 0) 
        return 0;
      float radius = v / omega;    
      return atan(wheelbase / radius);
    }
  
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr twi_msg) const
    {
      RCLCPP_INFO(this->get_logger(), "\nLinear:\n x: %f""\nAngular:\n z: %f", twi_msg->linear.x, twi_msg->angular.z);
      
      auto ack_msg = ackermann_msgs::msg::AckermannDriveStamped();
      ack_msg.header.stamp.sec = duration_cast<seconds>(system_clock::now().time_since_epoch()).count();
      ack_msg.header.stamp.nanosec = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
      ack_msg.header.frame_id = "odom";
      ack_msg.drive.speed = 4*twi_msg->linear.x;
      ack_msg.drive.steering_angle = convert_trans_rot_vel_to_steering_angle(twi_msg->linear.x, twi_msg->angular.z, 0.33);
      publisher_->publish(ack_msg);
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToAckermann>());
  rclcpp::shutdown();
  return 0;
}
