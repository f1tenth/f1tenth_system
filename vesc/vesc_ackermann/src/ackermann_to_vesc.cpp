// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/ackermann_to_vesc.h"

#include <cmath>
#include <sstream>

#include <std_msgs/Float64.h>

namespace vesc_ackermann
{

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& nh, std::string name, T& value);

AckermannToVesc::AckermannToVesc(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
  // get conversion parameters
  if (!getRequiredParam(nh, "speed_to_erpm_gain", speed_to_erpm_gain_))
    return;
  if (!getRequiredParam(nh, "speed_to_erpm_offset", speed_to_erpm_offset_))
    return;
  if (!getRequiredParam(nh, "steering_angle_to_servo_gain", steering_to_servo_gain_))
    return;
  if (!getRequiredParam(nh, "steering_angle_to_servo_offset", steering_to_servo_offset_))
    return;

  current_vel = 0.0;
  vel_diff_thresh = 0.3;

  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = nh.advertise<std_msgs::Float64>("commands/motor/speed", 10);
  servo_pub_ = nh.advertise<std_msgs::Float64>("commands/servo/position", 10);
  brake_pub_ = nh.advertise<std_msgs::Float64>("commands/motor/brake", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = nh.subscribe("ackermann_cmd", 10, &AckermannToVesc::ackermannCmdCallback, this);
  odom_sub_ = nh.subscribe("odom", 10 , &AckermannToVesc::odomCallback, this);
}

typedef ackermann_msgs::AckermannDriveStamped::ConstPtr AckermannMsgPtr;
void AckermannToVesc::ackermannCmdCallback(const AckermannMsgPtr& cmd)
{
  double commanded_vel = cmd->drive.speed;

  // calc vesc electric RPM (speed)
  std_msgs::Float64::Ptr erpm_msg(new std_msgs::Float64);
  erpm_msg->data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;

  // calc steering angle (servo)
  std_msgs::Float64::Ptr servo_msg(new std_msgs::Float64);
  servo_msg->data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  // brake msg
  std_msgs::Float64::Ptr brake_msg(new std_msgs::Float64);
  brake_msg->data = 20000;

  // publish
  if (ros::ok()) {
    if (commanded_vel > 0 && current_vel > commanded_vel + vel_diff_thresh) {
      brake_pub_.publish(brake_msg);
      servo_pub_.publish(servo_msg);
    } else if (commanded_vel < 0 && current_vel < commanded_vel - vel_diff_thresh) {
      brake_pub_.publish(brake_msg);
      servo_pub_.publish(servo_msg);
    } else {
      erpm_pub_.publish(erpm_msg);
      servo_pub_.publish(servo_msg);
    }
  }
}

void AckermannToVesc::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  current_vel = odom_msg->twist.twist.linear.x;
}

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& nh, std::string name, T& value)
{
  if (nh.getParam(name, value))
    return true;

  ROS_FATAL("AckermannToVesc: Parameter %s is required.", name.c_str());
  return false;
}

} // namespace vesc_ackermann
