// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_ACKERMANN_ACKERMANN_TO_VESC_H_
#define VESC_ACKERMANN_ACKERMANN_TO_VESC_H_

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>

namespace vesc_ackermann
{

class AckermannToVesc
{
public:

  AckermannToVesc(ros::NodeHandle nh, ros::NodeHandle private_nh);

private:
  // ROS parameters
  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;
  double current_vel, vel_diff_thresh;

  /** @todo consider also providing an interpolated look-up table conversion */

  // ROS services
  ros::Publisher erpm_pub_;
  ros::Publisher servo_pub_;
  ros::Publisher brake_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber ackermann_sub_;

  // ROS callbacks
  void ackermannCmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& cmd);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
};

} // namespace vesc_ackermann

#endif // VESC_ACKERMANN_ACKERMANN_TO_VESC_H_
