# f1tenth_system
Drivers onboard f1tenth race cars. This branch is under development for migration to ROS2.

## External Dependencies
1. ackermann_msgs [https://index.ros.org/r/ackermann_msgs/#foxy](https://index.ros.org/r/ackermann_msgs/#foxy)
2. urg_node [https://index.ros.org/p/urg_node/#foxy](https://index.ros.org/p/urg_node/#foxy)
3. joy [https://index.ros.org/p/joy/#foxy](https://index.ros.org/p/joy/#foxy)
4. joy_teleop [https://index.ros.org/p/joy_teleop/#foxy](https://index.ros.org/p/joy_teleop/#foxy)
3. vesc? not external right now but will be listed on ROS index [TODO](TODO)
4. rosbridge_suite [https://index.ros.org/p/rosbridge_suite/#foxy-overview](https://index.ros.org/p/rosbridge_suite/#foxy-overview)

## Included packages
1. f1tenth_stack: has the throttle interpolator and all bringup launch scripts and launch configs
2. ackermann_mx: modified from twist_mux
3. vesc: will be external when listed on ROS index

## Nodes launched in bringup
1. joy
2. joy_teleop
3. ackermann_to_vesc_node
4. vesc_to_odom_node
5. vesc_driver_node
6. throttle_interpolator.py
8. urg_node
9. ackermann_mux
10. rosbridge_websocket.launch

## TODOs
- [x] port the bringup package to ROS2
- [ ] finish vesc imu implementation
- [x] test urg_node on car
- [x] test joy on car
- [ ] test bringup launch on car
- [ ] test foxglove studio integration over rosbridge

## Notes and Gotchas
- joy_teleop installed through rosdep/apt has a bug where the stamp is not using the correct format, clone from the teleop_tools repo on foxy-devel branch and put it in f1tenth_stack/ so it works correctly.
- Testing results 09/24/21: mux is working but not correctly, when msgs published to /drive, the deadman switch doesn't work. Deadman switch for teleop works. Odom seems to be not published? 

## Extra doc to be put in other repos
### vesc_driver
1. Parameters:
    - duty_cycle_min, duty_cycle_max
    - current_min, current_max
    - brake_min, brake_max
    - speed_min, speed_max
    - position_min, position_max
    - servo_min, servo_max
2. Publishes to:
    - sensors/core
    - sensors/servo_position_command
3. Subscribes to:
    - commands/motor/duty_cycle
    - commands/motor/current
    - commands/motor/brake
    - commands/motor/speed
    - commands/motor/position
    - commands/servo/position

### ackermann_to_vesc
1. Parameters:
    - speed_to_erpm_gain
    - speed_to_erpm_offset
    - steering_angle_to_servo_gain
    - steering_angle_to_servo_offset
2. Publishes to:
    - ackermann_cmd
3. Subscribes to:
    - commands/motor/speed
    - commands/servo/position

### vesc_to_odom
1. Parameters:
    - odom_frame
    - base_frame
    - use_servo_cmd_to_calc_angular_velocity
    - speed_to_erpm_gain
    - speed_to_erpm_offset
    - steering_angle_to_servo_gain
    - steering_angle_to_servo_offset
    - wheelbase
    - publish_tf
2. Publishes to:
    - odom
3. Subscribes to:
    - sensors/core
    - sensors/servo_position_command

### throttle_interpolator
1. Parameters:
    - rpm_input_topic
    - rpm_output_topic
    - servo_input_topic
    - servo_output_topic
    - max_acceleration
    - speed_max
    - speed_min
    - throttle_smoother_rate
    - speed_to_erpm_gain
    - max_servo_speed
    - steering_angle_to_servo_gain
    - servo_smoother_rate
    - servo_max
    - servo_min
    - steering_angle_to_servo_offset
2. Publishes to:
    - topic described in rpm_output_topic
    - topic described in servo_output_topic
3. Subscribes to:
    - topic described in rpm_input_topic
    - topic described in servo_input_topic