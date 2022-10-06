# f1tenth_system

Drivers onboard f1tenth race cars. This branch is under development for migration to ROS2. See the [documentation of F1TENTH](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/index.html) on how to get started.

### Pre-requisites:
* [Install ROS 2](https://index.ros.org/doc/ros2/Installation/Foxy/)
* Install Navigation2

    ```sudo apt install ros-<ros2_distro>-navigation2 ros-<distro>-nav2-bringup```

* Install SLAM Toolbox

    ```sudo apt install ros-<ros2-distro>-slam-toolbox```
    
* Install Joint State Publisher

    ```sudo apt install ros-<ros2-distro>-joint-state-publisher```
    
* Install Robot Localization

    ```sudo apt install ros-<ros2-distro>-robot-localization```

## Deadman's switch
On Sony Interactive Entertainment Wireless Controller, the LB button is the deadman's switch for teleop, and the RB button is the deadman's switch for navigation. You can also remap buttons. See how on the readthedocs documentation.

## Topics

### Topics that the driver stack subscribe to
- `/drive`: Topic for autonomous navigation, uses `AckermannDriveStamped` messages.

### Sensor topics published by the driver stack
- `/scan`: Topic for `LaserScan` messages.
- `/odom`: Topic for `Odometry` messages.
- `/sensors/imu/raw`: Topic for `Imu` messages.
- `/sensors/core`: Topic for telemetry data from the VESC

## External Dependencies

1. ackermann_msgs [https://index.ros.org/r/ackermann_msgs/#foxy](https://index.ros.org/r/ackermann_msgs/#foxy).
2. sllidar_node [https://github.com/RISE-Dependable-Transport-Systems/sllidar_ros2/tree/main](https://github.com/RISE-Dependable-Transport-Systems/sllidar_ros2/tree/main) This is the driver for SLAMTEC LiDARs.
3. joy [https://index.ros.org/p/joy/#foxy](https://index.ros.org/p/joy/#foxy). This is the driver for joysticks in ROS 2.
4. teleop_tools  [https://index.ros.org/p/teleop_tools/#foxy](https://index.ros.org/p/teleop_tools/#foxy). This is the package for teleop with joysticks in ROS 2.
5. vesc [https://github.com/RISE-Dependable-Transport-Systems/vesc/tree/ros2](https://github.com/RISE-Dependable-Transport-Systems/vesc/tree/ros2). This is the driver for VESCs in ROS 2.
6. ackermann_mux [GitHub - f1tenth/ackermann_mux: Twist multiplexer](https://github.com/f1tenth/ackermann_mux). This is a package for multiplexing ackermann messages in ROS 2.
7. slam_toolbox [https://github.com/SteveMacenski/slam_toolbox/tree/foxy-devel](https://github.com/SteveMacenski/slam_toolbox/tree/foxy-devel). This is a package for SLAM.
8. nav2 [https://github.com/ros-planning/navigation2/tree/foxy-devel](https://github.com/ros-planning/navigation2/tree/foxy-devel). This is a ROS 2 navigation library.
9. joint_state_publisher [https://index.ros.org/p/joint_state_publisher/#foxy](https://index.ros.org/p/joint_state_publisher/#foxy). Package for publishing sensor_msgs/msg/JointState messages for a robot described with URDF.
10. robot_localization [https://index.ros.org/p/robot_localization/#foxy](https://index.ros.org/p/robot_localization/#foxy). Package of nonlinear state estimation nodes.
<!-- 7. rosbridge_suite [https://index.ros.org/p/rosbridge_suite/#foxy-overview](https://index.ros.org/p/rosbridge_suite/#foxy-overview) This is a package that allows for websocket connection in ROS 2. -->

## Package in this repo

1. `f1tenth_stack`: maintains the bringup launch and all parameter files
2. `dts_stack`: maintains control station launch and rover launch, URDF file, all parameter files and a node to convert twist messages to ackermann messages.

## Nodes launched by rover launch

1. joy
2. joy_teleop
3. ackermann_to_vesc_node
4. vesc_to_odom_node
5. vesc_driver_node
6. ackermann_mux
7. sllidar_node

## Nodes launched by control station launch

1. robot_state_publisher
2. joint_state_publisher
3. rviz2
4. ekf_filter_node
5. twist_to_ackermann
6. nav2_controller
7. nav2_planner
8. nav2_recoveries
9. nav2_bt_navigator
10. nav2_waypoint_follower
11. nav2_lifecycle_manager
12. slam_toolbox

## Parameters and topics for dependencies

### twist_to_ackermann

1. Publishes to:
   - drive
2. Subscribes to:
   - cmd_vel
   
### slam_toolbox

1. Publishes to:
   - map
2. Subscribes to:
   - scan

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
   - sensors/imu
   - sensors/imu/raw
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
