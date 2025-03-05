![image](https://github.com/user-attachments/assets/df4ae9c0-bf45-48c3-923c-35cd488ebeab)# How to run car (capstone instructions):
1. create local hotspot: on laptop go to Settings/Wifi then click the three dots in the upper right hand corner and click 'Turn on wifi hotspot'
2. plug in and connect jetson to the monitor and join the hotspot (f1net). run the command line 'hostname -I' on the jetson to find the new IP address. 
3. (optional) to access code on the jetson without a monitor: ssh into the jetson using VScode on the laptop. on VScode press f1, then click 'SSH: Connect to host', type in f1jetson@my-jetson or f1jetson@ip_address (e.g. f1jetson@10.42.0.153)
4. ssh into jetson on the laptop in command window (ssh f1jetson@my-jetson). note: the source /opt/ros/foxy/setup.bash and source ~/f1tenth_ws/install/setup.bash are added to the .bashrc file so no need to run those commnads.
5. To launch the system, on jetson terminal: ros2 launch f1tenth_stack bringup_launch.py
6. on laptop terminal: ros2 run joy joy_node.
  a. to move car: hold lb (top left button, the 'dead man' switch) at all times. use the left stick to go forwards/backwards and the right stick to steer left/right. its clunky. 
8. to launch rviz: on laptop terminal rune: rviz2.
  a.  to add LiDAR point cloud: click add (bottom left corner), then click LaserScan. Expand LaserScan. In the topic box write /scan. under globaloptions/fixed frame change it laser.  its useful to increase the size of to 0.03 (under LaserScan/Size (m)).
9. to run SLAM:
   a. on laptop terminal run: runslam (this is an alias/short cut for a longer command line, view bashrc file to see the full command line e.g. ros2 launch slam_toolbox online_async_launch.py params_file...)
   b. on Rviz: unclick LaserScan. Add Map (botton left corner), under topic select /map. under fixed frame click map. it looks slightly nice if you click the Type drop down menu under Views (right panel), then click TopDownOrtho.
   c. to save map: on laptop terminal: ros2 launch nav2_map_server map_saver_server.launch.py,
   then run: ros2 run nav2_map_server map_saver_cli -f /home/capstone/f1host_ws/src/f1tenth_stack/map/_map_name_
   d. to clean up map: click/look up GNU Image Manipulation Program, then find the map in the workspace, right click the paintbrush to select pencil (for hard edges), and color picker to select grey or black colours.

   
   
ros2 run nav2_map_server --ros-args -p yaml_filename:=test303.yaml -p use_sim_time:=false
ros2 run nav2_util lifecycle_bringup map_server

ros2 run nav2_amcl amcl --ros-args -p base_frame_id:=base_link use_sim_time:=false
ros2 run nav2_util lifecycle_bringup amcl


# From original source: f1tenth_system

Drivers onboard f1tenth race cars. This branch is under development for migration to ROS2. See the [documentation of F1TENTH](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/index.html) on how to get started.

## Deadman's switch
On Logitech F-710 joysticks, the LB button is the deadman's switch for teleop, and the RB button is the deadman's switch for navigation. You can also remap buttons. See how on the readthedocs documentation.

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
2. urg_node [https://index.ros.org/p/urg_node/#foxy](https://index.ros.org/p/urg_node/#foxy). This is the driver for Hokuyo LiDARs.
3. joy [https://index.ros.org/p/joy/#foxy](https://index.ros.org/p/joy/#foxy). This is the driver for joysticks in ROS 2.
4. teleop_tools  [https://index.ros.org/p/teleop_tools/#foxy](https://index.ros.org/p/teleop_tools/#foxy). This is the package for teleop with joysticks in ROS 2.
5. vesc [GitHub - f1tenth/vesc at ros2](https://github.com/f1tenth/vesc/tree/ros2). This is the driver for VESCs in ROS 2.
6. ackermann_mux [GitHub - f1tenth/ackermann_mux: Twist multiplexer](https://github.com/f1tenth/ackermann_mux). This is a package for multiplexing ackermann messages in ROS 2.
<!-- 7. rosbridge_suite [https://index.ros.org/p/rosbridge_suite/#foxy-overview](https://index.ros.org/p/rosbridge_suite/#foxy-overview) This is a package that allows for websocket connection in ROS 2. -->

## Package in this repo

1. f1tenth_stack: maintains the bringup launch and all parameter files

## Nodes launched in bringup

1. joy
2. joy_teleop
3. ackermann_to_vesc_node
4. vesc_to_odom_node
5. vesc_driver_node
6. urg_node
7. ackermann_mux

## Parameters and topics for dependencies

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
