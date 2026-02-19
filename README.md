# f1tenth_system

Drivers onboard f1tenth race cars. This branch is under development for migration to ROS2. See the [documentation of F1TENTH](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/index.html) on how to get started.

## Cloning submodules
If you clone this repository, make sure to clone the submodules as well. You can do this by running:

```bash
git submodule update --init --recursive --remote
```

This will ensure you have all the submodules cloned and updated to the configured branches.

## Deadman's switch
On Logitech F-710 joysticks, the LB button is the deadman's switch for teleop, and the RB button is the deadman's switch for navigation. You can also remap buttons. See how on the readthedocs documentation.

## Sick Lidar

Note that sick_scan_xd does not have an apt package for ROS2 foxy, you can try installing [from source](https://github.com/SICKAG/sick_scan_xd). Follow these steps to install sick_scan_xd on Linux for ROS 2 Humble:

```
sudo apt update
sudo apt-get install ros-humble-sick-scan-xd
```

**Warning:** Make sure to use the `sick_tim_5xx.launch` file located in the `f1tenth_stack` package's launch directory:

`<f1tenth_stack>/launch/sick_tim_5xx.launch`

This file is an updated version of the original `sick_tim_5xx.launch` file that changes the "frame_id" to "laser" and "tf_base_frame_id" to "base_link", which is compatible with the slam_toolbox and particle filter. You will still need to set the IP address of the lidar in the launch file.

See the [documentation of F1TENTH](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/index.html) on how to get started.

### Getting SICK Lidar IP

The lidar is delivered with a standard IP address. There are two ways to read or change the IP address:

1. **Using a Windows Computer :** Install [SICK SOPAS ET](https://www.sick.com/de/de/sopas-engineering-tool-2018/p/p367244) and follow [the instructions in the sick_scan_xd package](https://github.com/SICKAG/sick_scan_xd?tab=readme-ov-file#starting-with-a-new-lidar).

2. **Using a Linux Computer :** You can scan the network for the lidar IP address using the following command:

```bash
nmap -sn
```

If you have a SICK lidar connected to the network, you will see the IP address of the lidar in the output.

### Configure Ethernet (after you know the LiDAR IP)

Follow the [Hokuyo 10LX Ethernet connection setup](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/firmware_hokuyo10.html#hokuyo-10lx-ethernet-connection-setup). The same networking steps apply to the SICK LiDAR. Use the LiDAR IP you discovered (it may not be `192.168.0.10`).

Configure the Jetson `eth0` connection in the Linux GUI:

1. Open **Network Configuration** and edit the `eth0` connection (or create a new one named `SICK`).
2. In the **IPv4** tab, set **Manual** and add:
   - IP address: a free address on the same subnet as the LiDAR (example: LiDAR `192.168.0.10` -> Jetson `192.168.0.15`)
   - Subnet mask: `255.255.255.0` (or match the LiDAR subnet)
   - Gateway: the LiDAR IP (example: `192.168.0.10`)
3. Save the connection and select it.
4. Plug in the LiDAR and verify connectivity:

```bash
ping <lidar_ip>
```

### Update launch files

1. Open `f1tenth_stack/launch/sick_tim_5xx.launch` and set `<arg name="hostname" default="..."/>` to your LiDAR IP.
2. Open `f1tenth_stack/launch/sick_bringup_launch.py` and confirm `arguments=[...]` points to your local `sick_tim_5xx.launch` file (update the path if needed).

Bring up with `ros2 launch f1tenth_stack sick_bringup_launch.py`.

## Topics

### Topics that the driver stack subscribe to
- `/drive`: Topic for autonomous navigation, uses `AckermannDriveStamped` messages.

### Sensor topics published by the driver stack
- `/scan`: Topic for `LaserScan` messages.
- `/odom`: Topic for `Odometry` messages.
- `/sensors/imu/raw`: Topic for `Imu` messages.
- `/sensors/core`: Topic for telemetry data from the VESC

## External Dependencies

1. ackermann_msgs [https://index.ros.org/r/ackermann_msgs/#humble](https://index.ros.org/r/ackermann_msgs/#humble).
2. urg_node [https://index.ros.org/p/urg_node/#humble](https://index.ros.org/p/urg_node/#humble). This is the driver for Hokuyo LiDARs.
3. joy [https://index.ros.org/p/joy/#humble](https://index.ros.org/p/joy/#humble). This is the driver for joysticks in ROS 2.
4. teleop_tools  [https://index.ros.org/p/teleop_tools/#humble](https://index.ros.org/p/teleop_tools/#humble). This is the package for teleop with joysticks in ROS 2.
5. vesc [GitHub - f1tenth/vesc at ros2](https://github.com/f1tenth/vesc/tree/ros2). This is the driver for VESCs in ROS 2.
6. ackermann_mux [GitHub - f1tenth/ackermann_mux: Twist multiplexer](https://github.com/f1tenth/ackermann_mux). This is a package for multiplexing ackermann messages in ROS 2.
7. sick_scan_xd [https://index.ros.org/p/sick_scan_xd/#humble](https://index.ros.org/p/sick_scan_xd/#humble). This is the driver for SICK LiDARs in ROS 2.
<!-- 8. rosbridge_suite [https://index.ros.org/p/rosbridge_suite/#humble-overview](https://index.ros.org/p/rosbridge_suite/#humble-overview) This is a package that allows for websocket connection in ROS 2. -->

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
