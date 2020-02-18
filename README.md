# driver_base
You'll also need the driver_base package, , you could get this package via:

```sudo apt-get install ros-<distro>-driver-base```

# f110_system
Code/Drivers onboard f110 race cars.

## ackermann_msgs
The ROS message definitions for ackermann steering.

## hokuyo_node
The driver for Hokuyo 10LX and Hokuyo 30LX.

## joystick_drivers
The driver for Linux compatible joysticks

## racecar
The package including launch files handling starting the car, and the parameters for Odometry tuning, motor/servo settings.

## serial
A cross-platform library for interfacing with rs-232 serial like ports written in C++.

## vesc
The package handling communication with the VESC 6 Plus.

## waypoint_logger
The node that records the car's current position in the world, requires particle_filter to work.
