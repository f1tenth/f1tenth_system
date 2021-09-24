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
- 