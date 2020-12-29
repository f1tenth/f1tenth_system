cmake_minimum_required(VERSION 3.0.2)
project(f1tenth_racecar)

find_package(catkin REQUIRED rospy)

catkin_package(
  CATKIN_DEPENDS
    ackermann_cmd_mux
    hector_mapping
    joy
    mapping
    robot_pose_ekf
    rosbag
    rostopic
    rviz
    tf2_ros
    tf
    urg_node
)

install(PROGRAMS
  scripts/joy_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
