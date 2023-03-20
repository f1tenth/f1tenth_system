# launch file to bring up rover nodes

import os
import yaml

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def generate_launch_description():
    # get the directories
    dts_dir = get_package_share_directory('dts_stack')
    realsense_camera_dir = get_package_share_directory('realsense2_camera')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # args that can be set from the command line
    model = LaunchConfiguration('model')
    ekf_config = LaunchConfiguration('ekf_config')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    realsense_camera_config_file = LaunchConfiguration('realsense_camera_config_file')
    
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    lidar_serial_baudrate = LaunchConfiguration('lidar_serial_baudrate') 
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    lidar_inverted = LaunchConfiguration('lidar_inverted')
    lidar_angle_compensate = LaunchConfiguration('lidar_angle_compensate')
    
    # args that can be set from the command line or a default will be used
    robot_state_publisher_la = DeclareLaunchArgument(
        'model', default_value=os.path.join(dts_dir, 'urdf/box_car_camera.urdf'),
        description='Full path to robot urdf file')
    robot_localization_la = DeclareLaunchArgument(
        'ekf_config', default_value=os.path.join(dts_dir, 'config/ekf.yaml'),
        description='Full path to ekf config file')        
    vesc_la = DeclareLaunchArgument(
        'vesc_config', default_value=os.path.join(dts_dir, 'config/vesc.yaml'),
        description='Full path to params file')
    mux_la = DeclareLaunchArgument(
        'mux_config', default_value=os.path.join(dts_dir, 'config/mux.yaml'),
        description='Full path to params file')
    nav2_la = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(dts_dir, 'config/slam_nav2_params.yaml'),
        description='Full path to nav2 params file')
    realsense_camera_la = DeclareLaunchArgument(
        'realsense_camera_config_file', default_value=os.path.join(dts_dir, 'config/realsense_camera.yaml')
        )
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Use simulation/Gazebo clock')
    lidar_serial_port_la = DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB0')
    lidar_serial_baudrate_la = DeclareLaunchArgument('lidar_serial_baudrate', default_value='256000')
    lidar_frame_id_la = DeclareLaunchArgument('lidar_frame_id', default_value='laser')
    lidar_inverted_la = DeclareLaunchArgument('lidar_inverted', default_value='false')
    lidar_angle_compensate_la = DeclareLaunchArgument('lidar_angle_compensate', default_value='true')
        
    # include launch files
    realsense_camera_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dts_dir, 'launch/rs_camera_launch.py')),
        launch_arguments = {'camera_name':'camera',
                            'log_level':'INFO',
                            'config_file':realsense_camera_config_file
                            }.items()
    )
    slam_toolbox_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(slam_toolbox_dir, 'launch/online_async_launch.py')]),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time
            }.items()
    )
    nav2_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch/navigation_launch.py')]),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time
            }.items()
    )
    
    # start nodes and use args to set parameters
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )    
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )    
    map_odom_static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_static_tf_node',
        arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"]
    )
    odom_base_link_static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_base_link_static_tf_node',
        arguments = ["0", "0", "0", "0", "0", "0", "odom", "base_link"]
    )
    depthimage_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_node',
        remappings=[
            ('depth', '/depth/image_rect_raw'),
            ('depth_camera_info', '/depth/camera_info'),            
            ('/scan', '/scan_depth_camera')
            ]        
        ,parameters=[{
            'scan_height': 1,
            'scan_time': 0.033,
            'range_min': 0.02,
            'range_max': 20.0,
            'output_frame': "camera_depth_frame"
        }],
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', model]),
            'use_sim_time': use_sim_time,
        }]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'ekf_config': ekf_config,
            'use_sim_time': use_sim_time
        }]
    )
    twist_to_ackermann_node = Node(
        package='dts_stack',
        executable='twist_to_ackermann',
        name='twist_to_ackermann',
        output='screen'
    )   
    
    lidar_serial_port_la = DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB0')
    lidar_serial_baudrate_la = DeclareLaunchArgument('lidar_serial_baudrate', default_value='256000')
    lidar_frame_id_la = DeclareLaunchArgument('lidar_frame_id', default_value='laser')
    lidar_inverted_la = DeclareLaunchArgument('lidar_inverted', default_value='false')
    lidar_angle_compensate_la = DeclareLaunchArgument('lidar_angle_compensate', default_value='true')
    
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{'serial_port': lidar_serial_port, 
            'serial_baudrate': lidar_serial_baudrate, 
            'frame_id': lidar_frame_id,
            'inverted': lidar_inverted, 
            'angle_compensate': lidar_angle_compensate}],
        output='screen',
        remappings=[         
            ('/scan', '/scan_lidar')
            ]
    )
    
    # create launch description
    ld = LaunchDescription()
    
    # declare launch args
    ld.add_action(robot_state_publisher_la)
    ld.add_action(robot_localization_la)
    ld.add_action(use_sim_time_la)    
    ld.add_action(nav2_la)
    ld.add_action(vesc_la)
    ld.add_action(mux_la)
    ld.add_action(realsense_camera_la)
    ld.add_action(lidar_serial_port_la)
    ld.add_action(lidar_serial_baudrate_la)
    ld.add_action(lidar_frame_id_la)
    ld.add_action(lidar_inverted_la)
    ld.add_action(lidar_angle_compensate_la)
    
    # start nodes
    ld.add_action(map_odom_static_tf_node)  
    ld.add_action(odom_base_link_static_tf_node)  
    ld.add_action(sllidar_node)
    
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_localization_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(ackermann_mux_node)    
    ld.add_action(twist_to_ackermann_node)   
     
    ld.add_action(slam_toolbox_start)  
    ld.add_action(nav2_start)
         
    ld.add_action(realsense_camera_launch_file) 
    ld.add_action(depthimage_to_laserscan_node)  
    
    return ld
                
