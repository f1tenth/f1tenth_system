# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    joy_teleop_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'joy_teleop.yaml'
    )
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'vesc.yaml'
    )
    sensors_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'sensors.yaml'
    )
    camera_config = os.path.join(
        get_package_share_directory('stack_master'),
        'config',
        'camera',
        'd435_config.yaml'
    )

    use_vio_arg = DeclareLaunchArgument(
        'use_vio',
        default_value=EnvironmentVariable('USE_VIO', default_value='false'),
        description='Launch the RealSense camera (set via USE_VIO in .env)'
    )
    joy_la = DeclareLaunchArgument(
        'joy_teleop_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs'
    )
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs'
    )
    sensors_la = DeclareLaunchArgument(
        'sensors_config',
        default_value=sensors_config,
        description='Descriptions for sensor configs'
    )
    camera_la = DeclareLaunchArgument(
        'camera_config',
        default_value=camera_config,
        description='Path to RealSense camera config file'
    )

    # Initialize LaunchDescription with arguments
    ld = LaunchDescription([
        use_vio_arg,
        joy_la,
        vesc_la,
        sensors_la,
        camera_la
    ])

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_teleop_config')]
    )
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_teleop_config')]
    )
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    def vesc_to_odom(context):
        vesc_config = LaunchConfiguration('vesc_config').perform(context)
        params = [vesc_config]
        # stack_master configs keep the wheelbase in car_model.yaml next to vesc.yaml
        car_model = os.path.join(os.path.dirname(vesc_config), 'car_model.yaml')
        if os.path.exists(car_model):
            with open(car_model) as f:
                car = yaml.safe_load(f)
            params.append({'wheelbase': car['lf'] + car['lr']})
        return [Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            parameters=params
        )]
    vesc_to_odom_node = OpaqueFunction(function=vesc_to_odom)
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    actuation_manager_node = Node(
        package='f1tenth_stack',
        executable='actuation_manager',
        name='actuation_manager',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[LaunchConfiguration('sensors_config')]
    )
    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[LaunchConfiguration('camera_config')],
        condition=IfCondition(LaunchConfiguration('use_vio'))
    )

    # Finalize LaunchDescription
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(actuation_manager_node)
    ld.add_action(urg_node)
    ld.add_action(camera_node)

    return ld
