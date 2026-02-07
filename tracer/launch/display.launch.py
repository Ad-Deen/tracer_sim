import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 1. Define Paths (Using absolute paths as requested)
    urdf_path = '/home/deen/ros2_ws/src/tracer/urdf/tracer.urdf'
    rviz_config_path = '/home/deen/ros2_ws/src/tracer/rviz/drone.rviz'

    # 2. Read the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Joint State Publisher GUI (to move propellers/camera joints)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # RViz2 with the specific config file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])