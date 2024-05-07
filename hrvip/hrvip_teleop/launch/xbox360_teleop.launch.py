from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
    )
    xbox360_teleop_node = Node(
        package='hrvip_teleop', executable='xbox360_teleop',
        parameters=[
            {
                'max_gripper_size': 0.93,
                'joy_topic': 'joy',
                'twist_topic': 'cmd_vel',
                'gripper_command_topic': 'gripper_forward_position_controller/commands',
                'linear_scale': 0.1,
                'angular_scale': 0.2,
            }
        ]
    )

    nodes = [
        joy_node,
        xbox360_teleop_node,
    ] 

    return LaunchDescription(declared_arguments + nodes)