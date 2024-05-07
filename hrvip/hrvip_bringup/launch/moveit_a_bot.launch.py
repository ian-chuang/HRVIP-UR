from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
import os
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", 
            default_value="192.168.1.102",
            description="IP address by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_ip",
            default_value="192.168.1.2",
            description="IP that will be used for the robot controller to communicate back to the driver.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. "
            "Used only if 'use_fake_hardware' parameter is true.",
        )
    )


    # create move_it config
    moveit_config = (
        MoveItConfigsBuilder(
            "a_bot", package_name="hrvip_bringup"
        )
        .robot_description(
            file_path="urdf/a_bot.urdf.xacro",
        )
        .robot_description_semantic(file_path="config/moveit/a_bot.srdf")
        .robot_description_kinematics(file_path="config/moveit/kinematics.yaml")
        .joint_limits(file_path="config/moveit/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )


    robot_ip = LaunchConfiguration("robot_ip")
    reverse_ip = LaunchConfiguration("reverse_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")

    bringup_a_bot = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource(
    	    os.path.join(
    	    	get_package_share_directory('hrvip_bringup'),
                'launch',
                'bringup_a_bot.launch.py'
            )
    	),
    	launch_arguments={
    	    "robot_ip": robot_ip,
            "reverse_ip": reverse_ip,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            "activate_joint_controller": "true",
            "launch_rviz": "false",
    	}.items()
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("hrvip_bringup"), "rviz", "moveit_a_bot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # move group node enables moveit to plan and execute motion
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    def controller_spawner(name, *args):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[name] + [a for a in args],
        )

    # Active controllers
    active_list = [
        "gripper_action_controller",
    ]
    active_spawners = [controller_spawner(controller) for controller in active_list]

    nodes_to_start = [
        bringup_a_bot,
        rviz_node,
        move_group_node,
    ] + active_spawners

    return LaunchDescription(declared_arguments + nodes_to_start)