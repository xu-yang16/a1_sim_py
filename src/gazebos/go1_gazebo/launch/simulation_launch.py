#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from time import sleep
import filecmp
import xacro

package_name = 'go1_gazebo'


def generate_launch_description():
    # robot description
    pkg_share = get_package_share_directory(package_name)

    pkg_urdf = get_package_share_directory("go1_description")

    robot_description_path =  os.path.join(pkg_urdf, "urdf", "go1.urdf",)
    with open(robot_description_path, 'r') as infp:
        robot_desc = infp.read()
    robot_description = {"robot_description": robot_desc}
    # use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # paused
    paused = LaunchConfiguration('paused', default='true')

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="both",
        parameters=[robot_description],
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    world_file_name = LaunchConfiguration('world_name', default=os.path.join(pkg_share, "world", "normal.world"))
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file_name, '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )
    gazebo_use_sim_time = ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen',
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "go1_description"],
        output="screen",
    )

    # control
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["FR_hip_joint FR_thigh_joint FR_calf_joint FL_hip_joint FL_thigh_joint FL_calf_joint RR_hip_joint RR_thigh_joint RR_calf_joint RL_hip_joint RL_thigh_joint RL_calf_joint"],
        output="screen",
    )

    rviz2_arg = DeclareLaunchArgument('rviz2', default_value='false', description='Open RViz.')
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_share, "rviz", "go1_gazebo.rviz")],
        condition = IfCondition(LaunchConfiguration('rviz2')),
        output="screen",
    )
    ld = LaunchDescription()
    ld.add_action(gazebo)
    ld.add_action(gazebo_use_sim_time)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(spawn_entity)

    ld.add_action(controller_spawner)

    ld.add_action(rviz2_arg)
    ld.add_action(rviz2)

    return ld