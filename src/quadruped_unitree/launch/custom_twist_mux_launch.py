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


def generate_launch_description():
    twist_marker = Node(
        package="twist_mux",
        executable="twist_marker",
    )
    joystick_relay = Node(
        package="quadruped_unitree",
        executable="joystick_relay.py",
        output="screen",
    )


    ld = LaunchDescription()
    ld.add_action(twist_marker)

    return ld