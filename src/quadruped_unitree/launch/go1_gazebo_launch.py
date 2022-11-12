#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
import launch.actions
import launch


def generate_launch_description():
    print("beginning...")
    ld = LaunchDescription()
    # 输出当前launch文件位置
    current_launch_info = launch.actions.LogInfo(msg=['启动launch文件: ', ThisLaunchFileDir(), '/', os.path.basename(__file__)])
    ld.add_action(current_launch_info)
    print("finished...")

    # 启动其他launch文件
    included_launch = []
    # Gazebo Simulation
    included_launch.append(('go1_gazebo', 'launch/simulation_ROS2.launch'))
    # spawn urdf
    included_launch.append(('go1_gazebo', 'launch/spawn_launch.py'))
    # twist mux
    included_launch.append(('quadruped_unitree', 'launch/custom_twist_mux_ROS2.launch'))
    # Robot Controller
    included_launch.append(('go1_controller', 'launch/robot_controller_ROS2.launch'))


    for pkg_name, launch_file in included_launch:
        new_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.AnyLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg_name), launch_file)))

        new_launch_info = launch.actions.LogInfo(msg=['启动子launch文件 ', os.path.join(pkg_name, launch_file)])

        ld.add_action(new_launch)
        ld.add_action(new_launch_info)


    return ld