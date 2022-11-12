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
import launch

package_name = 'go1_gazebo'


def generate_launch_description():
    ld = LaunchDescription()

    # robot description
    pkg_share = get_package_share_directory("go1_description")

    robot_description_path =  os.path.join(pkg_share, "urdf", "go1.urdf",)
    with open(robot_description_path, 'r') as infp:
        robot_desc = infp.read()
    use_sim_time = True
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=["--ros-args --remap", "/joint_states:=/go1_gazebo/joint_states"],
        output="both",
        parameters=[{"robot_description": robot_desc, 'use_sim_time': use_sim_time, "publish_frequency": 40.}],
    )
    ld.add_action(robot_state_publisher_node)

    xacro_file = os.path.join(get_package_share_directory('go1_description'), 'urdf/', 'go1.urdf')    
    assert os.path.exists(xacro_file), "The go1.urdf doesn't exist in "+str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=['-topic', 'robot_description',
                    "-entity", robot_desc, 
                    "-x", "0.0",
                    "-y", "0.0",
                    "-z", "0.4"],
        output="screen",
    )

    ld.add_action(spawn_entity)
    return ld
