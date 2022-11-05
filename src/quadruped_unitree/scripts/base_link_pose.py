#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time

from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose

class GazeboLinkPose(Node):
  def __init__(self):
    self.declare_parameter('~link_name', 'go1_gazebo::base')
    self.link_name = GazeboLinkPose(self.get_parameter('~link_name').get_parameter_value().string_value)
    
    self.link_name_rectified = self.link_name.replace("::", "_")
    self.link_pose = Pose()

    if not self.link_name:
      raise ValueError("'link_name' is an empty string")

    self.states_sub = self.create_subscription(LinkStates, "/gazebo/link_states", self.callback)
    self.pose_pub = self.create_publisher(Pose, "/gazebo/" + self.link_name_rectified, 10)

    self.timer_ = self.create_timer(0.001, self.timer_callback)


  def timer_callback(self):
    self.pose_pub.publish(self.link_pose)

  def callback(self, data):
    try:
      ind = data.name.index(self.link_name)
      self.link_pose = data.pose
    except ValueError:
      pass

def main(args=None):       # ROS2节点主入口main函数
    rclpy.init(args=args)      # ROS2 Python接口初始化
    node = GazeboLinkPose("gazebo_link_pose")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)
    node.destroy_node()     # 销毁节点对象
    rclpy.shutdown()        # 关闭ROS2 Python接口
