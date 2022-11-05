#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Author: lnotspotl

import rclpy
from rclpy.node import Node                      # ROS2 节点类
from rclpy.time import Time

from sensor_msgs.msg import Joy, Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

from go1_controller.RobotController import RobotController
from go1_controller.InverseKinematics import robot_IK



class Controller(Node):
    def __init__(self, name):
        super().__init__(name)
        USE_IMU = True
        body = [0.3762, 0.0935]
        legs = [0.,0.08, 0.213, 0.213] 
        self.a1_robot = RobotController.Robot(body, legs, USE_IMU)
        self.inverseKinematics = robot_IK.InverseKinematics(body, legs)

        command_topics = ["/go1_gazebo/FR_hip_joint/command",
                        "/go1_gazebo/FR_thigh_joint/command",
                        "/go1_gazebo/FR_calf_joint/command",
                        "/go1_gazebo/FL_hip_joint/command",
                        "/go1_gazebo/FL_thigh_joint/command",
                        "/go1_gazebo/FL_calf_joint/command",
                        "/go1_gazebo/RR_hip_joint/command",
                        "/go1_gazebo/RR_thigh_joint/command",
                        "/go1_gazebo/RR_calf_joint/command",
                        "/go1_gazebo/RL_hip_joint/command",
                        "/go1_gazebo/RL_thigh_joint/command",
                        "/go1_gazebo/RL_calf_joint/command"]
        self.mypublishers = []
        for i in range(len(command_topics)):
            self.mypublishers.append(self.create_publisher(Float64, command_topics[i], 10))
        
        if USE_IMU:
            self.create_subscription(Imu, "go1_imu/base_link_orientation", self.a1_robot.imu_orientation, 10)
        # self.create_subscription(Joy, "ga1_joy/joy_ramped", a1_robot.joystick_command, 10)
        self.create_subscription(Twist, "/twist_mux/cmd_vel", self.a1_robot.cmd_vel_command, 10)

        RATE = 60
        self.timer_ = self.create_timer(1/RATE, self.timer_callback)
        self.get_logger().info("控制器初始化完成...")
    
    def timer_callback(self):
        leg_positions = self.a1_robot.run()
        self.a1_robot.change_controller()

        dx, dy, dz = self.a1_robot.state.body_local_position[0:3]
        
        roll, pitch, yaw = self.a1_robot.state.body_local_orientation[0:3]

        try:
            joint_angles = self.inverseKinematics.inverse_kinematics(leg_positions,
                                dx, dy, dz, roll, pitch, yaw)
            
            msg_ = Float64()
            for i in range(len(joint_angles)):
                msg_.data = joint_angles[i]
                self.mypublishers[i].publish(msg_)
        except:
            self.get_logger().info("发送控制信号出现错误...")
        

def main(args=None):       # ROS2节点主入口main函数
    rclpy.init(args=args)      # ROS2 Python接口初始化
    node = Controller("Robot_Controller")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)
    node.destroy_node()     # 销毁节点对象
    rclpy.shutdown()        # 关闭ROS2 Python接口
