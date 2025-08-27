#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import json
import threading
from typing import Tuple, List
import numpy as np

import rclpy
import rclpy.node
from std_msgs.msg import UInt8MultiArray
from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import JointState

from .interface_base import InterfaceBase

class DataInterface(InterfaceBase):

    def __init__(self, name: str = "unknown"):
        super(DataInterface, self).__init__(name = name)

        # ros node
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self.__logger = self.__node.get_logger()
        self.__node.declare_parameter('rate_ros', 300.0)
        rate_value = self.__node.get_parameter('rate_ros').value
        self._rate_param["ros"] = float(rate_value if rate_value is not None else 300.0)
        self.__rate = self.__node.create_rate(self._rate_param["ros"])

        # pamameter
        # declare parameters
        self.__node.declare_parameter('rate_state', 200.0)
        self._rate_param["state"] = self.__node.get_parameter('rate_state').value

        self.__node.declare_parameter('frame_id', "base_link")
        self.__frame_id = self.__node.get_parameter('frame_id').value

        self.__node.declare_parameter('simple_mode', True)
        self._simple_mode = self.__node.get_parameter('simple_mode').value

        # publisher
        self.__ws_down_pub = self.__node.create_publisher(
            UInt8MultiArray,
            'ws_down',
            10,
        )
        self.__motor_status_pub = self.__node.create_publisher(
            JointState,
            'motor_status',
            10,
        )
        self.__real_vel_pub = self.__node.create_publisher(
            TwistStamped,
            'real_vel',
            10,
        )

        # subscriber
        self.__ws_up_sub = self.__node.create_subscription(
            UInt8MultiArray,
            'ws_up',
            self.__ws_up_callback,
            10,
        )
        self.__joint_ctrl_sub = self.__node.create_subscription(
            JointState,
            'joint_ctrl',
            self.__joint_ctrl_callback,
            10,
        )
        self.__cmd_vel_sub = self.__node.create_subscription(
            Twist,
            'cmd_vel',
            self.__cmd_vel_callback,
            10,
        )

        self.__ws_up_sub
        self.__joint_ctrl_sub
        self.__cmd_vel_sub

        # spin thread
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

        # finish log
        print(f"#### DataInterface init: {self._name} ####")

    def __spin(self):
        rclpy.spin(self.__node)

    def ok(self):
        return rclpy.ok()

    def shutdown(self):
        self.__node.destroy_node()
        rclpy.shutdown()
        self.__spin_thread.join()

    def sleep(self):
        self.__rate.sleep()

    def logd(self, msg, *args, **kwargs):
        self.__logger.debug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        self.__logger.info(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        self.__logger.warning(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        self.__logger.error(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        self.__logger.fatal(msg, *args, **kwargs)

    # pub
    def pub_ws_down(self, data: List[int]):
        msg = UInt8MultiArray()
        msg.data = data
        self.__ws_down_pub.publish(msg)

    def pub_motor_status(self, pos: List[float], vel: List[float], eff: List[float]):
        length = len(pos)
        msg = JointState()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.name = [f"joint{i}" for i in range(length)]
        msg.position = pos
        msg.velocity = vel
        msg.effort = eff
        self.__motor_status_pub.publish(msg)

    def pub_real_vel(self, x: float, y: float, yaw: float):
        msg = TwistStamped()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.header.frame_id = self.__frame_id
        msg.twist.linear.x = x
        msg.twist.linear.y = y
        msg.twist.angular.z = yaw
        self.__real_vel_pub.publish(msg)

    # sub
    def __ws_up_callback(self, msg: UInt8MultiArray):
            self._bin_msg_queue.put(msg.data)

    def __joint_ctrl_callback(self, msg: JointState):
            self._joint_ctrl_queue.put(msg)

    def __cmd_vel_callback(self, msg: Twist):
            self._cmd_vel_queue.put(msg)