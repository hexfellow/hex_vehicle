#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import json
from typing import Tuple, List
import numpy as np

import rospy
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, Twist

from .interface_base import InterfaceBase


class DataInterface(InterfaceBase):

    def __init__(self, name: str = "unknown"):
        super(DataInterface, self).__init__(name=name)

        # ros node
        rospy.init_node(self._name, anonymous=True)
        self._rate_param["ros"] = rospy.get_param('~rate_ros', 300.0)
        self.__rate = rospy.Rate(self._rate_param["ros"])

        # pamameter
        # declare parameters
        self._rate_param["state"] = rospy.get_param('~rate_state', 200.0)
        self.__frame_id = rospy.get_param('~frame_id', "base_link")
        self._simple_mode = rospy.get_param('~simple_mode', True)

        # publisher
        self.__motor_status_pub = rospy.Publisher(
            'motor_status',
            JointState,
            queue_size=10,
        )
        self.__real_vel_pub = rospy.Publisher(
            'real_vel',
            TwistStamped,
            queue_size=10,
        )
        self.__ws_down_pub = rospy.Publisher(
            'ws_down',
            UInt8MultiArray,
            queue_size=10,
        )

        # subscriber
        self.__ws_up_sub = rospy.Subscriber(
            'ws_up',
            UInt8MultiArray,
            self.__ws_up_callback,
        )
        self.__ws_up_sub

        self.__cmd_vel_sub = rospy.Subscriber(
            'cmd_vel',
            Twist,
            self.__cmd_vel_callback,
        )
        self.__cmd_vel_sub

        self.__joint_ctrl_sub = rospy.Subscriber(
            'joint_ctrl',
            JointState,
            self.__joint_ctrl_callback,
        )
        self.__joint_ctrl_sub

        # finish log
        print(f"#### DataInterface init: {self._name} ####")

    def ok(self):
        return not rospy.is_shutdown()

    def shutdown(self):
        pass

    def sleep(self):
        self.__rate.sleep()

    def logd(self, msg, *args, **kwargs):
        rospy.logdebug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        rospy.loginfo(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        rospy.logwarn(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        rospy.logerr(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        rospy.logfatal(msg, *args, **kwargs)

    def pub_ws_down(self, data: List[int]):
        out = UInt8MultiArray()
        out.data = data
        self.__ws_down_pub.publish(out)    

    def pub_motor_status(self, pos: List[float], vel: List[float], eff: List[float]):
        out = JointState()
        out.header.stamp = rospy.Time.now()
        out.name = [f"joint{i}" for i in range(len(pos))]
        out.position = pos
        out.velocity = vel
        out.effort = eff
        self.__motor_status_pub.publish(out)
    
    def pub_real_vel(self, x: float, y: float, yaw: float):
        out = TwistStamped()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.__frame_id
        out.twist.linear.x = x
        out.twist.linear.y = y
        out.twist.angular.z = yaw
        self.__real_vel_pub.publish(out)

    def __ws_up_callback(self, msg: UInt8MultiArray):
        self._bin_msg_queue.put(msg.data)
        
    def __joint_ctrl_callback(self, msg: JointState):
        self._joint_ctrl_queue.put(msg)

    def __cmd_vel_callback(self, msg: Twist):
        self._cmd_vel_queue.put(msg)