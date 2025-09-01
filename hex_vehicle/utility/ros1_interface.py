#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import json
from typing import Tuple, List
import numpy as np

import rospy
from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8MultiArray

import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import public_api_down_pb2
import public_api_types_pb2
import public_api_up_pb2

from .interface_base import InterfaceBase


class DataInterface(InterfaceBase):

    def __init__(self, name: str = "unknown"):
        super(DataInterface, self).__init__(name=name)

        # ros node
        rospy.init_node(self._name, anonymous=True)
        self.__rate = rospy.Rate(300.0)

        # pamameter
        # declare parameters
        self.__frame_id = rospy.get_param('~frame_id', "base_link")
        self._simple_mode = rospy.get_param('~simple_mode', True)

        self.__api_initialized = False

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
        self.__odom_pub = rospy.Publisher(
            'odom',
            Odometry,
            queue_size=10,
        )

        # subscriber
        self.__ws_up_sub = rospy.Subscriber(
            'ws_up',
            UInt8MultiArray,
            self.__ws_up_callback,
        )
        self.__ws_up_sub

        if self._simple_mode:
            self.__cmd_vel_sub = rospy.Subscriber(
                'cmd_vel',
                Twist,
                self.__cmd_vel_callback,
            )
            self.__cmd_vel_sub
        else:
            self.__joint_ctrl_sub = rospy.Subscriber(
                'joint_ctrl',
                JointState,
                self.__joint_ctrl_callback,
            )
            self.__joint_ctrl_sub

        # finish log
        print(f"#### DataInterface init: {self._name} ####")

    def __spin(self):
        rospy.spin()

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

    def pub_odom(self, pos_x: float, pos_y: float, pos_yaw: float, linear_x: float, linear_y: float, angular_z: float):
        out = Odometry()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = "Odom"
        out.child_frame_id = self.__frame_id
        out.pose.pose.position.x = pos_x
        out.pose.pose.position.y = pos_y
        out.pose.pose.position.z = 0.0
        qz = np.sin(pos_yaw / 2.0)
        qw = np.cos(pos_yaw / 2.0)
        out.pose.pose.orientation.x = 0.0
        out.pose.pose.orientation.y = 0.0
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.twist.twist.linear.x = linear_x
        out.twist.twist.linear.y = linear_y
        out.twist.twist.angular.z = angular_z
        self.__odom_pub.publish(out)

    def __ws_up_callback(self, msg: UInt8MultiArray):
        api_up = public_api_up_pb2.APIUp()
        api_up.ParseFromString(bytes(msg.data))
        self.__api_initialized = api_up.base_status.api_control_initialized
        # parse data
        pp, vv, tt = self._prase_wheel_data(api_up)
        (spd_x, spd_y, spd_z), (pos_x, pos_y, pos_z) = self._parse_vehicle_data(api_up)
        acc, angular_velocity, quaternion = self._parse_imu_data(api_up)
        # publish data
        self.pub_motor_status(pp, vv, tt)
        self.pub_real_vel(spd_x, spd_y, spd_z)
        self.pub_odom(pos_x, pos_y, pos_z, spd_x, spd_y, spd_z)
        
    def __joint_ctrl_callback(self, msg: JointState):
        if not self.__api_initialized:
            api_init = public_api_down_pb2.APIDown()
            api_init.base_command.api_control_initialize = True
            bin = api_init.SerializeToString()
            self.pub_ws_down(bin)
        length = len(msg.name)
        if length != len(msg.position) or length != len(msg.velocity) or length != len(msg.effort):
            self.logw("JointState message format error.")
            return
        api_down = public_api_down_pb2.APIDown()
        for i in range(length):
            joint_cmd = api_down.base_command.motor_targets.targets.add()
            # joint_cmd.position = msg.position[i]
            joint_cmd.speed = msg.velocity[i]
            # joint_cmd.torque = msg.effort[i]
        bin = api_down.SerializeToString()
        self.pub_ws_down(list(bin))

    def __cmd_vel_callback(self, msg: Twist):
        if not self.__api_initialized:
            api_init = public_api_down_pb2.APIDown(
                base_command = public_api_types_pb2.BaseCommand(api_control_initialize = True)
            )
            bin = api_init.SerializeToString()
            self.pub_ws_down(bin)
        api_down = public_api_down_pb2.APIDown(
            base_command = public_api_types_pb2.BaseCommand(
                simple_move_command = public_api_types_pb2.SimpleBaseMoveCommand(
                    xyz_speed = public_api_types_pb2.XyzSpeed(
                        speed_x = msg.linear.x,
                        speed_y = msg.linear.y,
                        speed_z = msg.angular.z
                    )
                )
            )
        )
        bin = api_down.SerializeToString()
        self.pub_ws_down(bin)