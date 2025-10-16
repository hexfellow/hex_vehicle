#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import time

import rospy
import rospkg

from .interface_base import InterfaceBase

class DataInterface(InterfaceBase):
    def _init_node(self):
        # ros node
        rospy.init_node(self._node_name, anonymous=True)
        self.__rate = rospy.Rate(300.0)

    def create_publisher(self, msg_type, topic: str, queue_size: int = 10):
        pub = rospy.Publisher(topic, msg_type, queue_size=queue_size)
        return pub

    def create_subscriber(self, msg_type, topic: str, callback, queue_size: int = 10):
        rospy.Subscriber(topic, msg_type, callback, queue_size=queue_size)

    def create_timer(self, interval_sec: float, callback):
        # Wrap callback to ignore TimerEvent parameter from ROS1
        def timer_wrapper(event):
            callback()
        timer = rospy.Timer(rospy.Duration(interval_sec), timer_wrapper)
        return timer

    def cancel_timer(self, timer):
        if timer is not None:
            timer.shutdown()

    def set_parameter(self, name: str, value):
        rospy.set_param(name, value)

    def get_parameter(self, name: str):
        return rospy.get_param(name)
    
    def spin(self):
        rospy.spin()
    
    def ok(self):
        return not rospy.is_shutdown()

    def shutdown(self):
        try:
            rospy.signal_shutdown("Shutdown")
        except Exception:
            pass

    def sleep(self):
        self.__rate.sleep()

    def logd(self, msg, *args, **kwargs):
        try:
            rospy.logdebug(msg, *args, **kwargs)
        except Exception:
            pass

    def logi(self, msg, *args, **kwargs):
        try:
            rospy.loginfo(msg, *args, **kwargs)
        except Exception:
            pass

    def logw(self, msg, *args, **kwargs):
        try:
            rospy.logwarn(msg, *args, **kwargs)
        except Exception:
            pass

    def loge(self, msg, *args, **kwargs):
        try:
            rospy.logerr(msg, *args, **kwargs)
        except Exception:
            pass

    def logf(self, msg, *args, **kwargs):
        try:
            rospy.logfatal(msg, *args, **kwargs)
        except Exception:
            pass

    def get_pkg_share_path(self, package_name: str) -> str:
        try:
            rospack = rospkg.RosPack()
            return rospack.get_path(package_name)
        except rospkg.ResourceNotFound:
            self.loge(f"Package '{package_name}' not found.")
            return ""
        except Exception as e:
            self.loge(f"An error occurred while getting the path for package '{package_name}': {e}")
            return ""

    def get_timestamp(self):
        return rospy.Time.now()