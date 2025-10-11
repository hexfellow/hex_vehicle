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
        self.__subscribers = []  # Keep track of all subscribers
        self.__publishers = []   # Keep track of all publishers

    def create_publisher(self, msg_type, topic: str, queue_size: int = 10):
        pub = rospy.Publisher(topic, msg_type, queue_size=queue_size)
        self.__publishers.append(pub)
        return pub

    def create_subscriber(self, msg_type, topic: str, callback, queue_size: int = 10):
        sub = rospy.Subscriber(topic, msg_type, callback, queue_size=queue_size)
        self.__subscribers.append(sub)

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
        # Step 1: Unregister all subscribers to stop receiving messages
        for sub in self.__subscribers:
            sub.unregister()
        self.__subscribers.clear()

        # Step 2: Give a short grace period for ongoing callbacks to finish
        time.sleep(0.05)  # 50ms should be enough

        # Step 3: Unregister all publishers
        for pub in self.__publishers:
            pub.unregister()
        self.__publishers.clear()

        # Step 4: Finally signal shutdown
        rospy.signal_shutdown("Shutdown")

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