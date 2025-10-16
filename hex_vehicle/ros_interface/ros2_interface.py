#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import threading
import rclpy
import rclpy.node
from ament_index_python.packages import get_package_share_directory

from .interface_base import InterfaceBase

class DataInterface(InterfaceBase):
    def _init_node(self):
        # ros node
        rclpy.init()
        self.__node = rclpy.node.Node(self._node_name)
        self.__logger = self.__node.get_logger()
        self.__rate = self.__node.create_rate(300.0)

        self.__spin_thread = threading.Thread(target=self.spin)
        self.__spin_thread.start()

    def create_publisher(self, msg_type, topic: str, queue_size: int = 10):
        return self.__node.create_publisher(msg_type, topic, queue_size)

    def create_subscriber(self, msg_type, topic: str, callback, queue_size: int = 10):
        self.__node.create_subscription(msg_type, topic, callback, queue_size)

    def create_timer(self, interval_sec: float, callback):
        timer = self.__node.create_timer(interval_sec, callback)
        return timer

    def cancel_timer(self, timer):
        if timer is not None:
            timer.cancel()
            
    def set_parameter(self, name: str, value):
        self.__node.declare_parameter(name, value)

    def get_parameter(self, name: str):
        return self.__node.get_parameter(name).value
    
    def spin(self):
        try:
            rclpy.spin(self.__node)
        except Exception:
            pass

    def ok(self):
        return rclpy.ok()

    def shutdown(self):
        try:
            # First shutdown rclpy to stop spin thread
            if rclpy.ok():
                rclpy.shutdown()
            # Wait for spin thread to finish
            if self.__spin_thread and self.__spin_thread.is_alive():
                self.__spin_thread.join(timeout=1.0)
            # Then destroy node
            try:
                self.__node.destroy_node()
            except Exception:
                pass
        except Exception:
            # Ignore any shutdown errors
            pass

    def sleep(self):
        self.__rate.sleep()

    def logd(self, msg, *args, **kwargs):
        try:
            self.__logger.debug(msg, *args, **kwargs)
        except Exception:
            pass

    def logi(self, msg, *args, **kwargs):
        try:
            self.__logger.info(msg, *args, **kwargs)
        except Exception:
            pass

    def logw(self, msg, *args, **kwargs):
        try:
            self.__logger.warning(msg, *args, **kwargs)
        except Exception:
            pass

    def loge(self, msg, *args, **kwargs):
        try:
            self.__logger.error(msg, *args, **kwargs)
        except Exception:
            pass

    def logf(self, msg, *args, **kwargs):
        try:
            self.__logger.fatal(msg, *args, **kwargs)  
        except Exception:
            pass

    def get_pkg_share_path(self, package_name: str) -> str:
        try:
            return get_package_share_directory(package_name)
        except Exception as e:
            self.loge(f"An error occurred while getting the path for package '{package_name}': {e}")
            return ""

    def get_timestamp(self):
        return self.__node.get_clock().now().to_msg()
