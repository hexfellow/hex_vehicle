#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import json
from abc import ABC, abstractmethod

class InterfaceBase(ABC):
    def __init__(self, node_name: str):
        self._node_name = node_name
        self._init_node()

    @abstractmethod
    def _init_node(self):
        raise NotImplementedError("InterfaceBase._init_node")

    @abstractmethod
    def create_publisher(self, msg_type, topic: str, queue_size: int = 10):
        raise NotImplementedError("InterfaceBase.create_publisher")
    
    @abstractmethod
    def create_subscriber(self, msg_type, topic: str, callback, queue_size: int = 10):
        raise NotImplementedError("InterfaceBase.create_subscriber")
    
    @abstractmethod
    def create_timer(self, interval_sec: float, callback):
        """
        Create a timer that calls the callback at the specified interval.
        Returns a timer object that can be used to cancel the timer.
        """
        raise NotImplementedError("InterfaceBase.create_timer")

    @abstractmethod
    def cancel_timer(self, timer):
        """
        Cancel the specified timer.
        Args:
            timer: The timer object returned by create_timer()
        """
        raise NotImplementedError("InterfaceBase.cancel_timer")
    
    @abstractmethod
    def set_parameter(self, name: str, value):
        raise NotImplementedError("InterfaceBase.set_parameter")
    
    @abstractmethod
    def get_parameter(self, name: str):
        raise NotImplementedError("InterfaceBase.get_parameter")
    
    @abstractmethod
    def spin(self):
        raise NotImplementedError("InterfaceBase.spin")
    
    @abstractmethod
    def ok(self) -> bool:
        raise NotImplementedError("InterfaceBase.ok")

    @abstractmethod
    def shutdown(self):
        raise NotImplementedError("InterfaceBase.shutdown")

    @abstractmethod
    def sleep(self):
        raise NotImplementedError("InterfaceBase.sleep")

    # logging
    @abstractmethod
    def logd(self, msg, *args, **kwargs):
        raise NotImplementedError("logd")

    @abstractmethod
    def logi(self, msg, *args, **kwargs):
        raise NotImplementedError("logi")

    @abstractmethod
    def logw(self, msg, *args, **kwargs):
        raise NotImplementedError("logw")

    @abstractmethod
    def loge(self, msg, *args, **kwargs):
        raise NotImplementedError("loge")

    @abstractmethod
    def logf(self, msg, *args, **kwargs):
        raise NotImplementedError("logf")

    @abstractmethod
    def get_timestamp(self):
        raise NotImplementedError("get_timestamp")
    