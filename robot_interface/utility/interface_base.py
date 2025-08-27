#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import json
import queue
import threading
import typing
from abc import ABC, abstractmethod


class InterfaceBase(ABC):

    def __init__(self, name: str = "unknown"):
        # ros parameters
        self._rate_param = {}

        # rx msg queues
        self._bin_msg_queue = queue.Queue()
        self._joint_ctrl_queue = queue.Queue()
        self._cmd_vel_queue = queue.Queue()

        # name
        self._name = name
        print(f"#### InterfaceBase init: {self._name} ####")

    def __del__(self):
        self.shutdown()

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

    # parameters
    def _str_to_list(self, list_str: str) -> list:
        result = []
        for s in list_str:
            l = json.loads(s)
            result.append(l)
        return result

    def get_rate_param(self) -> dict:
        return self._rate_param

    # publishers
    @abstractmethod
    def pub_ws_down(self, out: typing.Any):
        raise NotImplementedError("InterfaceBase.pub_ws_down")
    
    @abstractmethod
    def pub_real_vel(self, out: typing.Any):
        raise NotImplementedError("InterfaceBase.pub_real_vel")
    
    @abstractmethod
    def pub_motor_status(self, out: typing.Any):
        raise NotImplementedError("InterfaceBase.pub_motor_status")
