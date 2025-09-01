#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import json
from math import pi as PI
from typing import Tuple, List
from abc import ABC, abstractmethod

import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import public_api_down_pb2
import public_api_types_pb2
import public_api_up_pb2

class InterfaceBase(ABC):

    def __init__(self, name: str = "unknown"):
        # ros parameters
        self._rate_param = {}

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
    
    def _prase_wheel_data(self, api_up: public_api_up_pb2.APIUp) -> Tuple[list, list, list]:
        vv = []
        tt = []
        pp = []
        if api_up.base_status.IsInitialized():
            for motor_status in api_up.base_status.motor_status:
                torque = motor_status.torque
                speed = motor_status.speed
                position = (motor_status.position % motor_status.pulse_per_rotation) / motor_status.pulse_per_rotation * (2.0 * PI) - PI # radian
                tt.append(torque)
                vv.append(speed)
                pp.append(position)
        return pp, vv, tt
    
    def _parse_vehicle_data(self, api_up: public_api_up_pb2.APIUp) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        (spd_x, spd_y, spd_z) = (0.0, 0.0, 0.0)
        (pos_x, pos_y, pos_z) = (0.0, 0.0, 0.0)
        if api_up.base_status.IsInitialized():
            spd_x = api_up.base_status.estimated_odometry.speed_x
            spd_y = api_up.base_status.estimated_odometry.speed_y
            spd_z = api_up.base_status.estimated_odometry.speed_z
            pos_x = api_up.base_status.estimated_odometry.pos_x
            pos_y = api_up.base_status.estimated_odometry.pos_y
            pos_z = api_up.base_status.estimated_odometry.pos_z
        return (spd_x, spd_y, spd_z), (pos_x, pos_y, pos_z)
    
    def _parse_imu_data(self, api_up: public_api_up_pb2.APIUp) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float, float]]:
        acc = (0.0, 0.0, 0.0)
        angular_velocity = (0.0, 0.0, 0.0)
        quaternion = (0.0, 0.0, 0.0, 0.0)
        if api_up.imu_data.IsInitialized():
            acc = (api_up.imu_data.acceleration.ax, api_up.imu_data.acceleration.ay, api_up.imu_data.acceleration.az)
            angular_velocity = (api_up.imu_data.angular_velocity.wx, api_up.imu_data.angular_velocity.wy, api_up.imu_data.angular_velocity.wz)
            quaternion = (api_up.imu_data.quaternion.qx, api_up.imu_data.quaternion.qy, api_up.imu_data.quaternion.qz, api_up.imu_data.quaternion.qw)
        return acc, angular_velocity, quaternion
