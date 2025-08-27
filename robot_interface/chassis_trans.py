#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np
from math import pi as PI
from typing import Tuple, List
import threading

import os
import sys

script_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(script_path)
from utility import DataInterface

from .proto import public_api_down_pb2
from .proto import public_api_types_pb2
from .proto import public_api_up_pb2

class ChassisTrans:
    def __init__(self):
        self.__data_interface = DataInterface(name = "chassis_trans")
        self.__api_initialized = False

    def work(self):
        try:
            while self.__data_interface.ok():
                self._handle_ws_up()
                if self.__data_interface._simple_mode:
                    self._handle_cmd_vel()
                else:
                    self._handle_joint_ctrl()
                self.__data_interface.sleep()
        except KeyboardInterrupt:
            self.__data_interface.logi("Received Ctrl-C.")
        finally:
            self.__data_interface.shutdown()
        exit(0)

    def _handle_ws_up(self):
        if not self.__data_interface._bin_msg_queue.empty():
            bin_msg = self.__data_interface._bin_msg_queue.get()
            api_up = public_api_up_pb2.APIUp()
            api_up.ParseFromString(bytes(bin_msg))
            self.__api_initialized = api_up.base_status.api_control_initialized
            # parse data
            pp, vv, tt = self._prase_wheel_data(api_up)
            (spd_x, spd_y, spd_z), (pos_x, pos_y, pos_z) = self._parse_vehicle_data(api_up)
            acc, angular_velocity, quaternion = self._parse_imu_data(api_up)
            # publish data
            self.__data_interface.pub_motor_status(pp, vv, tt)
            self.__data_interface.pub_real_vel(spd_x, spd_y, angular_velocity[2])

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
            spd_r = api_up.base_status.estimated_odometry.speed_z
            pos_x = api_up.base_status.estimated_odometry.pos_x
            pos_y = api_up.base_status.estimated_odometry.pos_y
            pos_r = api_up.base_status.estimated_odometry.pos_z
        return (spd_x, spd_y, spd_z), (pos_x, pos_y, pos_z)
    
    def _parse_imu_data(self, api_up: public_api_up_pb2.APIUp) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
        acc = (0.0, 0.0, 0.0)
        angular_velocity = (0.0, 0.0, 0.0)
        quaternion = (0.0, 0.0, 0.0, 0.0)
        if api_up.imu_data.IsInitialized():
            acc = (api_up.imu_data.acceleration.ax, api_up.imu_data.acceleration.ay, api_up.imu_data.acceleration.az)
            angular_velocity = (api_up.imu_data.angular_velocity.wx, api_up.imu_data.angular_velocity.wy, api_up.imu_data.angular_velocity.wz)
            quaternion = (api_up.imu_data.quaternion.qx, api_up.imu_data.quaternion.qy, api_up.imu_data.quaternion.qz, api_up.imu_data.quaternion.qw)
        return acc, angular_velocity, quaternion
    
    def _handle_cmd_vel(self):
        if not self.__data_interface._cmd_vel_queue.empty():
            if not self.__api_initialized:
                api_init = public_api_down_pb2.APIDown(
                    base_command = public_api_types_pb2.BaseCommand(api_control_initialize = True)
                )
                bin = api_init.SerializeToString()
                self.__data_interface.pub_ws_down(bin)
            msg = self.__data_interface._cmd_vel_queue.get()
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
            self.__data_interface.pub_ws_down(bin)

    def _handle_joint_ctrl(self):
        if not self.__data_interface._joint_ctrl_queue.empty():
            if not self.__api_initialized:
                api_init = public_api_down_pb2.APIDown()
                api_init.base_command.api_control_initialize = True
                bin = api_init.SerializeToString()
                self.__data_interface.pub_ws_down(bin)
            msg = self.__data_interface._joint_ctrl_queue.get()
            length = len(msg.name)
            if length != len(msg.position) or length != len(msg.velocity) or length != len(msg.effort):
                self.__data_interface.logw("JointState message format error.")
                return
            api_down = public_api_down_pb2.APIDown()
            for i in range(length):
                joint_cmd = api_down.base_command.motor_targets.targets.add()
                # joint_cmd.position = msg.position[i]
                joint_cmd.speed = msg.velocity[i]
                # joint_cmd.torque = msg.effort[i]
            bin = api_down.SerializeToString()
            self.__data_interface.pub_ws_down(list(bin))
    
def main():
    chassis_trans = ChassisTrans()
    chassis_trans.work()

if __name__ == '__main__':
    main()