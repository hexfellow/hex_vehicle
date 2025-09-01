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
    
def main():
    data_interface = DataInterface(name = "chassis_trans")
    try:
        while data_interface.ok():
            data_interface.sleep()
    except KeyboardInterrupt:
        data_interface.logi("Received Ctrl-C.")
    finally:
        data_interface.shutdown()
    exit(0)

if __name__ == '__main__':
    main()