#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare the launch arguments
    enable_bridge = DeclareLaunchArgument(
        'enable_bridge',
        default_value='false',
        description='Whether to enable the hex_bridge node, you can set it to false if you want to launch the node separately.'
    )

    url = DeclareLaunchArgument(
        'url',
        default_value='0.0.0.0:8439',
        description='The URL of the robot.'
    )

    read_only = DeclareLaunchArgument(
        'read_only',
        default_value='false',
        description='Whether to read only the chassis state.'
    )

    is_kcp = DeclareLaunchArgument(
        'is_kcp',
        default_value='true',
        description='Whether to use KCP protocol.'
    )

    frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='base_link',
        description='Frame ID of the chassis.'
    )

    simple_mode = DeclareLaunchArgument(
        'simple_mode',
        default_value='true',
        description='Simple mode of the chassis.'
    )

    report_freq = DeclareLaunchArgument(
        'report_freq',
        default_value='100',
        description='Report frequency of the chassis.'
    )

    # Define the node
    hex_bridge_node = Node(
        package='hex_bridge',
        executable='hex_bridge',
        name='hex_bridge',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_bridge')),
        parameters=[{
            'url': LaunchConfiguration('url'),
            'read_only': LaunchConfiguration('read_only'),
            'is_kcp': LaunchConfiguration('is_kcp'),
        }],
        remappings=[
            # subscribe
            ('/ws_down', '/ws_down'),
            # publish
            ('/ws_up', '/ws_up')
        ]
    )

    hex_vehicle_node = Node(
        package='hex_vehicle',
        executable='chassis_trans',
        name='hex_chassis',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'frame_id': LaunchConfiguration('frame_id'),
            'simple_mode': LaunchConfiguration('simple_mode'),
            'report_freq': LaunchConfiguration('report_freq'),
        }],
        remappings=[
            # subscribe
            ('/motor_states', '/motor_states'),
            ('/odom', '/odom'),
            # publish
            ('/joint_ctrl', '/joint_ctrl'),
            ('/cmd_vel', '/cmd_vel'),
            ('/clear_err', '/clear_err')
        ]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        enable_bridge,
        url,
        read_only,
        is_kcp,
        frame_id,
        simple_mode,
        report_freq,
        hex_bridge_node,
        hex_vehicle_node
    ])