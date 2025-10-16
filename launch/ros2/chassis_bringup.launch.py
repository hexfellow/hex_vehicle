#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the launch arguments
    url = DeclareLaunchArgument(
        'url',
        default_value='',
        description='The URL of the robot.'
    )

    read_only = DeclareLaunchArgument(
        'read_only',
        default_value='false',
        description='Whether to read only the chassis state.'
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
    xpkg_bridge_node = Node(
        package='xpkg_bridge',
        executable='xnode_bridge',
        name='xnode_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'url': LaunchConfiguration('url'),
            'read_only': LaunchConfiguration('read_only'),
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
            ('/real_vel', '/real_vel'),
            ('/odom', '/odom'),
            # publish
            ('/joint_ctrl', '/joint_ctrl'),
            ('/cmd_vel', '/cmd_vel'),
            ('/clear_err', '/clear_err')
        ]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        url,
        read_only,
        frame_id,
        simple_mode,
        report_freq,
        xpkg_bridge_node,
        hex_vehicle_node
    ])