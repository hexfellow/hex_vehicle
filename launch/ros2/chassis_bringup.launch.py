#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the launch arguments
    rate_ros = DeclareLaunchArgument(
        'rate_ros',
        default_value='300.0',
        description='Rate of the ROS node.'
    )
    rate_state = DeclareLaunchArgument(
        'rate_state',
        default_value='200.0',
        description='Rate of the state node.'
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

    # Define the node
    piper_node = Node(
        package='robot_interface',
        executable='chassis_trans',
        name='hex_chassis',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'rate_ros': LaunchConfiguration('rate_ros'),
            'rate_state': LaunchConfiguration('rate_state'),
            'frame_id': LaunchConfiguration('frame_id'),
            'simple_mode': LaunchConfiguration('simple_mode'),
        }],
        remappings=[
            # subscribe
            ('/motor_status', '/motor_status'),
            ('/real_vel', '/real_vel'),
            # publish
            ('/joint_ctrl', '/joint_ctrl'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        rate_ros,
        rate_state,
        frame_id,
        simple_mode,
        piper_node
    ])