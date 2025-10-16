# hex_vehicle
## Overview
This is a ROS packdge that provides ROS interface for hex vehicle.
## Prerequisites & Usage
Please check the [docs](https://docs.hexfellow.com/hex-base/ros_en/)
## Publish
| Topic           | Msg Type                     | Description               |
| --------------- | ---------------------------- | ------------------------- |
| /`ws_down`      | `std_msgs/UInt8MultiArray`   | Message from ws           |
| /`motor_status` | `sensor_msgs/JointState`     | Every single motor status |
| /`real_vel`     | `geometry_msgs/TwistStamped` | Real velocity             |
| /`odom`         | `nav_msgs/Odometry`          | Odometry                  |
## Subscribe
| Topic         | Msg Type                   | Description           |
| ------------- | -------------------------- | --------------------- |
| /`ws_up`      | `std_msgs/UInt8MultiArray` | Message to send to ws |
| /`joint_ctrl` | `sensor_msgs/JointState`   | Joint control command |
| /`cmd_vel`    | `geometry_msgs/Twist`      | Cmd velocity          |
| /`clear_err` | `std_msgs/Bool` | Clear parking stop error |
## Parameter
| Name        | Data Type | Required | Default Value | Description           | Note |
| ----------- | --------- | -------- | ------------- | --------------------- | ---- |
| frame_id    | string    | yes      | base_link     | Robot base frame name |      |
| simple_mode | bool      | yes      | true          | Is simple mode        |      |
| report_freq | int       | yes      | 100           | Report frequency      |      |