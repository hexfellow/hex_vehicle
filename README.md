# hex_vehicle
## Overview
This is a ROS packdge that provides ROS interface for hex vehicle.
## Prerequisites & Usage
Please check the [docs](https://docs.hexfellow.com/hex-base/ros_en/)
## Publish
| Topic           | Msg Type                   | Description               |
| --------------- | -------------------------- | ------------------------- |
| /`ws_down`      | `std_msgs/UInt8MultiArray` | Message from ws           |
| /`motor_states` | `sensor_msgs/JointState`   | Every single motor status |
| /`odom`         | `nav_msgs/Odometry`        | Odometry                  |
## Subscribe
| Topic         | Msg Type                   | Description              |
| ------------- | -------------------------- | ------------------------ |
| /`ws_up`      | `std_msgs/UInt8MultiArray` | Message to send to ws    |
| /`joint_ctrl` | `sensor_msgs/JointState`   | Joint control command    |
| /`cmd_vel`    | `geometry_msgs/Twist`      | Cmd velocity             |
| /`clear_err`  | `std_msgs/Bool`            | Clear parking stop error |
## Parameter
| Name        | Data Type | Required | Default Value | Description           | Note |
| ----------- | --------- | -------- | ------------- | --------------------- | ---- |
| frame_id    | string    | yes      | base_link     | Robot base frame name |      |
| simple_mode | bool      | yes      | true          | Is simple mode        |      |
| report_freq | int       | yes      | 100           | Report frequency      |      |
## Easy Usage
1. Create a workspace:
   ```bash
   mkdir -p ~/hex_ws/src
   cd ~/hex_ws/src
   ```
2. Clone the repository:
   ```bash
   git clone --recursive https://github.com/hexfellow/hex_vehicle.git 
   git clone --recursive https://github.com/hexfellow/hex_bridge.git
   ```
3. Install dependencies:
   ```bash
   cd ~/hex_ws/src/hex_vehicle
   python3 -m pip install -r requirements.txt
   ```
3. Compile Protocol Buffer messages:
   ```bash
    cd ~/hex_ws/src/hex_vehicle
    ./build_proto.sh
    ```
4. build and source:
   ```bash
   cd ~/hex_ws
   colcon build
   source install/setup.bash
   ```
5. run:
   ```bash
   ros2 launch hex_vehicle chassis_bringup.launch.py url:={YOUR_ID}:8439 enable_bridge:=true
   ```