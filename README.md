# hex_vehicle
## Notice
1. Compile Protocol Buffer messages:  
Note: This package requires newer protoc. If compilation fails, please try to install protoc-27.1 using the binary installation method below. Installing protoc-27.1:
```
# For Linux x86_64
wget https://github.com/protocolbuffers/protobuf/releases/download/v27.1/protoc-27.1-linux-x86_64.zip
sudo unzip protoc-27.1-linux-x86_64.zip -d /usr/local
rm protoc-27.1-linux-x86_64.zip

# For Linux arm64
wget https://github.com/protocolbuffers/protobuf/releases/download/v27.1/protoc-27.1-linux-aarch_64.zip
sudo unzip protoc-27.1-linux-aarch_64.zip -d /usr/local
rm protoc-27.1-linux-aarch_64.zip

#  Verify installation
protoc --version  # Should show libprotoc 27.1
```
2. Must be cd to the directory where build_proto.sh is located first, then run:
```
./build_proto.sh
```
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
## Parameter
| Name        | Data Type | Required | Default Value | Description           | Note |
| ----------- | --------- | -------- | ------------- | --------------------- | ---- |
| frame_id    | string    | yes      | base_link     | Robot base frame name |      |
| simple_mode | bool      | yes      | true          | Is simple mode        |      |