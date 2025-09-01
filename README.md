# hex_vehicle
## Notice
Must be cd to the directory where build_proto.sh is located first, then run:
```
./build_proto.sh
```
## Publish
| Topic           | Msg Type                     | Description               |
| --------------- | ---------------------------- | ------------------------- |
| /`ws_down`      | `std_msgs/UInt8MultiArray`   | Message from ws           |
| /`motor_status` | `sensor_msgs/JointState`     | Every single motor status |
| /`real_vel`     | `geometry_msgs/TwistStamped` | Real velocity             |
## Subscribe
| Topic         | Msg Type                   | Description           |
| ------------- | -------------------------- | --------------------- |
| /`ws_up`      | `std_msgs/UInt8MultiArray` | Message to send to ws |
| /`joint_ctrl` | `sensor_msgs/JointState`   | Joint control command |
| /`cmd_vel`    | `geometry_msgs/Twist`      | Cmd velocity          |
## Parameter
| Name           | Data Type | Required | Default Value | Description                        | Note |
| -------------- | --------- | -------- | ------------- | ---------------------------------- | ---- |
| rate_ros       | float     | yes      | 300           | ros rate                           |      |
| ~~rate_state~~ | float     | no       | 200           | Deprecated，it's useless parameter |      |
| frame_id       | string    | yes      | base_link     | robot base frame name              |      |
| simple_mode    | bool      | yes      | true          | is simple mode                     |      |