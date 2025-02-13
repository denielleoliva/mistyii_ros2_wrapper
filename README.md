# ROS 2 Wrapper for Misty II

## Overview
This is a ROS 2 wrapper for Misty II, enabling control of Misty using ROS 2 topics and services. The node communicates with Misty’s API to send movement commands, retrieve battery status, and perform actions like speech and LED control.

## Features
- **Movement Control**: Uses `/cmd_vel` to drive Misty.
- **Speech**: Misty can speak text received via a ROS 2 topic.
- **LED Control**: Change Misty's LED color.
- **Battery Monitoring**: Publishes battery status.

## Prerequisites
### 1. Install ROS 2
Ensure you have **ROS 2 Foxy** or **Humble** installed. Follow [ROS 2 installation instructions](https://docs.ros.org/en/foxy/Installation.html).

### 2. Install Dependencies
```sh
pip install requests
```

### 3. Set Misty's IP Address
Edit `misty_ros2_wrapper.py` and update:
```python
self.misty_ip = '192.168.1.100'  # Replace with Misty's IP
```

## Installation
Clone and build the ROS 2 package:
```sh
cd ~/ros2_ws/src
git clone <your-repo-url> misty_ros2
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Usage
### Run the ROS 2 Node
```sh
ros2 run misty_ros2 misty_ros2_wrapper
```

### Movement Control
Publish a `geometry_msgs/Twist` message to `/cmd_vel`:
```sh
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

### Make Misty Speak
```sh
ros2 topic pub /misty/speak std_msgs/msg/String "data: 'Hello from ROS 2!'"
```

### Control Misty’s LED
```sh
ros2 topic pub /misty/led std_msgs/msg/String "data: 'red'"
```

### Get Battery Status
```sh
ros2 topic echo /misty/battery
```

## Future Enhancements
- Face recognition feedback
- Object detection
- RViz integration

## License
MIT License