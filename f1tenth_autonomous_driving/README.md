# F1TENTH Autonomous Driving Package

This package provides autonomous driving capabilities for F1TENTH vehicles using LiDAR data. The autonomous driver uses a gap-finding algorithm to navigate through the environment while avoiding obstacles.

## Features

- LiDAR-based obstacle detection
- Gap-finding algorithm for path planning
- Configurable safety parameters
- Smooth speed and steering control

## Dependencies

- ROS2 Foxy or newer
- Python 3.8 or newer
- NumPy

## Installation

1. Clone this package into your ROS2 workspace:
```bash
cd ~/f1tenth_ws/src
git clone <repository-url>
```

2. Build the package:
```bash
cd ~/f1tenth_ws
colcon build --packages-select f1tenth_autonomous_driving
```

3. Source the workspace:
```bash
source ~/f1tenth_ws/install/setup.bash
```

## Usage

1. Launch the autonomous driver:
```bash
ros2 launch f1tenth_autonomous_driving autonomous_driver.launch.py
```

2. The autonomous driver will:
   - Subscribe to `/scan` topic for LiDAR data
   - Publish drive commands to `/drive` topic
   - Automatically navigate while avoiding obstacles

## Parameters

The following parameters can be configured:

- `max_speed` (default: 2.0 m/s): Maximum vehicle speed
- `max_steering_angle` (default: 0.34 rad): Maximum steering angle
- `safety_distance` (default: 0.5 m): Minimum distance to obstacles
- `lookahead_distance` (default: 1.0 m): Distance to look ahead for path planning

## How It Works

1. The autonomous driver receives LiDAR scan data from the `/scan` topic
2. It processes the scan data to find the widest gap in front of the vehicle
3. Based on the gap location and width, it calculates:
   - Steering angle to center the vehicle in the gap
   - Speed based on the gap width (wider gaps allow higher speeds)
4. Drive commands are published to the `/drive` topic
5. The vehicle's drive system executes these commands

## Safety Features

- Emergency stop when obstacles are too close
- Speed reduction in narrow gaps
- Smooth steering and speed transitions
- Configurable safety parameters

## License

This package is licensed under the MIT License. See the LICENSE file for details. 