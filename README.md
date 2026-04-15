# AMR - Autonomous Mobile Robot

A ROS2 Humble-based autonomous mobile robot with differential drive, 2D LIDAR SLAM, Nav2 autonomous navigation, and camera monitoring.

![Map](maps/my_map.png)

## Hardware

| Component | Model | Purpose |
|-----------|-------|---------|
| Compute | Raspberry Pi 4B | Main ROS2 computer |
| LIDAR | RPLidar A1/A2/A3 | 2D SLAM and obstacle detection |
| Camera | Pi Camera Module v3 | Visual monitoring |
| Motors | 2x Pro Range Planetary Gear DC | Differential drive with built-in hall effect encoders |
| Motor Drivers | 2x Cytron MD10C (10A each) | PWM + DIR motor control |
| Microcontroller | Arduino Uno | Motor PID control + encoder reading via USB serial |
| Drive | Differential (2 wheels + 1 caster) | |

## Repository Structure

```
amr/
├── description/          # Robot URDF/Xacro files
│   ├── robot.urdf.xacro  # Main robot description
│   ├── robot_core.xacro  # Chassis, wheels, caster
│   ├── lidar.xacro       # RPLidar sensor
│   ├── camera.xacro      # Pi Camera mount
│   ├── gazebo_control.xacro  # Gazebo diff_drive plugin
│   └── inertial_macros.xacro
├── launch/
│   ├── robot_bringup.launch.py  # Real robot: motors + LIDAR + camera
│   ├── slam_real.launch.py      # Real robot SLAM mapping
│   ├── nav_real.launch.py       # Real robot Nav2 navigation
│   ├── slam.launch.py           # Gazebo simulation SLAM
│   ├── navigation.launch.py     # Gazebo simulation Nav2
│   ├── launch_sim.launch.py     # Basic Gazebo simulation
│   └── rsp.launch.py            # Robot state publisher
├── config/
│   ├── motor_control_params.yaml    # Motor/encoder parameters
│   ├── slam_toolbox_real.yaml       # SLAM config for real hardware
│   ├── slam_toolbox_params.yaml     # SLAM config for simulation
│   ├── nav2_params.yaml             # Nav2 simulation params
│   ├── nav2_params_real.yaml        # Nav2 real hardware params
│   └── nav2_rviz.rviz              # RViz config with Nav2 toolbar
├── maps/
│   ├── my_map.pgm          # Saved SLAM occupancy grid
│   ├── my_map.yaml         # Map metadata
│   └── my_map.png          # Map preview image
├── arduino/
│   └── amr_motor_control/
│       └── amr_motor_control.ino  # Arduino firmware (PID + encoders)
├── motor_control_pkg/       # ROS2 motor control + odometry node
│   └── motor_control/
│       └── motor_control_node.py
├── docs/
│   └── wiring.md           # Wiring diagram and pin allocation
├── worlds/
│   └── empty.world         # Gazebo world file
└── README.md
```

## Software Stack

- **OS:** Ubuntu 22.04
- **ROS2:** Humble Hawksbill
- **SLAM:** slam_toolbox (async online SLAM)
- **Navigation:** Nav2 (AMCL + costmaps + DWB planner)
- **Simulation:** Gazebo Classic
- **Motor Control:** Custom Arduino firmware + ROS2 node via serial

## Setup

### Prerequisites

```bash
# ROS2 Humble (must be installed first)
source /opt/ros/humble/setup.bash

# Install dependencies
sudo apt install -y \
  ros-humble-rplidar-ros \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-v4l2-camera \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-gazebo-ros \
  ros-humble-tf2-ros \
  python3-serial
```

### Build

```bash
cd ~/amr_ws
colcon build --packages-select amr motor_control
source install/setup.bash
```

### Arduino Setup

1. Open `arduino/amr_motor_control/amr_motor_control.ino` in Arduino IDE
2. Wire motors and encoders per [docs/wiring.md](docs/wiring.md)
3. **Measure `TICKS_PER_REV`**: rotate one wheel exactly 1 revolution, note the tick count in serial monitor
4. Update `TICKS_PER_REV` in the `.ino` file and `encoder_ticks_per_rev` in `config/motor_control_params.yaml`
5. Upload firmware to Arduino Uno

## Usage

### 1. SLAM Mapping (build a map)

```bash
# With real hardware (motors + LIDAR)
ros2 launch amr slam_real.launch.py

# Drive the robot around with teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save the map when done (in another terminal)
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### 2. Autonomous Navigation (use saved map)

```bash
ros2 launch amr nav_real.launch.py map:=~/my_map.yaml
```

In RViz:
- Use **2D Pose Estimate** to set initial robot position
- Use **2D Goal Pose** to send navigation goals
- Robot navigates autonomously with costmap inflation borders around walls

### 3. Gazebo Simulation

```bash
# SLAM in simulation
ros2 launch amr slam.launch.py

# Navigation in simulation
ros2 launch amr navigation.launch.py map:=~/my_map.yaml

# Drive with teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 4. Robot Bringup Only (no SLAM/Nav)

```bash
# Start motors + LIDAR + camera
ros2 launch amr robot_bringup.launch.py
```

## Robot Specifications

| Parameter | Value |
|-----------|-------|
| Chassis | 0.3m x 0.3m x 0.15m |
| Wheel radius | 0.05m (measure and update) |
| Wheel separation | 0.35m (measure and update) |
| LIDAR range | 0.3m - 12m, 360 degrees |
| LIDAR update rate | 10 Hz |
| Map resolution | 0.05m (5cm grid) |
| Motor control rate | 50 Hz |
| Serial baud rate | 115200 |

## Architecture

```
                    ┌─────────────┐
                    │   RPi 4B    │
                    │   (ROS2)    │
                    ├─────────────┤
  /dev/ttyUSB0 ◄───┤  RPLidar    │───► /scan
                    │  slam_tool  │───► /map, map→odom TF
  /dev/ttyACM0 ◄───┤  motor_ctrl │───► /odom, odom→base_link TF
                    │  Nav2       │◄── /cmd_vel
        CSI    ◄───┤  camera     │───► /camera/image_raw
                    └─────────────┘
                          │
                    ┌─────┴─────┐
                    │  Arduino  │
                    │  PID ctrl │
                    ├───────────┤
                    │ MD10C #1  │──► Left Motor + Encoder
                    │ MD10C #2  │──► Right Motor + Encoder
                    └───────────┘
```

## Progress Log

### April 15, 2026
- Built and tested 2D LIDAR SLAM with slam_toolbox using real RPLidar
- Set up Nav2 autonomous navigation with costmap inflation borders
- Created and saved map of test environment
- Attempted RTAB-Map camera+LIDAR fusion -- abandoned (monocular camera lacks depth for visual loop closure)
- Created Arduino firmware for motor PID control with encoder feedback
- Created ROS2 motor_control node for odometry computation
- Set up full launch files for real hardware and Gazebo simulation
- Added wiring documentation and pin allocation

### Next Steps
- [ ] Get second Cytron MD10C motor driver
- [ ] Wire motors + encoders to Arduino per wiring diagram
- [ ] Upload Arduino firmware and measure encoder ticks per revolution
- [ ] Test motor_control_node with real hardware
- [ ] Tune PID parameters (Kp, Ki, Kd)
- [ ] Run SLAM with real odometry (expect significant map quality improvement)
- [ ] Test full Nav2 autonomous navigation
- [ ] Deploy to Raspberry Pi 4B via SSH

## License

MIT
