# ROS2 Robot Navigation and Exploration Project

## 🎥 Video Demonstration

### Main System Demo

[![System Demo](https://img.shields.io/badge/🎥_Watch_Demo-Video-blue?style=for-the-badge)](https://www.bilibili.com/video/BV1NVYNzREpB/?share_source=copy_web&vd_source=fde5311dcf7c67e738fc96cf42870f16)

**Click the button above to watch the complete system demonstration**

<details>
<summary>📹 Video Preview (Click to expand)</summary>

<video width="800" height="600" controls>
  <source src="docs/videos/main_demo.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

</details>

**System Overview**: Complete demonstration of autonomous exploration with bidirectional A* path planning, DWA local planner, PID controller and RRT exploration strategy.

### Alternative Video Links

If the video doesn't display properly, you can:
- [Download the demo video](docs/videos/main_demo.mp4) directly
- View the video in the `docs/videos/` directory

---

A ROS2-based robot navigation and exploration system with multiple path planning algorithms and navigation capabilities.

## Project Overview

This project implements a complete robot navigation system including:
- Multiple path planning algorithms (A*, RRT, etc.)
- Robot exploration functionality
- Map building and navigation
- Gazebo simulation environment
- Sensor data processing

## Features

### Core Features
- **Path Planning**: Implements A* algorithm, RRT algorithm and other path planning methods
- **Robot Exploration**: Automatically explores unknown environments and builds maps
- **Navigation Control**: PID-based navigation controller
- **Sensor Integration**: Supports lidar, infrared sensors, etc.
- **Simulation Environment**: Complete Gazebo simulation environment

### Main Components
- `main.py`: Main exploration node with bidirectional A* and DWA
- `astar_planner.py`: Bidirectional A* path planning algorithm
- `bezier_smoother.py`: Bezier curve path smoothing
- `dwa_planner.py`: Dynamic Window Approach local planner
- `pid_navigator.py`: PID motion controller
- `RRT_target_selection.py`: RRT-based exploration point selection

## System Requirements

- Ubuntu 22.04
- ROS2 Humble
- Python 3.8+
- Gazebo Classic

## Install Dependencies

```bash
# Install ROS2 dependencies
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher

# Install Python dependencies
pip3 install numpy scipy matplotlib
```

## Build and Run

### 1. Build Workspace
```bash
cd /path/to/ros2_ws
colcon build
source install/setup.bash
```

### 2. Launch Simulation Environment
```bash
# Launch Gazebo simulation
ros2 launch first_pkg gazebo_sim.launch.py

# Launch robot model display
ros2 launch first_pkg display_robot.launch.py
```

### 3. Run Navigation Nodes
```bash
# Run main exploration node
ros2 run first_pkg exploration_node
```

## Project Structure

```
ros2_ws/
├── src/
│   ├── first_pkg/          # Main package
│   │   ├── first_pkg/      # Python source code
│   │   ├── launch/         # Launch files
│   │   ├── config/         # Configuration files
│   │   ├── urdf/           # Robot models
│   │   ├── map/            # Map files
│   │   └── models/         # Gazebo models
│   └── first_node/         # Basic node package
├── build/                  # Build output
├── install/                # Install files
└── log/                    # Log files
```

### Robot Model
Robot URDF models are located in `first_pkg/urdf/` directory, supporting:
- Lidar sensors
- Infrared sensors
- Differential drive chassis

## Usage Examples

### Basic Navigation
```bash
# 1. Launch simulation environment
ros2 launch first_pkg gazebo_sim.launch.py

# 2. Launch navigation
ros2 launch first_pkg lightweight_mapping.launch.py

# 3. Run exploration algorithm
ros2 run first_pkg exploration_node
```

### Map Building
```bash
# Launch lightweight map building
ros2 launch first_pkg lightweight_mapping.launch.py
```

## Development Guide

### Adding New Path Planning Algorithms
1. Create new Python file in `first_pkg/first_pkg/` directory
2. Implement path planning algorithm class
3. Add entry point in `setup.py`
4. Create corresponding launch file

### Customizing Robot Model
1. Modify XACRO files in `urdf/robot/` directory
2. Update sensor configuration
3. Rebuild and test

## Troubleshooting

### Common Issues
1. **Build errors**: Ensure all dependencies are correctly installed
2. **Simulation launch failure**: Check Gazebo version compatibility
3. **Navigation anomalies**: Verify sensor data is being published normally

### Debug Tools
```bash
# Check node status
ros2 node list

# Check topics
ros2 topic list

# Check services
ros2 service list
```

## Contributing

Welcome to submit Issues and Pull Requests to improve this project.

### Development Workflow
1. Fork the project
2. Create feature branch
3. Submit changes
4. Create Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact

- Maintainer: dong
- Email: dchengteng@gmail.com

## Acknowledgments

Thanks to the ROS2 community and all contributors for their support.
