# Tutorial Guide

This tutorial will guide you from scratch to learn how to use the ROS2 Robot Navigation and Exploration System, including basic concepts, practical operations, and advanced applications.

## Table of Contents

1. [Quick Start](#quick-start)
2. [Basic Concepts](#basic-concepts)
3. [System Architecture](#system-architecture)
4. [Practical Operations](#practical-operations)
5. [Parameter Tuning](#parameter-tuning)


## Quick Start

### Step 1: Environment Setup

```bash
# 1. Clone project
git clone https://github.com/yourusername/ros2-smart-exploration.git
cd ros2-smart-exploration

# 2. Install dependencies
sudo apt install ros-humble-navigation2 ros-humble-slam-toolbox
pip3 install -r requirements.txt

# 3. Build project
colcon build --symlink-install
source install/setup.bash
```

### Step 1.5: Demo Materials Setup

The project includes demo materials located at:
```
E:\剪映\机器人导航素材\
```

This directory contains:
- **Simulation videos**: Demonstration of the exploration system in action
- **Performance analysis**: Videos showing system behavior under different conditions
- **Parameter tuning examples**: Visual examples of different parameter configurations
- **Troubleshooting guides**: Video solutions for common issues

**Note**: Make sure to have the demo materials available for reference during system operation and parameter tuning.

### Step 2: Launch Simulation

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch first_pkg gazebo_sim.launch.py

# Terminal 2: Launch mapping
ros2 launch first_pkg lightweight_mapping.launch.py

# Terminal 3: Launch exploration
ros2 run first_pkg exploration_node
```

### Step 3: Visualization

```bash
# Launch RViz to view exploration process
ros2 run rviz2 rviz2 
```

## Basic Concepts

### Exploration System Overview

The ROS2 Robot Navigation and Exploration System is an autonomous exploration platform based on multiple advanced algorithms, mainly including the following core components:

#### 1. Path Planning Module
- **Bidirectional A* Algorithm**: Efficient global path planning
- **DWA Local Planner**: Real-time obstacle avoidance and local optimization
- **Bezier Curve Smoothing**: Path smoothing processing

#### 2. Exploration Strategy Module
- **RRT Exploration Strategy**: Intelligent exploration based on boundary point evaluation
- **Target Point Selection**: Comprehensive consideration of distance, information gain, and exploration efficiency
- **Exploration Point Evaluation**: Dynamic evaluation of exploration point value

#### 3. Control Module
- **PID Motion Control**: Precise robot motion control
- **Adaptive Control**: Dynamic control mode switching based on environment
- **Safety Control**: Multiple safety guarantee mechanisms

### Key Terms

| Term | Description |
|------|-------------|
| **Exploration Point** | Target position the robot needs to reach |
| **Boundary Point** | Boundary between known and unknown areas |
| **Safety Map** | Map representation considering safety distance |
| **Lookahead Distance** | Forward distance considered by local planner |
| **DWA Threshold** | Obstacle distance that triggers Dynamic Window Approach |

## System Architecture

### Overall Architecture Diagram

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Sensor Data   │    │ Exploration     │    │ Path Planning   │
│  - Lidar        │───▶│ Strategy        │───▶│  - Bidirectional│
│  - Odometry     │    │  - RRT Selector │    │    A*           │
│  - Map Data     │    │  - Target Eval  │    │  - DWA Planner  │
└─────────────────┘    │  - Boundary Det │    │  - Path Smooth  │
         │              └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ Data Fusion     │    │ Decision System │    │ Motion Control  │
│  - Map Update   │    │  - Mode Switch  │    │  - PID Control  │
│  - State Est    │    │  - Safety Check │    │  - Speed Limit  │
│  - Info Fusion  │    │  - Strategy Sel │    │  - Safety Stop  │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Data Flow Diagram

```
Sensor Data → Data Preprocessing → Exploration Decision → Path Planning → Motion Control → Robot Execution
     ↑                                                                                           │
     └─────────────────── Feedback Loop ──────────────────────────────────────────────────────┘
```

## Practical Operations

### Basic Operation Workflow

#### 1. Launch System

```bash
# Launch complete exploration system
ros2 launch first_pkg gazebo_sim.launch.py &
ros2 launch first_pkg lightweight_mapping.launch.py &
ros2 run first_pkg exploration_node
```

#### 2. Monitor Exploration Process

```bash
# View exploration topics
ros2 topic list | grep lidar

# Monitor exploration progress
ros2 topic echo /lidar_path
ros2 topic echo /lidar_goal_marker
```

#### 3. Adjust Parameters

```bash
# View current parameters
ros2 param list /lidar_explorer_double_astar

# Modify parameters
ros2 param set /lidar_explorer_double_astar safety_distance 0.4
```

## Parameter Tuning

### Core Parameters

#### Exploration Parameters
- `safety_distance`: Safety distance for path planning (default: 0.35m)
- `lookahead_distance`: Lookahead distance for local planning (default: 0.5m)
- `dwa_obstacle_threshold`: DWA activation threshold (default: 0.35m)

#### Control Parameters
- `max_linear_speed`: Maximum linear velocity (default: 2.0 m/s)
- `max_angular_speed`: Maximum angular velocity (default: 2.0 rad/s)
- `angle_kp/ki/kd`: PID angle control parameters

### Tuning Strategies

#### Conservative Configuration (High Safety)
```python
safety_distance = 0.5
max_linear_speed = 1.0
dwa_obstacle_threshold = 0.4
```

#### Aggressive Configuration (High Efficiency)
```python
safety_distance = 0.2
max_linear_speed = 2.5
dwa_obstacle_threshold = 0.25
```

#### Balanced Configuration (Recommended)
```python
safety_distance = 0.35
max_linear_speed = 2.0
dwa_obstacle_threshold = 0.35
```

## Advanced Applications

### Multi-Robot Coordination
- Coordinate multiple robots for efficient exploration
- Implement distributed exploration strategies
- Handle inter-robot communication

### Dynamic Environment Adaptation
- Adapt to changing environments
- Handle dynamic obstacles
- Real-time parameter adjustment

### Learning-Based Exploration
- Implement machine learning algorithms
- Learn from exploration experience
- Optimize exploration strategies

## Performance Optimization Issues

### Critical Performance Problems

The current system has several performance bottlenecks that need attention:

#### 1. PID Control Parameter Optimization Issues

**Problem**: PID control parameters are not optimized for different scenarios, leading to:
- Poor control performance in complex environments
- Oscillatory behavior during navigation
- Inefficient energy consumption

**Impact**: 
- Reduced navigation accuracy
- Increased exploration time
- Higher computational overhead

**Solutions**:
```python
# Adaptive PID parameters based on environment complexity
def adaptive_pid_params(environment_complexity):
    if environment_complexity == "simple":
        return {"kp": 1.0, "ki": 0.1, "kd": 0.05}
    elif environment_complexity == "complex":
        return {"kp": 0.8, "ki": 0.05, "kd": 0.1}
    else:  # very_complex
        return {"kp": 0.6, "ki": 0.02, "kd": 0.15}
```

#### 2. Large Map Performance Degradation

**Problem**: When maps become very large during late-stage mapping, the system experiences:
- Significant computational delays
- Memory consumption issues
- Real-time performance degradation
- System lag and responsiveness problems

**Root Causes**:
- Inefficient map data structures
- Lack of map segmentation and caching
- Memory leaks in long-running processes
- Unoptimized path planning algorithms for large maps

**Performance Impact**:
- **Small maps (< 1000x1000)**: Normal performance
- **Medium maps (1000x1000 - 5000x5000)**: Noticeable delays
- **Large maps (> 5000x5000)**: Severe performance degradation

**Optimization Strategies**:
```python
# Map segmentation for large environments
class MapSegmenter:
    def __init__(self, segment_size=1000):
        self.segment_size = segment_size
        self.active_segments = {}
    
    def get_relevant_segments(self, robot_position, radius):
        # Only load segments within exploration radius
        relevant_segments = []
        # Implementation for efficient segment loading
        return relevant_segments

# Memory management for large maps
class MapMemoryManager:
    def __init__(self, max_memory_mb=512):
        self.max_memory = max_memory_mb * 1024 * 1024
        self.current_usage = 0
    
    def cleanup_unused_segments(self):
        # Remove segments not currently needed
        pass
```

#### 3. RRT Target Selection Computational Complexity

**Problem**: The RRT tree-based target point search method has excessive computational time:
- Exponential time complexity in complex environments
- Long planning delays (several seconds)
- Real-time performance issues
- Exploration efficiency degradation

**Current Issues**:
- **Simple environments**: RRT works well (< 0.1s)
- **Complex environments**: RRT becomes slow (1-5s)
- **Very complex environments**: RRT becomes unusable (> 10s)

**Optimization Approaches**:

1. **Hybrid Search Strategy**:
```python
class HybridTargetSelector:
    def __init__(self):
        self.simple_threshold = 100  # boundary points
        self.complex_threshold = 500
    
    def select_target(self, boundary_points):
        if len(boundary_points) < self.simple_threshold:
            return self.greedy_selection(boundary_points)
        elif len(boundary_points) < self.complex_threshold:
            return self.optimized_rrt(boundary_points)
        else:
            return self.hierarchical_selection(boundary_points)
```

2. **Parallel RRT Implementation**:
```python
import multiprocessing as mp

class ParallelRRT:
    def __init__(self, num_workers=4):
        self.num_workers = num_workers
        self.pool = mp.Pool(num_workers)
    
    def parallel_target_evaluation(self, candidates):
        # Distribute RRT evaluation across multiple processes
        results = self.pool.map(self.evaluate_target, candidates)
        return max(results, key=lambda x: x.score)
```

3. **Caching and Precomputation**:
```python
class TargetCache:
    def __init__(self, cache_size=1000):
        self.cache = {}
        self.cache_size = cache_size
    
    def get_cached_target(self, map_hash, robot_pose):
        key = f"{map_hash}_{robot_pose}"
        return self.cache.get(key)
    
    def cache_target(self, map_hash, robot_pose, target):
        if len(self.cache) >= self.cache_size:
            self.cache.popitem()  # Remove oldest entry
        self.cache[f"{map_hash}_{robot_pose}"] = target
```

### Performance Monitoring

```bash
# Monitor system performance
ros2 topic echo /performance_metrics

# Check memory usage
ros2 topic echo /memory_usage

# Monitor computation time
ros2 topic echo /computation_time
```

## Troubleshooting

### Common Issues

1. **Low Exploration Efficiency**
   - Check safety distance settings
   - Verify exploration strategy parameters
   - Monitor path planning performance
   - **New**: Check for RRT computation delays

2. **Frequent Collisions**
   - Increase safety distance
   - Adjust DWA threshold
   - Check sensor data quality
   - **New**: Verify PID parameter optimization

3. **Path Planning Failures**
   - Verify goal point accessibility
   - Check map data integrity
   - Adjust algorithm parameters
   - **New**: Check large map performance issues

4. **System Performance Degradation**
   - Monitor memory usage during long runs
   - Check for RRT computation timeouts
   - Verify PID control stability
   - **New**: Implement map segmentation for large environments

### Debug Tools

```bash
# Enable detailed logging
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG

# Monitor system performance
ros2 topic echo /lidar_path
ros2 topic echo /safety_map

# Check node status
ros2 node list
ros2 node info /lidar_explorer_double_astar
```

## Best Practices

### General Guidelines
1. **Progressive Tuning**: Start with conservative parameters and gradually optimize
2. **Multi-Environment Testing**: Test in different environments to validate parameters
3. **Performance Monitoring**: Continuously monitor system performance metrics
4. **Safety First**: Always prioritize safety over efficiency
5. **Documentation**: Record parameter tuning processes and results

### Performance Optimization Best Practices

#### 1. PID Control Optimization
- **Environment-Adaptive Tuning**: Adjust PID parameters based on environment complexity
- **Real-time Monitoring**: Monitor control performance and adjust parameters dynamically
- **Energy Efficiency**: Balance control accuracy with energy consumption
- **Stability Analysis**: Ensure system stability under various conditions

#### 2. Large Map Management
- **Proactive Segmentation**: Implement map segmentation before performance degradation
- **Memory Monitoring**: Continuously monitor memory usage during long runs
- **Cache Management**: Implement intelligent caching strategies for frequently accessed map regions
- **Progressive Loading**: Load map segments on-demand rather than all at once

#### 3. RRT Algorithm Optimization
- **Hybrid Approaches**: Use different algorithms based on environment complexity
- **Parallel Processing**: Implement multi-threaded RRT evaluation
- **Caching Strategy**: Cache frequently used target evaluations
- **Early Termination**: Implement timeout mechanisms for RRT computation

#### 4. System Monitoring
```bash
# Continuous performance monitoring script
#!/bin/bash
while true; do
    echo "=== Performance Metrics $(date) ==="
    ros2 topic echo /performance_metrics --once
    ros2 topic echo /memory_usage --once
    ros2 topic echo /computation_time --once
    sleep 10
done
```

#### 5. Demo Materials Integration
- **Reference Videos**: Use demo materials in `E:\剪映\机器人导航素材\` for parameter tuning reference
- **Performance Comparison**: Compare current performance with demo videos
- **Visual Validation**: Use video analysis to validate system behavior
- **Troubleshooting Guide**: Reference video solutions for common issues

---

**Note**: This tutorial is based on the current system version. Some content may need adjustment as the system updates. For questions, please check the latest documentation or submit an Issue.
