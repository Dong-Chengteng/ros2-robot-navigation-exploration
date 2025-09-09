# API Documentation

## Core Modules

### LidarExplorer Class

The main exploration node class responsible for coordinating all exploration and navigation functions.

#### Initialization Parameters

```python
def __init__(self):
    """
    Initialize lidar exploration node
    
    Main functions:
    - Subscribe to map, odometry, and lidar data
    - Publish paths, goal markers, and velocity commands
    - Initialize various planners and controllers
    """
```

#### Main Methods

##### Path Planning Methods

```python
def plan_path(self, goal_x, goal_y):
    """
    Use bidirectional A* algorithm for path planning
    
    Args:
        goal_x (float): Target point x coordinate
        goal_y (float): Target point y coordinate
        
    Returns:
        list: List of path points, each point is (x, y) tuple
        None: If path planning fails
    """
```

```python
def smooth_path(self, path):
    """
    Smooth path using Bezier curves
    
    Args:
        path (list): Original path point list
        
    Returns:
        list: Smoothed path point list
    """
```

##### Exploration Methods

```python
def find_frontier_boundary_point(self):
    """
    Find boundary points for exploration
    
    Returns:
        tuple: (x, y) exploration target point coordinates
        None: If no suitable exploration point found
    """
```

```python
def find_rrt_exploration_point(self, robot_x, robot_y):
    """
    Use RRT exploration strategy to select optimal exploration point
    
    Args:
        robot_x (float): Robot current x coordinate
        robot_y (float): Robot current y coordinate
        
    Returns:
        tuple: (x, y) optimal exploration point coordinates
        None: If no suitable exploration point found
    """
```

##### Navigation Control Methods

```python
def navigation_callback(self):
    """
    Path tracking navigation callback function
    
    Functions:
    - Execute PID control or DWA control
    - Check navigation completion conditions
    - Update path points
    """
```

```python
def execute_dwa_control(self):
    """
    Execute DWA control logic
    
    State machine implementation:
    - IDLE: Idle state
    - SEARCHING_TARGET: Searching for local target point
    - NAVIGATING: Currently navigating
    - REACHED_TARGET: Target reached
    """
```

##### Obstacle Avoidance Methods

```python
def obstacle_avoidance_callback(self):
    """
    Real-time obstacle avoidance, dynamic PID/DWA mode switching
    
    Functions:
    - Detect obstacle distance
    - Automatically switch control modes
    - Manage DWA state
    """
```

##### Utility Methods

```python
def world_to_map(self, x, y):
    """
    Convert world coordinates to map coordinates
    
    Args:
        x (float): World coordinate x
        y (float): World coordinate y
        
    Returns:
        tuple: (map_x, map_y) map coordinates
    """
```

```python
def map_to_world(self, map_x, map_y):
    """
    Convert map coordinates to world coordinates
    
    Args:
        map_x (int): Map coordinate x
        map_y (int): Map coordinate y
        
    Returns:
        tuple: (world_x, world_y) world coordinates
    """
```

## Planner Modules

### AStarPlanner

Bidirectional A* path planning algorithm implementation.

#### Main Functions

```python
def bidirectional_astar(map_array, start, goal, is_accessible, map_to_world_fn, logger):
    """
    Bidirectional A* path planning algorithm
    
    Args:
        map_array (numpy.ndarray): Map array
        start (tuple): Start point coordinates (x, y)
        goal (tuple): Goal point coordinates (x, y)
        is_accessible (function): Node accessibility check function
        map_to_world_fn (function): Map coordinate to world coordinate function
        logger: ROS2 logger
        
    Returns:
        list: List of path points
        None: If no path found
    """
```

### DWAPlanner

Dynamic Window Approach local planner.

#### Main Methods

```python
def set_target(self, target_x, target_y):
    """
    Set DWA target point
    
    Args:
        target_x (float): Target point x coordinate
        target_y (float): Target point y coordinate
    """
```

```python
def generate_velocity_command(self):
    """
    Generate DWA velocity command
    
    Returns:
        Twist: ROS2 velocity message
    """
```

```python
def update_robot_state(self, odom_msg):
    """
    Update robot state
    
    Args:
        odom_msg (Odometry): Odometry message
    """
```

### PIDNavigator

PID motion controller.

#### Main Functions

```python
def compute_cmd(pose, path, params):
    """
    Compute PID control command
    
    Args:
        pose (Pose): Current robot pose
        path (list): List of path points
        params (dict): PID parameters
        
    Returns:
        tuple: (cmd_vel, reached_goal, pop_waypoint, path_offset)
    """
```

```python
def reset_pid_controllers():
    """
    Reset PID controller state
    """
```

## Exploration Strategy Modules

### RRTExplorationSelector

RRT-based exploration point selector.

#### Main Methods

```python
def set_parameters(self, max_iterations, step_size, goal_sample_rate, 
                   laser_range, information_radius, distance_weight, 
                   information_weight, exploration_weight):
    """
    Set RRT exploration parameters
    
    Args:
        max_iterations (int): Maximum number of iterations
        step_size (float): Step size
        goal_sample_rate (float): Goal sampling rate
        laser_range (float): Lidar range
        information_radius (float): Information radius
        distance_weight (float): Distance weight
        information_weight (float): Information weight
        exploration_weight (float): Exploration weight
    """
```

```python
def select_best_exploration_point(self, robot_position, map_array, 
                                  safety_map, origin, resolution):
    """
    Select optimal exploration point
    
    Args:
        robot_position (tuple): Robot position (x, y)
        map_array (numpy.ndarray): Map array
        safety_map (numpy.ndarray): Safety map
        origin (tuple): Map origin
        resolution (float): Map resolution
        
    Returns:
        tuple: (x, y) optimal exploration point coordinates
        None: If no suitable exploration point found
    """
```

### TargetSelector

Target point selector.

#### Main Methods

```python
def check_path_safety(self, path, map_array, origin, resolution):
    """
    Check path safety
    
    Args:
        path (list): List of path points
        map_array (numpy.ndarray): Map array
        origin (tuple): Map origin
        resolution (float): Map resolution
        
    Returns:
        tuple: (is_safe, error_msg)
    """
```

## Smoothing Algorithm Modules

### BezierSmoother

Bezier curve path smoother.

#### Main Functions

```python
def smooth_path(path, is_accessible_fn, logger):
    """
    Smooth path using Bezier curves
    
    Args:
        path (list): Original path point list
        is_accessible_fn (function): Accessibility check function
        logger: Logger
        
    Returns:
        list: Smoothed path point list
    """
```

## Configuration Parameters

### Exploration Parameters

```python
# Safety distance
safety_distance = 0.35  # meters

# Lookahead distance
lookahead_distance = 0.5  # meters

# DWA activation threshold
dwa_obstacle_threshold = 0.35  # meters

# Local target point reach threshold
local_target_reached_threshold = 0.20  # meters
```

### Control Parameters

```python
# Maximum speed
max_linear_speed = 2.0  # m/s
max_angular_speed = 2.0  # rad/s

# PID parameters
angle_kp = 1.0
angle_ki = 0.05
angle_kd = 0.15
angle_threshold = math.radians(5)
```

### RRT Parameters

```python
# RRT exploration parameters
max_iterations = 1000
step_size = 0.3
goal_sample_rate = 0.15
laser_range = 8.0
distance_weight = 0.4
information_weight = 0.6
exploration_weight = 0.0
```

## Topic Interface

### Subscribed Topics

- `/map` (nav_msgs/OccupancyGrid): SLAM map
- `/odom` (nav_msgs/Odometry): Odometry data
- `/scan` (sensor_msgs/LaserScan): Lidar data

### Published Topics

- `/lidar_path` (nav_msgs/Path): Planned path
- `/lidar_goal_marker` (visualization_msgs/Marker): Goal point marker
- `/local_target_marker` (visualization_msgs/Marker): Local target point marker
- `/cmd_vel` (geometry_msgs/Twist): Velocity command
- `/safety_map` (nav_msgs/OccupancyGrid): Safety map

## Error Handling

### Common Error Types

1. **Path Planning Failure**
   - Target point unreachable
   - Map data anomalies
   - Start/goal points out of map range

2. **Exploration Point Selection Failure**
   - Map not initialized
   - Robot position anomalies
   - No available boundary points

3. **Navigation Control Anomalies**
   - Insufficient path points
   - Missing sensor data
   - Control parameter anomalies

### Error Recovery Mechanisms

- Automatic retry mechanism
- Failed target point recording
- Safety stop mechanism
- State reset functionality
