#!/usr/bin/env python3
"""
双向A*算法路径规划探索节点 + DWA局部规划器 + PID运动控制 + 贝塞尔平滑曲线 + 代价地图

本文件实现了一个基于激光雷达的探索节点，使用双向A*算法进行长距离路径规划，
并集成DWA局部规划器进行实时避障和局部路径优化。

主要特性：
1. 智能选择算法：根据距离障碍物距离自动选择PID或者DWA
2. 高效路径规划：双向搜索减少了搜索空间，提升了性能
3. 详细性能统计：记录前向/后向搜索的扩展节点数
4. 探索点策略：根据全局边界点进行RRT最优选择探索点
5. DWA局部规划：实时避障、平滑控制、动态窗口优化

作者：dong
版本：7.0 
"""

import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from nav2_msgs.action import NavigateToPose
import numpy as np
import random
import math
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
import transforms3d
import heapq
import cv2

import time

# 导入DWA局部规划器
from .dwa_planner import DWAPlanner

# 导入A*算法
from .astar_planner import bidirectional_astar, standard_astar

# 导入目标选择器
from .target_selector import TargetSelector

# 导入RRT探索选择器
from .RRT_target_selection import RRTExplorationSelector

# 导入PID导航器
from .pid_navigator import compute_cmd, reset_pid_controllers

#导入贝塞尔平滑曲线
from .bezier_smoother import smooth_path as bezier_smooth

class LidarExplorer(Node):
    def __init__(self):
        super().__init__('lidar_explorer_double_astar')

        # 订阅器
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # 发布器
        self.path_pub = self.create_publisher(Path, '/lidar_path', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/lidar_goal_marker', 10)
        self.local_target_marker_pub = self.create_publisher(Marker, '/local_target_marker', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safety_map_pub = self.create_publisher(OccupancyGrid, '/safety_map', 10)
        
        # TF监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 地图相关
        self.map = None
        self.safety_map = None  # 安全地图，与SLAM地图实时同步
        self.safety_distance = 0.35  # 安全距离（米）
        self.resolution = 0.05  # 修正为实际地图分辨率0.05米/像素
        self.origin = (0.0, 0.0)
        self.current_pose = None
        self.last_pose = None
        self.current_scan = None
        self.current_path = None
        self.global_exploration_goal = None
        self.is_navigating = False
        
        # 探索相关
        self.target_selector = TargetSelector(logger=self.get_logger())
        self.rrt_selector = RRTExplorationSelector(logger=self.get_logger())
        self.failed_goals = set()
        self.goal_retry_count = 0
        self.max_retry_count = 3
        
        # 探索策略选择
        self.exploration_strategy = "rrt"  # 使用RRT策略
        
        # 激光雷达相关
        self.scan_ranges = None
        self.scan_angles = None
        self.scan_min_range = 0.0
        self.scan_max_range = 8.0
        self.scan_angle_increment = 0.0
        
        # DWA相关
        self.dwa_planner = DWAPlanner()
        self.use_dwa_mode = False  # 是否使用DWA模式
        self.dwa_obstacle_threshold = 0.35 # DWA避障距离阈值（距离障碍物0.35m以内，就开启DWA模式）
        
        # 局部目标点管理
        self.current_local_target = None  # 当前局部目标点
        self.local_target_reached_threshold = 0.20  # 到达局部目标点的距离阈值
        self.local_target_initialized = False  # 是否已初始化局部目标点
        
        # 前瞻距离相关参数
        self.lookahead_distance = 0.5  # 前瞻距离（米）- 增加以避免拐弯内切
        self.lookahead_angle_range = math.pi * 1 / 3  # 前方角度范围（60度）
        
        # 障碍物检测
        self.obstacle_detected_flag = False
        self.obstacle_detection_time = 0.0
        self.consecutive_obstacle_detections = 0  # 连续障碍物检测次数
        self.dwa_activation_threshold = 1  # 需要连续检测到障碍物的次数才激活DWA（降低延迟）
        
        # DWA状态管理
        self.dwa_state = "IDLE"  # DWA状态：IDLE, SEARCHING_TARGET, NAVIGATING, REACHED_TARGET
        self.dwa_target_search_attempts = 0  # 目标点搜索尝试次数
        self.max_target_search_attempts = 5  # 最大搜索尝试次数
        
        
        
        # 定时器
        self.explore_timer = self.create_timer(2.0, self.explore_callback)
        self.navigation_timer = self.create_timer(0.1, self.navigation_callback)
        self.obstacle_avoidance_timer = self.create_timer(0.05, self.obstacle_avoidance_callback)
        self.local_target_marker_timer = self.create_timer(1.0, self.publish_local_target_marker_callback)
        
        self.get_logger().info('双向A*探索节点已启动')

    # ====================== 补充方法 =====================
    
    def publish_stop(self):
        """发布零速度一次，用于安全停止。"""
        try:
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_cmd)
        except Exception:
            pass

    # ==================== 坐标转换方法 ====================
    
    def world_to_map(self, x, y):
        """将世界坐标转换为地图坐标"""
        map_x = int((x - self.origin[0]) / self.resolution)
        map_y = int((y - self.origin[1]) / self.resolution)
        return map_x, map_y

    def map_to_world(self, map_x, map_y):
        """将地图坐标转换为世界坐标"""
        world_x = self.origin[0] + map_x * self.resolution
        world_y = self.origin[1] + map_y * self.resolution
        return world_x, world_y

    # ==================== 路径规划方法 ====================
    
    def create_safety_map(self, map_array, safety_distance=0.35):
        """
        创建安全地图：预先标记距离障碍物太近的点
        
        Args:
            map_array: 原始地图数组
            safety_distance: 安全距离（米）
            
        Returns:
            safety_map: 安全地图，True表示安全，False表示不安全
        """
        if map_array is None:
            return None
            
        height, width = map_array.shape
        safety_map = np.ones((height, width), dtype=bool)  # 默认都是安全的
        
        # 计算安全距离对应的像素数
        safety_pixels = int(safety_distance / self.resolution)
        
        # 遍历所有障碍物点，标记其周围的不安全区域
        for y in range(height):
            for x in range(width):
                if map_array[y][x] == 100:  # 如果是障碍物
                    # 在障碍物周围标记不安全区域
                    for dy in range(-safety_pixels, safety_pixels + 1):
                        for dx in range(-safety_pixels, safety_pixels + 1):
                            check_x = x + dx
                            check_y = y + dy
                            
                            # 检查边界
                            if (0 <= check_x < width and 0 <= check_y < height):
                                # 计算实际距离
                                distance = math.sqrt(dx*dx + dy*dy) * self.resolution
                                if distance <= safety_distance:
                                    safety_map[check_y][check_x] = False
        
        # self.get_logger().info(f'安全地图创建完成，安全距离: {safety_distance}m，安全像素: {safety_pixels}')
        return safety_map
    
    
    def plan_path(self, goal_x, goal_y):
        """使用双向A*算法进行路径规划"""
        if self.map is None or self.current_pose is None:
            self.get_logger().warn('地图或机器人位置未初始化，无法进行路径规划')
            return None
            
        # 获取起点和终点（使用地图索引坐标）
        start_x, start_y = self.world_to_map(self.current_pose.position.x, self.current_pose.position.y)
        goal_map_x, goal_map_y = self.world_to_map(goal_x, goal_y)
        
        if goal_map_x is None or goal_map_y is None:
            self.get_logger().warn(f'目标点 ({goal_x}, {goal_y}) 超出地图范围')
            return None
            
        # 检查目标点是否在地图范围内
        if not (0 <= goal_map_x < self.map.shape[1] and 0 <= goal_map_y < self.map.shape[0]):
            self.get_logger().warn(f'目标点 ({goal_map_x}, {goal_map_y}) 超出地图边界')
            return None
            
        # 检查目标点是否为障碍物
        if self.map[goal_map_y, goal_map_x] == 100:
            self.get_logger().warn(f'目标点 ({goal_map_x}, {goal_map_y}) 是障碍物')
            return None
        
        # 计算距离
        distance = math.sqrt((goal_map_x - start_x)**2 + (goal_map_y - start_y)**2)
        
        map_array = self.map
        
        # 使用安全地图进行路径规划
        if self.safety_map is None:
            self.get_logger().warn('安全地图未初始化，创建临时安全地图')
            safety_map = self.create_safety_map(map_array, self.safety_distance)
        else:
            safety_map = self.safety_map
        
        
        # 基于安全地图的可达性检查
        def is_node_accessible_safe(map_x, map_y, map_value):
            # 边界检查
            if not (0 <= map_x < safety_map.shape[1] and 0 <= map_y < safety_map.shape[0]):
                return False
            
            # 0表示安全区域，100表示危险区域
            if map_value == 0:
                return True
            else:
                return False
        
        is_accessible = is_node_accessible_safe
        map_to_world_fn = lambda mx, my: self.map_to_world(mx, my)

        # 检查终点的可达性
        goal_accessible = safety_map[goal_map_y][goal_map_x]
        
        if not goal_accessible:
            self.get_logger().warn(f'终点 ({goal_map_x}, {goal_map_y}) 在安全地图中标记为危险，不可达')
            return None

        # 使用双向A*算法进行路径规划
        self.get_logger().info(f'使用双向A*算法进行路径规划，距离: {distance:.1f}像素')
        # 将安全地图转换为A*算法可用的格式：True->0(空闲), False->100(障碍物)
        safety_map_for_astar = np.where(safety_map, 0, 100).astype(np.int8)
        path = bidirectional_astar(safety_map_for_astar, (start_x, start_y), (goal_map_x, goal_map_y), is_accessible, map_to_world_fn, self.get_logger())
        
        # 路径规划完成
        if path:
            self.get_logger().info(f'路径规划完成：包含 {len(path)} 个点')
            
            
            # 检查原始路径安全性
            is_safe, error_msg = self.target_selector.check_path_safety(
                path, self.map, self.origin, self.resolution
            )
            
            if not is_safe:
                self.get_logger().warn(f'原始路径安全性检查失败：{error_msg}')
                return None
            
            # 使用贝塞尔曲线平滑路径
            smoothed_path = self.smooth_path(path)
            if smoothed_path:
                # 检查平滑后路径安全性
                is_smooth_safe, smooth_error_msg = self.target_selector.check_path_safety(
                    smoothed_path, self.map, self.origin, self.resolution
                )
                
                if is_smooth_safe:
                    return smoothed_path
                else:
                    self.get_logger().warn(f'平滑后路径安全性检查失败：{smooth_error_msg}')
                    return path
            else:
                return path
        else:
            self.get_logger().warn('A*算法返回空路径')
        
        return None

    def smooth_path(self, path):
        """使用贝塞尔曲线平滑路径，减少急转弯（外部模块）"""
        if len(path) <= 5:
            return path
        # 移除安全检查，因为A*已经保证了路径安全
        return bezier_smooth(path, lambda x, y: True, self.get_logger())

#用于局部规划器在全局路径中局部目标的搜索
    def find_local_target_on_path(self, current_x, current_y, global_plan=None, lookahead_distance=None):
        """
        基于前瞻距离的局部目标点查找方法
        
        Args:
            current_x, current_y: 机器人当前位置
            global_plan: 全局路径点列表（如果为None则使用self.current_path）
            lookahead_distance: 前瞻距离（如果为None则使用self.lookahead_distance）
            
        Returns:
            局部目标点坐标 (x, y) 或 None
        """
        # 使用传入的参数或默认值
        if global_plan is None:
            global_plan = self.current_path
        if lookahead_distance is None:
            lookahead_distance = self.lookahead_distance
            
        if global_plan is None or len(global_plan) == 0:
            self.get_logger().warn('🎯 全局路径为空，无法计算局部目标点')
            return None
        
        if len(global_plan) < 2:
            self.get_logger().warn('🎯 路径点不足（少于2个），无法计算局部目标点')
            return None
        
        # 获取机器人当前朝向角度
        robot_angle = self.get_robot_current_angle()
        
        # 步骤1：遍历全局路径，找到机器人前方120度范围内最近的点
        closest_point_index = -1
        min_distance_in_range = float('inf')
        
        for i, point in enumerate(global_plan):
            # 计算路径点到机器人的距离
            distance = math.sqrt((point[0] - current_x)**2 + (point[1] - current_y)**2)
            
            # 计算路径点相对于机器人的角度
            point_angle = math.atan2(point[1] - current_y, point[0] - current_x)
            angle_diff = abs(self.normalize_angle(point_angle - robot_angle))
            
            # 检查是否在前方120度范围内
            if angle_diff <= self.lookahead_angle_range:
                if distance < min_distance_in_range:
                    min_distance_in_range = distance
                    closest_point_index = i
        
        # 如果没有找到前方120度范围内的点，返回None
        if closest_point_index == -1:
            return None
        
        # 步骤2：从最近点开始，沿着全局路径向前遍历，找到距离大于等于lookahead_distance的点
        for i in range(closest_point_index, len(global_plan)):
            point = global_plan[i]
            distance = math.sqrt((point[0] - current_x)**2 + (point[1] - current_y)**2)
            
            if distance >= lookahead_distance:
                return point
        
        return None
    
    
    def reset_local_target_state(self):
        """重置局部目标点状态（当DWA模式停止时调用）"""
        self.current_local_target = None
        self.local_target_initialized = False
    
    def update_path_index_after_dwa(self, current_x, current_y):
        """
        智能更新路径索引，跳过DWA已经走过的点
        
        Args:
            current_x, current_y: 机器人当前位置
        """
        if self.current_path is None or len(self.current_path) < 2:
            return
        
        # 获取机器人当前朝向角度
        robot_angle = self.get_robot_current_angle()
        
        # 找到距离机器人前方最近的点，舍弃该点之前的所有点
        closest_forward_index = -1
        min_forward_distance = float('inf')
        
        for i, point in enumerate(self.current_path):
            distance = math.sqrt((point[0] - current_x)**2 + (point[1] - current_y)**2)
            
            # 计算路径点相对于机器人的角度
            point_angle = math.atan2(point[1] - current_y, point[0] - current_x) + math.pi
            angle_diff = abs(self.normalize_angle(point_angle - robot_angle))
            
            # 判断是否在机器人前方：角度在前方120度范围内
            is_in_front = angle_diff <= self.lookahead_angle_range  # 120度
            
            if is_in_front and distance < min_forward_distance:
                min_forward_distance = distance
                closest_forward_index = i
        
        # 如果找到了前方最近的点，舍弃该点之前的所有点
        if closest_forward_index > 0:
            # 移除前方最近点之前的所有点
            for _ in range(closest_forward_index):
                if len(self.current_path) > 1:
                    self.current_path.pop(0)
    
    def quaternion_to_yaw(self, q):
        """将四元数转换为 yaw 角"""
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))
        return yaw
    
    def get_robot_current_angle(self):
        """获取机器人当前朝向角度"""
        if self.current_pose is None:
            return 0.0
        return self.quaternion_to_yaw(self.current_pose.orientation)
    
    def normalize_angle(self, angle):
        """将角度标准化到[-π, π]范围"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    

    # ==================== 安全检查方法 ====================

    #用于判断机器人现在是否在全局规划的路径上遇到了障碍物


    # ==================== 探索相关方法 ====================
    #探索点获取流程
    def explore_callback(self):
        """探索回调函数"""
        if not self.is_navigating:
            # 打印当前机器人位置
            if self.current_pose is not None:
                robot_x = self.current_pose.position.x
                robot_y = self.current_pose.position.y
                self.get_logger().info(f'当前机器人位置: ({robot_x:.2f}, {robot_y:.2f})')
            
            # 直接使用find_frontier_boundary_point进行探索
            goal = self.find_frontier_boundary_point()
            if goal is not None:
                goal_x, goal_y = goal
                # 计算距离
                robot_x = self.current_pose.position.x
                robot_y = self.current_pose.position.y
                distance = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
                self.get_logger().info(f'找到新的探索目标: ({goal_x:.2f}, {goal_y:.2f}), 距离: {distance:.2f}m')
                
                # 检查是否已经尝试过这个目标（使用更精确的比较）
                goal_key = (round(goal_x, 2), round(goal_y, 2))
                if goal_key in self.failed_goals:
                    self.get_logger().warn(f'目标点 {goal_key} 之前失败过，跳过')
                    return
                
                # 尝试规划路径
                path = self.plan_path(goal_x, goal_y)
                if path is not None:
                    # 全局路径规划成功
                    self.current_path = path
                    self.global_exploration_goal = (goal_x, goal_y)
                    self.goal_retry_count = 0
                    self.is_navigating = True
                    self.reset_pid_controllers()  # 重置PID控制器，开始新的导航
                    self.publish_path(path)
                    self.publish_goal_marker(goal_x, goal_y)
                    self.get_logger().info(f'开始导航到目标点: ({goal_x:.2f}, {goal_y:.2f})')
                else:
                    # 路径规划失败，记录失败目标并停止
                    self.failed_goals.add(goal_key)
                    self.goal_retry_count += 1
                    self.get_logger().warn(f'无法规划到目标点 ({goal_x:.2f}, {goal_y:.2f}) 的路径，停止机器人')
                    self.is_navigating = False
                    self.current_path = None
                    self.global_exploration_goal = None
                    self.use_dwa_mode = False  # 关闭DWA模式
                    self.reset_local_target_state()  # 重置局部目标点状态
                    self.reset_pid_controllers()  # 重置PID控制器
                    self.clear_path()  # 清除路径显示
                    self.publish_stop()
                    
                    # 如果重试次数过多，清除失败目标记录
                    if self.goal_retry_count >= self.max_retry_count:
                        self.get_logger().warn('重试次数过多，清除失败目标记录')
                        self.failed_goals.clear()
                        self.goal_retry_count = 0
            else:
                self.get_logger().warn('未找到合适的探索目标，停止机器人')
                self.is_navigating = False
                self.current_path = None
                self.global_exploration_goal = None
                self.use_dwa_mode = False  # 关闭DWA模式
                self.reset_local_target_state()  # 重置局部目标点状态
                self.reset_pid_controllers()  # 重置PID控制器
                self.clear_path()  # 清除路径显示
                self.publish_stop()

    #探索点搜索主程序
    def find_frontier_boundary_point(self):
        """寻找边界点进行探索"""
        if self.map is None:
            self.get_logger().warn('地图未初始化，无法寻找探索目标')
            return None
        
        if self.current_pose is None:
            self.get_logger().warn('机器人位置未接收到，请检查：1) Gazebo是否启动 2) /odom话题是否发布数据')
            return None
            
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        self.get_logger().info(f'寻找探索目标：机器人位置 ({robot_x:.2f}, {robot_y:.2f})')
        self.get_logger().info(f'地图参数：origin={self.origin}, resolution={self.resolution}, shape={self.map.shape}')
        
        robot_map_x, robot_map_y = self.world_to_map(robot_x, robot_y)
        
        # 检查地图坐标是否有效
        if not (0 <= robot_map_x < self.map.shape[1] and 0 <= robot_map_y < self.map.shape[0]):
            self.get_logger().warn(f'机器人地图坐标无效: ({robot_map_x}, {robot_map_y}), 地图尺寸: {self.map.shape}')
            return None
            
        self.get_logger().info(f'机器人地图坐标: ({robot_map_x}, {robot_map_y})')
        
        # 检查安全地图是否已初始化
        if self.safety_map is None:
            self.get_logger().warn('安全地图未初始化，创建临时安全地图')
            # 如果安全地图还没创建，先创建一个
            self.safety_map = self.create_safety_map(self.map, self.safety_distance)
        
        self.get_logger().info(f'安全地图状态: {self.safety_map.shape if self.safety_map is not None else "None"}')
        
        # 使用RRT探索策略
        if self.exploration_strategy == "rrt":
            # 使用RRT探索策略
            self.get_logger().info('使用RRT探索策略')
            rrt_point = self.find_rrt_exploration_point(robot_x, robot_y)
            if rrt_point is not None:
                self.get_logger().info(f'RRT探索找到目标点: {rrt_point}')
                return rrt_point
            else:
                self.get_logger().warn('RRT探索未找到目标点')
                return None   
        return None


    
    #探索点搜索主程序中第三个探索点搜索策略 - 边界点评估探索
    def find_rrt_exploration_point(self, robot_x, robot_y):
        """使用边界点评估算法进行探索点选择（评估所有边界点，选择得分最高的）"""
        if self.map is None or self.safety_map is None:
            self.get_logger().warn('地图或安全地图未初始化，无法进行RRT探索')
            return None
        
        # 设置边界点评估参数
        self.rrt_selector.set_parameters(
            max_iterations=1000,  # 不再使用RRT，此参数无效
            step_size=0.3,  # 不再使用RRT，此参数无效
            goal_sample_rate=0.15,  # 不再使用RRT，此参数无效
            laser_range=8.0,  # 激光雷达范围8米（用于探索面积计算）
            information_radius=2.0,  # 不再使用，此参数无效
            distance_weight=0.4,  # 距离权重（距离越短分数越高）
            information_weight=0.6,  # 探索权重（探索未知区域越大分数越高）
            exploration_weight=0.0  # 不使用探索效率权重
        )
        
        
        # 使用边界点评估选择最优探索点
        robot_position = (robot_x, robot_y)
        best_point = self.rrt_selector.select_best_exploration_point(
            robot_position, self.map, self.safety_map, self.origin, self.resolution
        )
        
        if best_point is not None:
            # 检查是否在失败目标列表中
            goal_key = (round(best_point[0], 2), round(best_point[1], 2))
            if goal_key in self.failed_goals:
                self.get_logger().warn(f'RRT选择的点 {goal_key} 之前失败过，跳过')
                return None
            
            # 获取探索统计信息
            stats = self.rrt_selector.get_exploration_statistics(
                robot_position, self.map, self.safety_map, self.origin, self.resolution
            )
            
            if stats:
                self.get_logger().info(f'探索统计: 未知区域{stats["unknown_ratio"]:.1f}%, '
                                     f'探索进度{stats["exploration_progress"]:.1f}%, '
                                     f'安全区域{stats["safe_ratio"]:.1f}%')
            
            # 输出详细的探索点信息
            distance = math.sqrt((best_point[0] - robot_x)**2 + (best_point[1] - robot_y)**2)
            self.get_logger().info(f'边界点评估探索成功: 目标点({best_point[0]:.2f}, {best_point[1]:.2f}), '
                                 f'距离机器人{distance:.2f}m')
            
            return best_point
        else:
            self.get_logger().warn('边界点评估探索未找到合适的探索点')
            return None

    # ==================== 导航控制方法 ====================
    
    def navigation_callback(self):
        """路径跟踪（增强版：包含路径障碍物检测）"""
        if not self.is_navigating:
            self.get_logger().debug('导航状态：未在导航中')
            return
        if self.current_path is None:
            self.get_logger().debug('导航状态：当前路径为空')
            return
        if len(self.current_path) < 2:
            self.get_logger().debug(f'导航状态：路径点不足，当前路径长度: {len(self.current_path)}')
            return
        
            
        self.get_logger().debug(f'导航状态：正在导航，路径长度: {len(self.current_path)}，目标点: {self.global_exploration_goal}')
            

        # 获取当前位置
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # 障碍物检测和DWA切换由 obstacle_avoidance_callback 统一处理
        # 这里不再重复处理，让 navigation_callback 专注于导航逻辑
        
        # 如果使用DWA模式，执行DWA控制
        if self.use_dwa_mode:
            self.get_logger().debug('使用DWA模式进行导航')
            self.execute_dwa_control()
            return
        else:
            self.get_logger().debug('使用PID模式进行导航')
        
        # 只在从DWA切换到PID时更新路径索引
        if hasattr(self, 'dwa_just_switched') and self.dwa_just_switched:
            self.update_path_index_after_dwa(current_x, current_y)
            self.dwa_just_switched = False  # 重置标志
        
        # 获取下一个目标点
        next_point = self.current_path[1]
        target_x = next_point[0]
        target_y = next_point[1]
        
        # 检查目标点是否在机器人后方，如果是则舍弃该点
        robot_angle = self.get_robot_current_angle()
        target_angle = math.atan2(target_y - current_y, target_x - current_x)
        angle_diff = abs(self.normalize_angle(target_angle - robot_angle))
        
        # 判断条件：目标点在机器人后方120度且距离在0.1m以内则舍弃
        distance_to_target = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        should_skip_target = False
        
        if angle_diff > math.pi * 2/3:  # 120度以上
            if distance_to_target < 0.1:  # 距离在0.1m以内
                should_skip_target = True
                self.get_logger().warn(f'⚠️ 目标点在后方120度且距离很近，舍弃该点: 角度差={math.degrees(angle_diff):.1f}°, 距离={distance_to_target:.2f}m')
            else:
                self.get_logger().info(f'🎯 目标点在后方但距离较远，允许旋转对齐: 角度差={math.degrees(angle_diff):.1f}°, 距离={distance_to_target:.2f}m')
        
        if should_skip_target:
            self.get_logger().warn(f'⚠️ 机器人位置=({current_x:.2f}, {current_y:.2f}), 目标位置=({target_x:.2f}, {target_y:.2f})')
            
            # 移除当前目标点
            if len(self.current_path) > 1:
                removed_point = self.current_path.pop(1)  # 移除索引1的点（当前目标点）
                self.get_logger().info(f'🗑️ 已移除后方目标点: ({removed_point[0]:.3f}, {removed_point[1]:.3f})')
                
                # 如果路径点不足，结束导航
                if len(self.current_path) < 2:
                    self.get_logger().warn('路径点不足，结束导航')
                    self.is_navigating = False
                    self.current_path = None
                    self.global_exploration_goal = None
                    self.use_dwa_mode = False
                    self.reset_local_target_state()
                    self.reset_pid_controllers()
                    self.clear_goal_marker()
                    self.clear_path()
                    self.publish_stop()
                    return
                
                # 更新目标点
                next_point = self.current_path[1]
                target_x = next_point[0]
                target_y = next_point[1]
                self.get_logger().info(f'🔄 更新目标点: ({target_x:.3f}, {target_y:.3f})')
                
                # 重新计算距离
                distance_to_target = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        # 计算到目标点的距离
        distance_to_target = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        # 检查导航结束条件：仅根据距离判断（探索完成检查已被注释掉）
        reached_target = distance_to_target < 0.5
        exploration_complete = False  # 探索完成检查已被禁用
        
        
        # 只根据距离条件结束导航（探索完成检查已被禁用）
        if reached_target or exploration_complete:
            if reached_target:
                self.get_logger().info('🎯 条件1满足：到达目标点')
            if exploration_complete:
                self.get_logger().info('🔍 条件2满足：目标点周围1米内探索完毕')
                
            # 处理路径点移除（只有真正到达时才移除）
            if reached_target:
                self.current_path.pop(0)
                if len(self.current_path) > 1:
                    self.get_logger().info('到达中间点，更新路径')
                    return  # 继续导航到下一个路径点
            
            # 结束当前导航任务
            self.get_logger().info('✅ 导航任务完成，准备寻找新的探索目标')
            self.is_navigating = False
            self.current_path = None
            self.global_exploration_goal = None
            self.original_goal = None
            self.use_dwa_mode = False  # 关闭DWA模式
            self.reset_local_target_state()  # 重置局部目标点状态
            self.reset_pid_controllers()  # 重置PID控制器
            self.clear_goal_marker()
            self.clear_path()  # 清除路径显示
            self.publish_stop()  # 立即发布停止命令
            return
        
        # 使用PID控制器计算速度命令
        params = {
            'max_linear_speed': 2.0,
            'max_angular_speed': 2.0,
            'linear_tolerance': 0.1,
            'angular_tolerance': 0.1,
            'debug_nav': True,
            # 优化角度PID参数，减少超调
            'angle_kp': 1.0,      # 降低比例系数，减少响应速度
            'angle_ki': 0.05,     # 降低积分系数，减少积分饱和
            'angle_kd': 0.15,     # 增加微分系数，增强阻尼效果
            'angle_threshold': math.radians(5),  # 降低角度阈值到5度，提高精度
        }
        cmd_vel, reached_goal, pop_waypoint, path_offset = compute_cmd(self.current_pose, self.current_path, params)
        
        # 输出PID导航关键调试信息
        self.get_logger().info(f'🎯 PID导航: 目标({target_x:.2f}, {target_y:.2f}), 位置({current_x:.2f}, {current_y:.2f}), 距离={distance_to_target:.2f}m, 偏移={path_offset:.2f}m')
        
        #（移除立即停止导航的逻辑）
        # 发布速度命令将在参数自适应后统一发送
        
        # 检查路径点密度，如果路径点太少，降低速度
        path_density_factor = 1.0
        if len(self.current_path) < 5:
            path_density_factor = 0.6  # 路径点太少时降低速度
        
        # 根据路径偏移调整控制参数
        if abs(path_offset) > 0.3:
            params['max_linear_speed'] *= 0.8
            params['max_angular_speed'] *= 1.1
        
        # 应用路径密度因子
        params['max_linear_speed'] *= path_density_factor
        
        
        # 重新计算速度命令
        cmd_vel, reached_goal, pop_waypoint, path_offset = compute_cmd(self.current_pose, self.current_path, params)
        
        # 发布速度命令
        if cmd_vel is not None and self.is_navigating:
            self.cmd_vel_pub.publish(cmd_vel)
        elif not self.is_navigating:
            self.publish_stop()
        
        # 如果到达中间点，更新路径
        if pop_waypoint:
            self.current_path.pop(0)

    def execute_dwa_control(self):
        """执行DWA控制逻辑 - 状态机实现"""
        # 检查导航状态
        if not self.is_navigating:
            self.use_dwa_mode = False
            self.dwa_state = "IDLE"
            self.publish_stop()
            return
            
        # 检查基本条件
        if self.current_path is None or len(self.current_path) < 1:
            self.use_dwa_mode = False
            self.dwa_state = "IDLE"
            self.publish_stop()
            return
        
        # 获取当前位置
        if self.current_pose is None:
            return
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # DWA状态机处理
        if self.dwa_state == "IDLE":
            # 状态1：开始寻找局部目标点
            self.dwa_state = "SEARCHING_TARGET"
            self.dwa_target_search_attempts = 0
            self.current_local_target = None
            self.local_target_initialized = False
            
        elif self.dwa_state == "SEARCHING_TARGET":
            # 状态2：寻找局部目标点
            self.publish_stop()
            self.dwa_target_search_attempts += 1
            
            # 尝试寻找局部目标点
            new_target = self.find_local_target_on_path(
                current_x, current_y, 
                global_plan=self.current_path, 
                lookahead_distance=self.lookahead_distance
            )
            
            if new_target is not None:
                # 找到目标点，进入导航状态
                self.current_local_target = new_target
                self.dwa_state = "NAVIGATING"
                self.dwa_target_search_attempts = 0
                
                # 发布局部目标点可视化
                self.publish_local_target_marker(new_target[0], new_target[1])
                
                # 设置DWA目标点
                self.dwa_planner.set_target(new_target[0], new_target[1])
            else:
                # 未找到局部目标点，直接使用全局目标点
                if self.global_exploration_goal is not None:
                    global_goal_x, global_goal_y = self.global_exploration_goal
                    self.current_local_target = (global_goal_x, global_goal_y)
                    self.dwa_state = "NAVIGATING"
                    self.dwa_target_search_attempts = 0
                    
                    # 发布局部目标点可视化
                    self.publish_local_target_marker(global_goal_x, global_goal_y)
                    
                    # 设置DWA目标点
                    self.dwa_planner.set_target(global_goal_x, global_goal_y)
                else:
                    # 没有全局目标点，检查是否超过最大尝试次数
                    if self.dwa_target_search_attempts >= self.max_target_search_attempts:
                        self.use_dwa_mode = False
                        self.dwa_state = "IDLE"
                        self.clear_local_target_marker()
                        self.publish_stop()
                        return
                    else:
                        return
                    
        elif self.dwa_state == "NAVIGATING":
            # 状态3：正在导航到局部目标点
            if self.current_local_target is None:
                self.dwa_state = "SEARCHING_TARGET"
                return
            
            # 检查是否到达目标点
            target_distance = math.sqrt((self.current_local_target[0] - current_x)**2 + (self.current_local_target[1] - current_y)**2)
            
            if target_distance < self.local_target_reached_threshold:
                # 到达目标点，进入到达状态
                self.dwa_state = "REACHED_TARGET"
                self.current_local_target = None
                self.clear_local_target_marker()
                self.publish_stop()
                return
            else:
                # 检查障碍物距离，如果太近则舍弃当前局部目标点
                min_obstacle_distance = float('inf')
                if self.scan_ranges is not None:
                    for range_val in self.scan_ranges:
                        if self.scan_min_range < range_val < self.scan_max_range:
                            if range_val < min_obstacle_distance:
                                min_obstacle_distance = range_val
                
                # 如果障碍物距离小于0.16米，舍弃当前局部目标点，重新选择
                if min_obstacle_distance < 0.16:
                    self.current_local_target = None
                    self.clear_local_target_marker()
                    self.dwa_state = "SEARCHING_TARGET"
                    self.publish_stop()
                    return
                
                
                # 生成DWA速度命令
                cmd_vel = self.dwa_planner.generate_velocity_command()
                
                # 发布速度命令
                self.cmd_vel_pub.publish(cmd_vel)
                
        elif self.dwa_state == "REACHED_TARGET":
            # 状态4：已到达目标点，检查是否到达全局目标点
            
            # 检查DWA导航结束条件：条件1-到达目标点 OR 条件2-目标点周围探索完毕
            if self.global_exploration_goal is not None:
                goal_x, goal_y = self.global_exploration_goal
                goal_distance = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
                
                # 条件1：到达目标点（距离全局目标点0.5米以内）
                reached_target = goal_distance < 0.5
                
                # 探索完成检查已被禁用
                exploration_complete = False
                
                # 只根据距离条件结束导航
                if reached_target or exploration_complete:
                    
                    # 结束当前导航任务
                    self.is_navigating = False
                    self.current_path = None
                    self.global_exploration_goal = None
                    self.original_goal = None
                    self.use_dwa_mode = False
                    self.dwa_state = "IDLE"
                    self.reset_local_target_state()
                    self.reset_pid_controllers()
                    self.clear_goal_marker()
                    self.clear_path()
                    self.publish_stop()
                    return
                else:
                    # 未满足结束条件，继续寻找新的局部目标点
                    pass
            else:
                # 没有全局目标点，继续寻找新的局部目标点
                pass
            
            # 寻找新的局部目标点
            self.publish_stop()
            self.dwa_state = "SEARCHING_TARGET"
            self.dwa_target_search_attempts = 0
            return


    #确定机器人的避障模式，是采用PID，还是采用DWA
    def obstacle_avoidance_callback(self):
        """实时避障，动态切换PID/DWA"""
        if self.current_scan is None:
            return
        
        # 检查360度激光雷达数据
        obstacle_detected = False
        
        # 检查激光雷达周围一圈（360度）
        for i, (range_val, angle) in enumerate(zip(self.scan_ranges, self.scan_angles)):
            # 检查激光雷达数据是否有效
            if self.scan_min_range < range_val < self.scan_max_range:
                # 如果距离小于阈值，认为有障碍物
                if range_val < self.dwa_obstacle_threshold:
                    obstacle_detected = True
                    break  # 检测到障碍物就立即退出循环
        
        # 动态切换PID/DWA模式
        if obstacle_detected:
            if not self.use_dwa_mode:
                self.use_dwa_mode = True
                self.dwa_state = "IDLE"
                self.dwa_target_search_attempts = 0
                self.get_logger().warn(f'检测到障碍物，切换到DWA模式 (阈值: {self.dwa_obstacle_threshold:.2f}m)')
        
        # 检查DWA模式是否应该结束（仅检查是否脱离障碍物）
        if self.use_dwa_mode:
            # 检查是否脱离障碍物
            min_obstacle_distance = float('inf')
            if self.scan_ranges is not None:
                for range_val in self.scan_ranges:
                    if self.scan_min_range < range_val < self.scan_max_range:
                        if range_val < min_obstacle_distance:
                            min_obstacle_distance = range_val
            
            # DWA结束条件：障碍物距离超过阈值
            if min_obstacle_distance > self.dwa_obstacle_threshold:
                # 脱离障碍物，切换回PID模式
                self.use_dwa_mode = False
                self.dwa_state = "IDLE"
                self.dwa_just_switched = True
                self.current_local_target = None
                self.clear_local_target_marker()
                self.reset_pid_controllers()
                self.get_logger().info(f'脱离障碍物，切换回PID模式：障碍物距离{min_obstacle_distance:.2f}m')
            else:
                # 仍有障碍物，继续DWA模式
                self.use_dwa_mode = True

    def reset_pid_controllers(self):
        """重置PID控制器"""
        reset_pid_controllers()
    
    

    # ==================== 回调函数方法 ====================
    
    def odom_callback(self, msg):
        """处理里程计数据"""
        self.last_pose = self.current_pose
        self.current_pose = msg.pose.pose
        
        
        # 更新DWA规划器的机器人状态
        self.dwa_planner.update_robot_state(msg)

    def scan_callback(self, msg):
        """处理激光雷达数据"""
        self.current_scan = msg
        
        # 更新激光雷达数据
        self.scan_ranges = np.array(msg.ranges)
        self.scan_min_range = msg.range_min
        self.scan_max_range = msg.range_max
        self.scan_angle_increment = msg.angle_increment
        
        # 计算每个激光束的角度
        self.scan_angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        
        # DWA规划器不再使用激光雷达数据，只使用地图信息

    def map_callback(self, msg):
        """处理地图数据"""
        # self.get_logger().info('收到地图数据')
        
        # 转换地图数据
        width = msg.info.width
        height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        
        # 将一维数组转换为二维数组
        map_data = np.array(msg.data).reshape((height, width))
        
        # 处理未知区域（-1值）
        map_data[map_data == -1] = -1  # 保持未知区域为-1
        
        self.map = map_data
        
        # 更新DWA规划器的地图信息
        self.dwa_planner.update_map(map_data, self.resolution, self.origin)
        
        # SLAM地图更新时实时同步更新安全地图
        self.safety_map = self.create_safety_map(map_data, self.safety_distance)
        
        # 地图更新时实时检查当前路径安全性
        self.check_current_path_safety_on_map_update()
        
        # 发布安全地图到RViz可视化
        if self.safety_map is not None:
            self.publish_safety_map(self.safety_map)

    def check_current_path_safety_on_map_update(self):
        """
        地图更新时实时检查当前路径的安全性
        如果发现路径不安全，立即停止导航并重新规划
        """
        # 只有在导航状态下才检查路径安全性
        if not self.is_navigating or self.current_path is None:
            return
        
        # 检查当前路径的安全性
        is_safe, error_msg = self.target_selector.check_path_safety(
            self.current_path, self.map, self.origin, self.resolution
        )
        
        if not is_safe:
            self.get_logger().warn(f'地图更新后路径安全检查失败：{error_msg}')
            self.get_logger().warn('当前路径不再安全，立即停止导航并重新规划')
            
            # 立即停止导航
            self.is_navigating = False
            self.current_path = None
            self.global_exploration_goal = None
            self.use_dwa_mode = False
            self.reset_local_target_state()
            self.reset_pid_controllers()
            self.clear_goal_marker()
            self.clear_path()
            self.publish_stop()
            
            # 记录失败的目标点，避免重复选择
            if self.global_exploration_goal is not None:
                goal_x, goal_y = self.global_exploration_goal
                goal_key = (round(goal_x, 2), round(goal_y, 2))
                self.failed_goals.add(goal_key)

    # ==================== 工具和调试方法 ====================
    
    def publish_goal_marker(self, x, y):
        """发布目标点标记"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.goal_marker_pub.publish(marker)

    def clear_goal_marker(self):
        """清除目标点标记"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.action = Marker.DELETE
        self.goal_marker_pub.publish(marker)

    def publish_local_target_marker(self, x, y):
        """发布局部目标点标记"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 1  # 使用不同的ID区分全局目标点和局部目标点
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.3  # 比全局目标点小一些
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 0.0  # 蓝色表示局部目标点
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.8  # 稍微透明
        self.local_target_marker_pub.publish(marker)

    def publish_local_target_marker_callback(self):
        """定时发布局部目标点标记，确保话题一直存在"""
        if self.current_local_target is not None:
            # 有目标点时，发布正常标记
            self.publish_local_target_marker(self.current_local_target[0], self.current_local_target[1])
        else:
            # 没有目标点时，发布空标记（让话题存在但不可见）
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = 1
            marker.type = Marker.SPHERE
            marker.action = Marker.DELETE  # 使用DELETE让标记不可见
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.0  # 完全透明
            self.local_target_marker_pub.publish(marker)

    def clear_local_target_marker(self):
        """清除局部目标点标记"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 1
        marker.action = Marker.DELETE
        self.local_target_marker_pub.publish(marker)

    def publish_safety_map(self, safety_map):
        """
        发布安全地图到/safety_map话题供RViz可视化
        
        Args:
            safety_map: 安全地图数组，True表示安全，False表示不安全
        """
        if safety_map is None or self.origin is None:
            return
            
        # 创建OccupancyGrid消息
        safety_grid_msg = OccupancyGrid()
        safety_grid_msg.header.frame_id = 'map'
        safety_grid_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 设置地图信息
        safety_grid_msg.info.resolution = self.resolution
        safety_grid_msg.info.width = safety_map.shape[1]
        safety_grid_msg.info.height = safety_map.shape[0]
        
        # 设置原点
        safety_grid_msg.info.origin.position.x = self.origin[0]
        safety_grid_msg.info.origin.position.y = self.origin[1]
        safety_grid_msg.info.origin.position.z = 0.0
        safety_grid_msg.info.origin.orientation.w = 1.0
        
        # 转换安全地图数据格式
        # True (安全) -> 0 (空闲)
        # False (不安全) -> 100 (障碍物)
        safety_data = []
        for y in range(safety_map.shape[0]):
            for x in range(safety_map.shape[1]):
                if safety_map[y][x]:
                    safety_data.append(0)    # 安全区域显示为空闲
                else:
                    safety_data.append(100)  # 不安全区域显示为障碍物
        
        safety_grid_msg.data = safety_data
        
        # 发布安全地图
        self.safety_map_pub.publish(safety_grid_msg)

    def publish_path(self, path):
        """发布路径"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for point in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)

    def clear_path(self):
        """清除路径显示"""
        empty_path = Path()
        empty_path.header.frame_id = "map"
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty_path)



def main(args=None):
    rclpy.init(args=args)
    node = LidarExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
