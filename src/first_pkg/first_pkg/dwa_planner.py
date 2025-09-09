#!/usr/bin/env python3
"""
DWA (Dynamic Window Approach) 局部规划器

本文件实现了一个基于动态窗口方法的局部路径规划器，用于机器人的实时避障和局部路径优化。

主要特性：
1. 动态窗口计算：根据机器人当前速度和加速度限制计算可行的速度搜索空间
2. 轨迹评估：评估不同速度组合的轨迹质量，包括距离目标、速度、安全性等
3. 实时避障：基于激光雷达数据进行实时障碍物检测和避障
4. 平滑控制：生成平滑的速度命令，避免急转弯和急停

作者：dong
版本：1.0
"""

import numpy as np
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import transforms3d


class DWAPlanner:
    def __init__(self):
        """初始化DWA规划器
        
        基于三个核心评分公式的简化DWA算法：
        1. direction(v,w) = π - |目标角度 - 当前角度|
        2. dist(v,w) = 预测轨迹离最近障碍物的距离
        3. velocity(v,w) = |v|
        """
        # 机器人物理参数
        self.max_linear_speed = 0.5     # 最大线速度 (m/s) - 提高以增强速度评分效果
        self.min_linear_speed = 0.1     # 最小线速度 (允许完全停止)
        self.max_angular_speed = 3.0    # 最大角速度 (rad/s) - 提高转弯能力
        self.min_angular_speed = -3.0   # 最小角速度 (rad/s)
        self.max_linear_accel = 0.75     # 最大线加速度 (m/s²) - 提高响应性
        self.max_angular_accel = 4.0    # 最大角加速度 (rad/s²) - 提高转向响应

        # 控制参数
        self.control_interval = 0.1     # 控制周期 (s)
        self.predict_time = 1.0         # 轨迹预测时间 (s) - 适当增加预测距离

        # 搜索分辨率参数 - 优化大角度调头
        self.velocity_samples = 8        # 线速度采样数量 - 减少以节省计算
        self.angular_samples = 15       # 角速度采样数量 - 增加以提高大角度调头精度

        # 权重参数 - 平衡方向跟踪和安全性
        self.weight_direction = 0.50    # 方向评分权重 - 提高以改善目标跟踪
        self.weight_obstacle = 0.35     # 障碍物距离评分权重 - 降低以避免过度避障
        self.weight_velocity = 0.15     # 速度评分权重 - 保持较低

        # 当前状态
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.current_pose = None

        # 地图信息
        self.map = None
        self.map_resolution = 0.05
        self.map_origin = (0.0, 0.0)

        # 目标点
        self.target_x = 0.0
        self.target_y = 0.0

        # 调试参数
        self.debug_enabled = True       # 开启调试以便观察行为

    # ============ 工具函数 ============

    def quaternion_to_yaw(self, q):
        """将四元数转换为 yaw 角"""
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))
        return yaw

    def get_robot_current_angle(self):
        """获取机器人当前角度 (yaw) - 考虑角速度影响"""
        if self.current_pose is None:
            return 0.0
        
        # 基础角度：从里程计获取的当前朝向
        base_angle = self.quaternion_to_yaw(self.current_pose.orientation)
        
        # 考虑角速度的影响：当前角度 = 基础角度 + 角速度 × 预测总时间
        # 这反映了机器人在整个预测时间内的朝向变化趋势
        current_angle = base_angle + self.current_angular_speed * self.predict_time      
        
        return current_angle

    def normalize_angle(self, angle):
        """将角度归一化到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def world_to_map(self, x, y):
        """将世界坐标转换为地图坐标"""
        map_x = int((x - self.map_origin[0]) / self.map_resolution)
        map_y = int((y - self.map_origin[1]) / self.map_resolution)
        return map_x, map_y

    def map_to_world(self, map_x, map_y):
        """将地图坐标转换为世界坐标"""
        world_x = self.map_origin[0] + map_x * self.map_resolution
        world_y = self.map_origin[1] + map_y * self.map_resolution
        return world_x, world_y

    # ============ 状态更新 ============

    def set_target(self, x, y):
        """设置目标点"""
        self.target_x = x
        self.target_y = y
        if self.debug_enabled:
            print(f"DWA目标点设置为: ({x:.2f}, {y:.2f})")

    def update_robot_state(self, odom_msg):
        """更新机器人状态"""
        self.current_pose = odom_msg.pose.pose
        self.current_linear_speed = odom_msg.twist.twist.linear.x
        self.current_angular_speed = odom_msg.twist.twist.angular.z
        if self.debug_enabled:
            print(f"机器人状态更新: 位置({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}), "
                  f"线速度={self.current_linear_speed:.2f}m/s, 角速度={self.current_angular_speed:.2f}rad/s")

    def update_map(self, map_array, resolution, origin):
        """更新地图信息"""
        self.map = map_array
        self.map_resolution = resolution
        self.map_origin = origin

    # ============ 动态窗口计算 ============

    def compute_dynamic_window(self):
        """计算动态窗口"""
        v0 = self.current_linear_speed
        w0 = self.current_angular_speed

        # 基础动态窗口
        v_min = max(self.min_linear_speed, v0 - self.max_linear_accel * self.control_interval)
        v_max = min(self.max_linear_speed, v0 + self.max_linear_accel * self.control_interval)
        w_min = max(self.min_angular_speed, w0 - self.max_angular_accel * self.control_interval)
        w_max = min(self.max_angular_speed, w0 + self.max_angular_accel * self.control_interval)

        return v_min, v_max, w_min, w_max

    # ============ 轨迹预测 ============

    def predict_trajectory(self, v, w, predict_time):
        """预测轨迹"""
        if self.current_pose is None:
            return []

        trajectory = []
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        theta = self.get_robot_current_angle()

        dt = self.control_interval
        t = 0.0
        while t <= predict_time:
            trajectory.append((x, y))
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += w * dt
            t += dt
        return trajectory

    # ============ 轨迹评价 ============

    def calculate_direction_score(self, trajectory, v, w):
        """方向角评价: 使用标准DWA公式
        
        公式: score = 1 - |delta_theta| / pi
        其中:
        - delta_theta = theta_g - theta_r
        - theta_g = arctan2(yg - yr, xg - xr)  # 目标角度
        - theta_r = theta0 + w * t  # 机器人朝向角度
        - (xr, yr): 机器人预测轨迹末端点位置
        - (xg, yg): 全局路径上的目标点
        - theta0: 机器人当前位置朝向
        - w: 轨迹角速度
        - t: 轨迹预测时间
        """
        if not trajectory or self.current_pose is None:
            return 0.0

        # 获取机器人当前位置和朝向
        xr, yr = self.current_pose.position.x, self.current_pose.position.y
        theta0 = self.quaternion_to_yaw(self.current_pose.orientation) + math.pi
        
        # 计算轨迹末端点位置 (xr, yr) - 使用轨迹的最后一个点
        if len(trajectory) > 0:
            xr, yr = trajectory[-1]  # 轨迹末端点
        
        # 计算目标角度 theta_g = arctan2(yg - yr, xg - xr)
        theta_g = math.atan2(self.target_y - yr, self.target_x - xr)
        
        # 计算机器人朝向角度 theta_r = theta0 + w * t
        theta_r = theta0 + w * self.predict_time
        
        # 计算角度差 delta_theta = theta_g - theta_r
        delta_theta = theta_g - theta_r
        delta_theta = self.normalize_angle(delta_theta)  # 归一化到[-pi, pi]
        
        # 计算方向评分: score = 1 - |delta_theta| / pi
        direction_score = 1.0 - abs(delta_theta) / math.pi
        
        return direction_score

    def calculate_obstacle_score(self, trajectory):
        """障碍物距离评价: 使用标准DWA公式计算
        
        公式:
        score(d) = 0                    if d <= d_safe
        score(d) = (d - d_safe)/(d_max - d_safe)  if d_safe < d < d_max  
        score(d) = 1                    if d >= d_max
        """
        if not trajectory or self.map is None:
            return 0.0  # 没有轨迹或地图数据时给最高分

        # 计算轨迹上每个点到障碍物的最小距离
        min_distance = float('inf')
        found_obstacle = False
        
        for x, y in trajectory:
            d = self.get_distance_to_obstacle_from_map(x, y)
            if d is not None:  # 找到了障碍物
                found_obstacle = True
                if d < min_distance:
                    min_distance = d

        # 如果整个轨迹路径上都没有找到障碍物，给最高分
        if not found_obstacle:
            return 1.0  # 没有障碍物时给最高分

        # 使用标准DWA公式计算评分
        d_safe = 0.20  # 安全距离阈值（降低阈值，允许更接近障碍物）
        d_max = 2.0   # 最大有效距离
        
        if min_distance <= d_safe:
            return 0.0  # 距离太近，评分为0
        elif min_distance >= d_max:
            return 1.0  # 距离足够远，评分为1
        else:
            # 线性插值计算评分
            score = (min_distance - d_safe) / (d_max - d_safe)
            return score


    def get_distance_to_obstacle_from_map(self, x, y):
        """使用地图信息计算预测点到障碍物的距离
        
        这个方法使用地图信息而不是激光雷达数据来计算距离
        更准确，因为直接使用地图的障碍物信息
        
        Args:
            x, y: 预测轨迹点的世界坐标
            
        Returns:
            float: 到最近障碍物的距离(米)，如果没有有效数据返回None
        """
        if self.map is None:
            if self.debug_enabled:
                print("⚠️ DWA地图数据为空！")
            return None

        # 将世界坐标转换为地图坐标
        map_x, map_y = self.world_to_map(x, y)
        
        # 检查是否在地图范围内
        if not (0 <= map_x < self.map.shape[1] and 0 <= map_y < self.map.shape[0]):
            return None

        # 使用距离变换或搜索算法找到最近的障碍物
        # 这里使用简单的搜索方法
        search_radius = int(1 / self.map_resolution)  # 搜索半径1米
        min_distance = float('inf')
        obstacle_count = 0
        
        for dy in range(-search_radius, search_radius + 1):
            for dx in range(-search_radius, search_radius + 1):
                # 检查是否在圆形搜索范围内
                if dx*dx + dy*dy <= search_radius*search_radius:
                    check_x = map_x + dx
                    check_y = map_y + dy
                    
                    # 检查是否在地图范围内
                    if (0 <= check_x < self.map.shape[1] and 
                        0 <= check_y < self.map.shape[0]):
                        
                        # 检查是否为障碍物
                        if self.map[check_y, check_x] == 100:  # 100表示障碍物
                            obstacle_count += 1
                            # 计算距离
                            distance = math.sqrt(dx*dx + dy*dy) * self.map_resolution
                            if distance < min_distance:
                                min_distance = distance
        
        # 如果搜索范围内没有障碍物，返回None
        if min_distance == float('inf'):
            return None  # 没有找到障碍物
        
        return min_distance

    def calculate_speed_score(self, v):
        """速度评价: 使用 velocity(v,w) = |v| 公式计算并归一化"""
        # 使用指定的公式: velocity(v,w) = |v|
        velocity_score = abs(v)
        
        # 直接进行归一化，除以最大线速度
        normalized_velocity_score = velocity_score / self.max_linear_speed
             
        return normalized_velocity_score


    # ============ 搜索最佳速度组合 ============

    def find_best_velocity_angle_combination(self):
        """寻找最佳速度角度组合 (v, w) - 使用跨轨迹归一化"""
        if self.current_pose is None:
            return 0.0, 0.0

        # 注意：移除了紧急停车检查，安全性通过障碍物距离评分处理

        v_min, v_max, w_min, w_max = self.compute_dynamic_window()
        
        # 使用可配置的搜索分辨率参数
        v_samples = np.linspace(v_min, v_max, self.velocity_samples)
        w_samples = np.linspace(w_min, w_max, self.angular_samples)

        # 第一步：计算所有轨迹的原始评分
        trajectories_data = []
        direction_scores = []
        obstacle_scores = []
        velocity_scores = []

        for v in v_samples:
            for w in w_samples:
                traj = self.predict_trajectory(v, w, self.predict_time)
                
                # 计算原始评分
                direction_score = self.calculate_direction_score(traj, v, w)
                obstacle_score = self.calculate_obstacle_score(traj)
                velocity_score = self.calculate_speed_score(v)
                
                # 应用特定的归一化规则
                # 1. 方向评分：已经标准化到[0,1]范围，无需额外处理
                # 2. 速度评分：已经在calculate_speed_score中归一化，无需额外处理
                # 3. 障碍物评分：已经标准化到[0,1]范围，无需额外处理
                
                trajectories_data.append((v, w, traj))
                direction_scores.append(direction_score)
                obstacle_scores.append(obstacle_score)
                velocity_scores.append(velocity_score)

        # 第二步：直接使用已归一化的评分计算最终得分
        best_score = float('-inf')
        best_v, best_w = 0.0, 0.0
        valid_trajectories = 0

        for i, (v, w, traj) in enumerate(trajectories_data):

            # 直接使用已归一化的评分（每个评分都已经在[0,1]范围内）
            # 计算最终得分：三个已归一化评分加权相加
            total_score = (self.weight_direction * direction_scores[i] + 
                          self.weight_obstacle * obstacle_scores[i] + 
                          self.weight_velocity * velocity_scores[i])
            
            if total_score > 0:  # 有效轨迹
                valid_trajectories += 1
                
            if total_score > best_score:
                best_score, best_v, best_w = total_score, v, w

        return best_v, best_w



    def generate_velocity_command(self):
        """生成速度命令"""
        v, w = self.find_best_velocity_angle_combination()
        cmd_vel = Twist()
        cmd_vel.linear.x = v
        cmd_vel.angular.z = w
             
        return cmd_vel