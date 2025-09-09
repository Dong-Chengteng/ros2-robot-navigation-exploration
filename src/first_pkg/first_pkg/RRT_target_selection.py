#!/usr/bin/env python3
"""
基于RRT树扩张的探索点选择算法

本模块实现了一个基于RRT（Rapidly-exploring Random Tree）的探索点选择算法，
在安全地图的可行走区域随机采样，利用激光雷达计算信息增益，
根据效用函数选择最优的探索点。

主要特性：
1. RRT树扩张：在安全地图可行走区域随机采样
2. 信息增益计算：基于激光雷达数据计算探索价值
3. 效用函数：综合考虑距离、信息增益和探索效率
4. 安全约束：确保采样点不在安全地图的膨胀范围内

作者：dong
版本：1.0
"""

import math
import random
import numpy as np
from typing import List, Tuple, Optional, Callable
import heapq

Point = Tuple[float, float]


class RRTExplorationSelector:
    """基于RRT的探索点选择器"""
    
    def __init__(self, logger=None):
        """
        初始化RRT探索选择器
        
        Args:
            logger: 日志记录器，可选
        """
        self.logger = logger
        
        # RRT参数
        self.max_iterations = 1000  # 最大迭代次数
        self.step_size = 0.5  # 步长（米）
        self.goal_sample_rate = 0.1  # 目标采样率（10%概率采样目标点）
        self.min_distance_threshold = 0.1  # 最小距离阈值
        
        # 信息增益计算参数
        self.laser_range = 8.0  # 激光雷达最大范围（米）
        self.laser_angle_increment = 0.01  # 激光雷达角度增量（弧度）
        self.information_radius = 2.0  # 信息增益计算半径（米）
        
        # 效用函数权重
        self.distance_weight = 0.1  # 距离权重（调低）
        self.information_weight = 0.5  # 信息增益权重
        self.exploration_weight = 0.4  # 探索效率权重（相应调高）
        
        # 安全参数
        self.safety_margin = 0.3  # 安全边距（米）
        self.min_safe_distance = 0.5  # 最小安全距离（米）
    
    def set_parameters(self, max_iterations=1000, step_size=0.5, goal_sample_rate=0.1,
                      laser_range=8.0, information_radius=2.0, 
                      distance_weight=0.1, information_weight=0.5, exploration_weight=0.4):
        """
        设置RRT探索参数
        
        Args:
            max_iterations: 最大迭代次数
            step_size: 步长（米）
            goal_sample_rate: 目标采样率
            laser_range: 激光雷达最大范围（米）
            information_radius: 信息增益计算半径（米）
            distance_weight: 距离权重
            information_weight: 信息增益权重
            exploration_weight: 探索效率权重
        """
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.laser_range = laser_range
        self.information_radius = information_radius
        self.distance_weight = distance_weight
        self.information_weight = information_weight
        self.exploration_weight = exploration_weight
        
    
    
    
    
    def get_random_point(self, map_bounds: Tuple[float, float, float, float]) -> Point:
        """
        在地图边界内生成随机点
        
        Args:
            map_bounds: 地图边界 (min_x, min_y, max_x, max_y)
            
        Returns:
            随机点坐标 (x, y)
        """
        min_x, min_y, max_x, max_y = map_bounds
        x = random.uniform(min_x, max_x)
        y = random.uniform(min_y, max_y)
        return (x, y)
    
    def get_nearest_node(self, point: Point, tree: List[Point]) -> Point:
        """
        在树中找到距离给定点最近的节点
        
        Args:
            point: 目标点
            tree: RRT树节点列表
            
        Returns:
            最近的节点
        """
        if not tree:
            return None
        
        min_distance = float('inf')
        nearest_node = None
        
        for node in tree:
            distance = math.sqrt((point[0] - node[0])**2 + (point[1] - node[1])**2)
            if distance < min_distance:
                min_distance = distance
                nearest_node = node
        
        return nearest_node
    
    def steer(self, from_point: Point, to_point: Point) -> Point:
        """
        从from_point向to_point方向扩展一步
        
        Args:
            from_point: 起始点
            to_point: 目标点
            
        Returns:
            新点坐标
        """
        distance = math.sqrt((to_point[0] - from_point[0])**2 + (to_point[1] - from_point[1])**2)
        
        if distance <= self.step_size:
            return to_point
        
        # 计算单位方向向量
        dx = (to_point[0] - from_point[0]) / distance
        dy = (to_point[1] - from_point[1]) / distance
        
        # 扩展一步
        new_x = from_point[0] + dx * self.step_size
        new_y = from_point[1] + dy * self.step_size
        
        return (new_x, new_y)
    
    def find_frontier_points_in_map(self, map_array, origin: Tuple[float, float], resolution: float) -> List[Point]:
        """
        在/map地图中查找已知区域和未知区域的交界点（边界点）
        边界点必须满足：1）周围有未知区域 2）1米半径范围内没有障碍物
        
        Args:
            map_array: 地图数组
            origin: 地图原点
            resolution: 地图分辨率
            
        Returns:
            边界点列表（已知区域中靠近未知区域的点）
        """
        frontier_points = []
        height, width = map_array.shape
        
        for y in range(height):
            for x in range(width):
                # 只考虑已知空闲区域（值为0）
                if map_array[y][x] == 0:
                    # 检查周围8个邻居是否有未知区域
                    has_unknown_neighbor = False
                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            if dx == 0 and dy == 0:
                                continue
                            neighbor_x = x + dx
                            neighbor_y = y + dy
                            
                            # 检查邻居是否在地图范围内且为未知区域
                            if (0 <= neighbor_x < width and 0 <= neighbor_y < height and 
                                map_array[neighbor_y][neighbor_x] == -1):
                                has_unknown_neighbor = True
                                break
                        if has_unknown_neighbor:
                            break
                    
                    # 如果周围有未知区域，检查1米半径范围内是否有障碍物
                    if has_unknown_neighbor:
                        # 检查1米半径范围内是否有障碍物
                        if self.is_point_safe_from_obstacles(x, y, map_array, origin, resolution, safety_radius=1.0):
                            world_x, world_y = self.map_to_world(x, y, origin, resolution)
                            frontier_points.append((world_x, world_y))
        
        return frontier_points
    
    def is_point_safe_from_obstacles(self, map_x: int, map_y: int, map_array, 
                                   origin: Tuple[float, float], resolution: float, 
                                   safety_radius: float = 1.0) -> bool:
        """
        检查点1米半径范围内是否有障碍物（数值为100的点）
        
        Args:
            map_x, map_y: 地图坐标
            map_array: 地图数组
            origin: 地图原点
            resolution: 地图分辨率
            safety_radius: 安全半径（米），默认1.0米
            
        Returns:
            是否安全（1米半径范围内没有障碍物）
        """
        if map_array is None:
            return True
        
        # 检查边界
        if not (0 <= map_x < map_array.shape[1] and 0 <= map_y < map_array.shape[0]):
            return False
        
        # 计算安全半径对应的像素数
        safety_radius_pixels = int(safety_radius / resolution)
        
        # 在1米半径范围内检查是否有障碍物
        for dy in range(-safety_radius_pixels, safety_radius_pixels + 1):
            for dx in range(-safety_radius_pixels, safety_radius_pixels + 1):
                check_x = map_x + dx
                check_y = map_y + dy
                
                # 检查是否在地图范围内
                if (0 <= check_x < map_array.shape[1] and 0 <= check_y < map_array.shape[0]):
                    # 计算实际距离
                    distance = math.sqrt(dx*dx + dy*dy) * resolution
                    if distance <= safety_radius:
                        # 如果在这个距离内有障碍物（值为100），则不安全
                        if map_array[check_y][check_x] == 100:
                            return False
        
        return True
    
    def evaluate_all_frontier_points(self, robot_position: Point, map_array, origin: Tuple[float, float], 
                                   resolution: float) -> List[Tuple[Point, float]]:
        """
        评估所有边界点，计算每个边界点的综合评分
        采用分批处理机制，确保能够遍历所有边界点直到找到合适的探索点
        
        Args:
            robot_position: 机器人当前位置
            map_array: 地图数组
            origin: 地图原点
            resolution: 地图分辨率
            
        Returns:
            边界点列表，每个元素为(点坐标, 综合评分)
        """
        if map_array is None:
            return []
        
        # 首先找到所有边界点（已知区域和未知区域的交界处）
        frontier_points = self.find_frontier_points_in_map(map_array, origin, resolution)
        
        if not frontier_points:
            return []
        
        # 分批处理参数
        batch_size = 200
        min_valid_points = 5
        max_batches = 15
        
        evaluated_points = []
        total_filtered_count = 0
        batch_count = 0
        
        # 分批处理所有边界点
        for batch_start in range(0, len(frontier_points), batch_size):
            batch_count += 1
            batch_end = min(batch_start + batch_size, len(frontier_points))
            batch_points = frontier_points[batch_start:batch_end]
            
            
            batch_evaluated_points = []
            batch_filtered_count = 0
            
            # 处理当前批次
            batch_evaluation_count = 0
            for point in batch_points:
                # 检查距离是否满足最小要求
                distance = math.sqrt((point[0] - robot_position[0])**2 + (point[1] - robot_position[1])**2)
                if distance < 1.0:
                    batch_filtered_count += 1
                    continue
                
                
                # 计算评分
                score = self.calculate_frontier_point_score(point, robot_position, map_array, origin, resolution)
                if score > 0.0:
                    batch_evaluated_points.append((point, score))
                
                batch_evaluation_count += 1
            
            # 将当前批次的有效点添加到总列表
            evaluated_points.extend(batch_evaluated_points)
            total_filtered_count += batch_filtered_count
            
            
            # 如果已经找到足够多的有效点，可以提前结束
            if len(evaluated_points) >= min_valid_points:
                break
            
            # 防止处理过多批次
            if batch_count >= max_batches:
                break
        
        
        
        return evaluated_points
    
    def calculate_frontier_point_score(self, point: Point, robot_position: Point, map_array, 
                                     origin: Tuple[float, float], resolution: float) -> float:
        """
        计算边界点的综合评分：探索面积得分 + 距离得分（通过RRT树计算路径距离）
        
        Args:
            point: 边界点
            robot_position: 机器人当前位置
            map_array: 地图数组
            origin: 地图原点
            resolution: 地图分辨率
            
        Returns:
            综合评分
        """
        # 探索面积得分
        exploration_score = self.calculate_exploration_area_normalized_score(point, map_array, origin, resolution)
        
        # 距离得分
        path_distance = self.calculate_rrt_path_distance(robot_position, point, map_array, origin, resolution)
        distance_score = math.exp(-path_distance)
        
        # 综合评分
        exploration_weight = 0.8
        distance_weight = 0.2
        
        comprehensive_score = exploration_weight * exploration_score + distance_weight * distance_score
        
        return comprehensive_score
    
    def calculate_exploration_area_normalized_score(self, point: Point, map_array, origin: Tuple[float, float], 
                                                  resolution: float) -> float:
        """
        计算探索面积归一化得分：8m范围内未知点个数 / 8m范围内全是未知点的个数（8m是激光类所能探索最远的距离）
        
        Args:
            point: 目标点
            map_array: 地图数组
            origin: 地图原点
            resolution: 地图分辨率
            
        Returns:
            归一化的探索面积得分（0-1）
        """
        if map_array is None:
            return 0.0
        
        # 转换为地图坐标
        map_x, map_y = self.world_to_map(point[0], point[1], origin, resolution)
        
        # 检查边界
        if not (0 <= map_x < map_array.shape[1] and 0 <= map_y < map_array.shape[0]):
            return 0.0
        
        # 计算8m范围对应的像素数
        exploration_radius = 8.0
        exploration_radius_pixels = int(exploration_radius / resolution)
        
        unknown_count = 0
        total_count = 0
        
        # 在8m范围内搜索未知区域
        for dy in range(-exploration_radius_pixels, exploration_radius_pixels + 1):
            for dx in range(-exploration_radius_pixels, exploration_radius_pixels + 1):
                check_x = map_x + dx
                check_y = map_y + dy
                
                # 检查是否在地图范围内
                if (0 <= check_x < map_array.shape[1] and 0 <= check_y < map_array.shape[0]):
                    # 检查是否在8m圆形半径内
                    distance = math.sqrt(dx*dx + dy*dy) * resolution
                    if distance <= exploration_radius:
                        total_count += 1
                        if map_array[check_y][check_x] == -1:
                            unknown_count += 1
        
        # 归一化
        if total_count > 0:
            normalized_score = unknown_count / total_count
            return normalized_score
        
        return 0.0
    
    def calculate_rrt_path_distance(self, start: Point, goal: Point, map_array, 
                                  origin: Tuple[float, float], resolution: float) -> float:
        """
        通过RRT树计算从起点到终点的路径距离
        
        Args:
            start: 起点坐标
            goal: 终点坐标
            map_array: 地图数组
            origin: 地图原点
            resolution: 地图分辨率
            
        Returns:
            路径距离（米），如果无法到达则返回一个很大的值
        """
        if map_array is None:
            return float('inf')
        
        # 计算地图边界
        height, width = map_array.shape
        min_x = origin[0]
        min_y = origin[1]
        max_x = origin[0] + width * resolution
        max_y = origin[1] + height * resolution
        
        map_bounds = (min_x, min_y, max_x, max_y)
        
        # 初始化RRT树
        tree = [start]
        max_iterations = 100
        step_size = 1.0
        
        # RRT主循环
        for iteration in range(max_iterations):
            # 生成随机点
            if random.random() < 0.1:
                random_point = goal
            else:
                random_point = self.get_random_point(map_bounds)
            
            # 找到最近的节点
            nearest_node = self.get_nearest_node(random_point, tree)
            if nearest_node is None:
                continue
            
            # 扩展树
            new_point = self.steer(nearest_node, random_point)
            
            # 检查新点是否在地图范围内且为已知空闲区域
            if not self.is_point_in_free_space(new_point, map_array, origin, resolution):
                continue
            
            # 检查路径是否安全
            if not self.is_straight_path_clear(nearest_node, new_point, map_array, origin, resolution):
                continue
            
            # 将新点添加到树中
            tree.append(new_point)
            
            # 检查是否到达目标点
            distance_to_goal = math.sqrt((new_point[0] - goal[0])**2 + (new_point[1] - goal[1])**2)
            if distance_to_goal <= step_size:
                total_distance = self.calculate_tree_path_distance(tree, start, new_point)
                return total_distance
        
        return float('inf')
    
    def is_point_in_free_space(self, point: Point, map_array, origin: Tuple[float, float], resolution: float) -> bool:
        """
        检查点是否在已知空闲区域
        
        Args:
            point: 目标点
            map_array: 地图数组
            origin: 地图原点
            resolution: 地图分辨率
            
        Returns:
            是否在空闲区域
        """
        map_x, map_y = self.world_to_map(point[0], point[1], origin, resolution)
        
        # 检查边界
        if not (0 <= map_x < map_array.shape[1] and 0 <= map_y < map_array.shape[0]):
            return False
        
        return map_array[map_y][map_x] == 0
    
    def is_straight_path_clear(self, start: Point, end: Point, map_array, 
                             origin: Tuple[float, float], resolution: float) -> bool:
        """
        检查两点之间的直线路径是否安全（简单版本，只检查障碍物）
        
        Args:
            start: 起始点
            end: 终点
            map_array: 地图数组
            origin: 地图原点
            resolution: 地图分辨率
            
        Returns:
            路径是否安全
        """
        # 计算路径上的采样点数
        distance = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        num_samples = int(distance / resolution) + 1
        
        for i in range(num_samples + 1):
            t = i / num_samples if num_samples > 0 else 0
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            
            # 检查路径上的每个点是否在空闲区域
            if not self.is_point_in_free_space((x, y), map_array, origin, resolution):
                return False
        
        return True
    
    def calculate_tree_path_distance(self, tree: List[Point], start: Point, goal: Point) -> float:
        """
        计算RRT树中从起点到终点的路径距离
        
        Args:
            tree: RRT树节点列表
            start: 起点
            goal: 终点
            
        Returns:
            路径总距离
        """
        if not tree or start not in tree or goal not in tree:
            return float('inf')
        
        # 简单的路径距离计算
        total_distance = 0.0
        current_point = start
        
        # 找到从起点到终点的路径
        while current_point != goal:
            # 找到距离当前点最近的下一个点
            min_distance = float('inf')
            next_point = None
            
            for point in tree:
                if point != current_point:
                    distance = math.sqrt((point[0] - current_point[0])**2 + (point[1] - current_point[1])**2)
                    if distance < min_distance:
                        min_distance = distance
                        next_point = point
            
            if next_point is None:
                break
            
            total_distance += min_distance
            current_point = next_point
            
            # 防止无限循环
            if total_distance > 1000:
                break
        
        return total_distance
    
    def select_best_exploration_point(self, robot_position: Point, map_array, safety_map,
                                    origin: Tuple[float, float], resolution: float) -> Optional[Point]:
        """
        选择最优的探索点（评估所有边界点，选择得分最高的）
        当所有边界点都无效时，说明地图探索已完成
        
        Args:
            robot_position: 机器人当前位置
            map_array: 地图数组
            safety_map: 安全地图
            origin: 地图原点
            resolution: 地图分辨率
            
        Returns:
            最优探索点坐标，如果没有找到则返回None（表示探索完成）
        """
        # 评估所有边界点，计算每个边界点的综合评分
        candidate_points = self.evaluate_all_frontier_points(robot_position, map_array, origin, resolution)
        
        if not candidate_points:
            return None
        
        # 按综合评分排序，选择最优的点
        candidate_points.sort(key=lambda x: x[1], reverse=True)
        best_point, best_score = candidate_points[0]
        
        
        return best_point
    
    
    def get_exploration_statistics(self, robot_position: Point, map_array, safety_map,
                                 origin: Tuple[float, float], resolution: float) -> dict:
        """
        获取探索统计信息
        
        Args:
            robot_position: 机器人当前位置
            map_array: 地图数组
            safety_map: 安全地图
            origin: 地图原点
            resolution: 地图分辨率
            
        Returns:
            统计信息字典
        """
        if map_array is None or safety_map is None:
            return {}
        
        # 计算地图统计信息
        total_pixels = map_array.size
        unknown_pixels = np.sum(map_array == -1)
        free_pixels = np.sum(map_array == 0)
        occupied_pixels = np.sum(map_array == 100)
        
        # 计算安全区域统计
        safe_pixels = np.sum(safety_map) if safety_map is not None else 0
        
        # 计算探索进度
        exploration_progress = (free_pixels + occupied_pixels) / total_pixels * 100
        
        stats = {
            'total_pixels': total_pixels,
            'unknown_pixels': unknown_pixels,
            'free_pixels': free_pixels,
            'occupied_pixels': occupied_pixels,
            'safe_pixels': safe_pixels,
            'exploration_progress': exploration_progress,
            'unknown_ratio': unknown_pixels / total_pixels * 100,
            'safe_ratio': safe_pixels / total_pixels * 100 if total_pixels > 0 else 0
        }
        
        return stats
