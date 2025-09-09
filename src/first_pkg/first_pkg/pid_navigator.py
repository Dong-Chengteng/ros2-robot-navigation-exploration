#!/usr/bin/env python3
"""
PID路径跟踪控制模块

基于PID控制器的路径跟踪导航器，提供更稳定和精确的导航控制。
包含角度PID控制器和路径偏移PID控制器。

作者：dong
版本：1.0 (PID版本)
"""

import math
import time
from typing import Dict, List, Tuple

import numpy as np
from geometry_msgs.msg import Twist

Point = Tuple[float, float]


class PIDController:
    """PID控制器类"""
    
    def __init__(self, kp: float, ki: float, kd: float, output_limit: float = float('inf')):
        self.kp = kp  # 比例增益
        self.ki = ki  # 积分增益
        self.kd = kd  # 微分增益
        self.output_limit = output_limit
        
        # 状态变量
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
        # 积分限幅
        self.integral_limit = output_limit / ki if ki > 0 else float('inf')
        
    def reset(self):
        """重置PID控制器状态"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
    def compute(self, error: float) -> float:
        """计算PID输出
        
        Args:
            error: 当前误差
            
        Returns:
            PID控制输出
        """
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            return 0.0
            
        # 积分项
        self.integral += error * dt
        
        # 积分限幅
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < -self.integral_limit:
            self.integral = -self.integral_limit
            
        # 微分项
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        
        # PID输出
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # 输出限幅
        if output > self.output_limit:
            output = self.output_limit
        elif output < -self.output_limit:
            output = -self.output_limit
            
        # 更新状态
        self.prev_error = error
        self.last_time = current_time
        
        return output


def compute_cmd(
    current_pose,                # geometry_msgs/Pose
    current_path: List[Point],   # [(x, y), ...]
    params: Dict[str, float],    # 各类参数
) -> Tuple[Twist, bool, bool, float]:
    """计算PID路径跟踪指令。

    返回: (cmd_vel, reached_goal, pop_waypoint, path_offset)
    """
    cmd = Twist()

    if current_pose is None or current_path is None or len(current_path) < 2:
        return cmd, False, False, 0.0

    # 当前位置
    cx = current_pose.position.x
    cy = current_pose.position.y

    # 目标点（下一个路径点）
    next_x, next_y = current_path[1]

    # 与目标点的几何关系
    dx = next_x - cx
    dy = next_y - cy
    distance = math.hypot(dx, dy)
    path_angle = math.atan2(dy, dx)

    # 终极目标距离
    goal_distance = None
    if params.get("goal_x") is not None and params.get("goal_y") is not None:
        gx = float(params["goal_x"])
        gy = float(params["goal_y"])
        goal_distance = math.hypot(gx - cx, gy - cy)

    # 当前朝向
    ori = current_pose.orientation
    # yaw from quaternion
    yaw = math.atan2(2.0 * (ori.w * ori.z + ori.x * ori.y), 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z))

    # 计算角度差：当前角度 - 目标角度
    angle_diff = yaw - path_angle
    
    # 角度归一化到 [-π, π]
    while angle_diff > math.pi:
        angle_diff -= 2.0 * math.pi
    while angle_diff < -math.pi:
        angle_diff += 2.0 * math.pi

    # 路径偏移（到当前段的垂直距离）
    path_offset = distance
    if len(current_path) >= 2:
        sx, sy = current_path[0]
        ex, ey = current_path[1]
        vx = ex - sx
        vy = ey - sy
        seg_len = math.hypot(vx, vy)
        if seg_len > 1e-3:
            a = vy
            b = -vx
            c = vx * sy - vy * sx
            path_offset = abs(a * cx + b * cy + c) / math.hypot(a, b)

    # 到达目标点判定
    reached_goal = False
    goal_dist_thresh = float(params.get("goal_distance_threshold", 0.2))
    if goal_distance is not None and goal_distance < goal_dist_thresh:
        reached_goal = True
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return cmd, True, False, path_offset

    # PID控制器参数
    angle_kp = float(params.get("angle_kp", 1.5))
    angle_ki = float(params.get("angle_ki", 0.1))
    angle_kd = float(params.get("angle_kd", 0.05))
    
    offset_kp = float(params.get("offset_kp", 0.8))
    offset_ki = float(params.get("offset_ki", 0.05))
    offset_kd = float(params.get("offset_kd", 0.02))
    
    # 速度控制参数
    min_linear = float(params.get("min_linear_speed", 0.0))
    max_linear = float(params.get("max_linear_speed", 2.0))
    max_angular = float(params.get("max_angular_speed", 2.0))
    
    # 控制阈值
    angle_threshold = float(params.get("angle_threshold", math.radians(10)))
    offset_threshold = float(params.get("offset_threshold", 0.3))
    
    # 路径点密度因子
    density_factor = 1.0 if len(current_path) >= 5 else 0.6

    # 检查机器人是否面向目标方向（角度差小于90度）
    facing_target = abs(angle_diff) < math.pi/2
    
    # 初始化PID控制器（如果不存在）
    if not hasattr(compute_cmd, 'angle_pid'):
        compute_cmd.angle_pid = PIDController(angle_kp, angle_ki, angle_kd, max_angular)
        compute_cmd.offset_pid = PIDController(offset_kp, offset_ki, offset_kd, max_angular * 0.5)
        # 添加状态跟踪变量
        compute_cmd.was_rotate_only = False
        compute_cmd.angle_converged = False
    
    # 更新PID参数
    compute_cmd.angle_pid.kp = angle_kp
    compute_cmd.angle_pid.ki = angle_ki
    compute_cmd.angle_pid.kd = angle_kd
    compute_cmd.angle_pid.output_limit = max_angular
    
    compute_cmd.offset_pid.kp = offset_kp
    compute_cmd.offset_pid.ki = offset_ki
    compute_cmd.offset_pid.kd = offset_kd
    compute_cmd.offset_pid.output_limit = max_angular * 0.5

    # 角度收敛检查：检查是否从rotate_only模式中恢复
    angle_convergence_threshold = math.radians(5)  # 5度收敛阈值
    if compute_cmd.was_rotate_only and abs(angle_diff) < angle_convergence_threshold:
        compute_cmd.angle_converged = True
    elif abs(angle_diff) > angle_threshold:
        compute_cmd.angle_converged = False

    # PID控制逻辑
    if abs(angle_diff) > angle_threshold:
        # 大角度差：优先旋转对齐
        cmd.angular.z = compute_cmd.angle_pid.compute(angle_diff)
        
        # 根据角度差大小决定是否给予线速度
        if abs(angle_diff) > math.radians(65):  # 65度以上，大角度差
            # 大角度差时，完全停止，只旋转
            cmd.linear.x = 0.0
            control_mode = "rotate_only"
            compute_cmd.was_rotate_only = True
        else:
            # 中等角度差时，检查是否刚从rotate_only模式恢复
            if compute_cmd.was_rotate_only and not compute_cmd.angle_converged:
                # 刚从rotate_only恢复，但角度还未充分收敛，继续只旋转
                cmd.linear.x = 0.0
                control_mode = "rotate_only"
            else:
                # 角度已收敛或非rotate_only模式，给予较小的线速度
                base_linear = float(np.clip(distance * 0.2, min_linear, max_linear))
                cmd.linear.x = base_linear * density_factor
                control_mode = "rotate_with_speed"
                compute_cmd.was_rotate_only = False
        
        # 重置路径偏移PID
        compute_cmd.offset_pid.reset()
        
    elif facing_target:
        # 小角度差且面向目标：允许前进，同时进行角度校正
        base_linear = float(np.clip(distance * 2, min_linear, max_linear))
        cmd.linear.x = base_linear * density_factor
        
        # 重置rotate_only状态，因为已经面向目标
        compute_cmd.was_rotate_only = False
        compute_cmd.angle_converged = False
        
        # 角度PID控制
        if abs(angle_diff) > math.radians(3):
            angle_correction = compute_cmd.angle_pid.compute(angle_diff)
            
            # 路径偏移PID控制
            if path_offset > offset_threshold:
                offset_correction = compute_cmd.offset_pid.compute(path_offset)
                # 根据偏移方向调整校正
                if angle_diff > 0:
                    offset_correction = abs(offset_correction)
                else:
                    offset_correction = -abs(offset_correction)
            else:
                offset_correction = 0.0
                compute_cmd.offset_pid.reset()
                
            cmd.angular.z = angle_correction + offset_correction
            control_mode = "track_with_pid"
        else:
            cmd.angular.z = 0.0
            control_mode = "straight"
            # 重置PID控制器
            compute_cmd.angle_pid.reset()
            compute_cmd.offset_pid.reset()
            
    else:
        # 小角度差但背向目标：只旋转，不前进
        cmd.angular.z = compute_cmd.angle_pid.compute(angle_diff)
        cmd.linear.x = 0.0
        control_mode = "rotate_backward"
        
        # 重置路径偏移PID
        compute_cmd.offset_pid.reset()

    # 到达当前段中间点则推进路径
    waypoint_thresh = float(params.get("waypoint_distance_threshold", 0.4))
    pop_waypoint = distance < waypoint_thresh and len(current_path) > 2

    # 过大偏移时的特殊处理
    if path_offset > 0.6:
        target_angle = math.atan2(dy, dx)
        angle_correction = target_angle - yaw
        if angle_correction > math.pi:
            angle_correction -= 2.0 * math.pi
        elif angle_correction < -math.pi:
            angle_correction += 2.0 * math.pi

        if abs(angle_correction) > angle_threshold:
            # 未对正，继续原地旋转
            cmd.linear.x = 0.0
            cmd.angular.z = compute_cmd.angle_pid.compute(angle_correction)
            control_mode = "offset_rotate"
        else:
            # 已基本对正，给适量前进速度来实际减小偏移
            base_linear = float(np.clip(distance * 2, min_linear, max_linear))
            cmd.linear.x = base_linear * 0.8
            cmd.angular.z = compute_cmd.angle_pid.compute(angle_correction) * 0.6
            control_mode = "offset_forward"

    # 调试输出
    if bool(params.get("debug_nav", True)):
        print(
            f"[PID] yaw={math.degrees(yaw):.1f}°, target={math.degrees(path_angle):.1f}°, diff={math.degrees(angle_diff):.1f}°, "
            f"facing={facing_target}, dist={distance:.2f}m, offset={path_offset:.2f}m, mode={control_mode}, "
            f"was_rotate_only={compute_cmd.was_rotate_only}, angle_converged={compute_cmd.angle_converged}, "
            f"v={cmd.linear.x:.2f}m/s, w={cmd.angular.z:.2f}rad/s",
            flush=True
        )

    return cmd, reached_goal, pop_waypoint, path_offset


def reset_pid_controllers():
    """重置所有PID控制器状态"""
    if hasattr(compute_cmd, 'angle_pid'):
        compute_cmd.angle_pid.reset()
    if hasattr(compute_cmd, 'offset_pid'):
        compute_cmd.offset_pid.reset()
    # 重置状态跟踪变量
    if hasattr(compute_cmd, 'was_rotate_only'):
        compute_cmd.was_rotate_only = False
    if hasattr(compute_cmd, 'angle_converged'):
        compute_cmd.angle_converged = False
