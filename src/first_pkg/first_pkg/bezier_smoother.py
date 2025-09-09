#!/usr/bin/env python3
"""
贝塞尔路径平滑模块

提供与原有实现等价的二次贝塞尔曲线平滑逻辑，作为独立工具函数供节点调用。
"""

import math
from typing import Callable, Iterable, List, Tuple, Optional

Point = Tuple[float, float]

def smooth_path(
    path: Iterable[Point],
    safety_checker: Callable[[float, float], bool],
    logger: Optional[object] = None,
    control_weight: float = 0.3,
    max_control_distance: float = 0.5,
    point_spacing: float = 0.2,
    min_spacing: float = 0.1,
) -> List[Point]:
    """使用二次贝塞尔曲线对路径进行平滑。

    参数:
    - path: 原始路径点序列 [(x, y), ...]
    - safety_checker: 可调用对象，用于判断点 (x, y) 是否安全
    - logger: 可选日志对象（需支持 info / warn），可为 None
    - control_weight: 控制点距离当前段长度的权重
    - max_control_distance: 控制点最大偏移距离（米）
    - point_spacing: 每段内生成的贝塞尔点的间距目标（米）
    - min_spacing: 输出路径点的最小间距（米）
    """

    path = list(path)
    if len(path) < 3:
        return path

    smoothed_points: List[Point] = []

    for i in range(len(path) - 1):
        current_point = path[i]
        next_point = path[i + 1]

        # 计算控制点
        if i == 0:
            # 使用下一个方向作为控制方向
            if len(path) > 2:
                direction_x = path[2][0] - current_point[0]
                direction_y = path[2][1] - current_point[1]
                length = math.hypot(direction_x, direction_y)
                if length > 0:
                    direction_x /= length
                    direction_y /= length
                else:
                    direction_x = next_point[0] - current_point[0]
                    direction_y = next_point[1] - current_point[1]
                    length = math.hypot(direction_x, direction_y)
                    if length > 0:
                        direction_x /= length
                        direction_y /= length
            else:
                direction_x = next_point[0] - current_point[0]
                direction_y = next_point[1] - current_point[1]
                length = math.hypot(direction_x, direction_y)
                if length > 0:
                    direction_x /= length
                    direction_y /= length

            segment_length = math.hypot(next_point[0] - current_point[0], next_point[1] - current_point[1])
            control_distance = min(segment_length * control_weight, max_control_distance)
            control_point = (
                current_point[0] + direction_x * control_distance,
                current_point[1] + direction_y * control_distance,
            )

        elif i == len(path) - 2:
            # 最后一段，用前一段方向
            direction_x = current_point[0] - path[i - 1][0]
            direction_y = current_point[1] - path[i - 1][1]
            length = math.hypot(direction_x, direction_y)
            if length > 0:
                direction_x /= length
                direction_y /= length
            else:
                direction_x = next_point[0] - current_point[0]
                direction_y = next_point[1] - current_point[1]
                length = math.hypot(direction_x, direction_y)
                if length > 0:
                    direction_x /= length
                    direction_y /= length

            segment_length = math.hypot(next_point[0] - current_point[0], next_point[1] - current_point[1])
            control_distance = min(segment_length * control_weight, max_control_distance)
            control_point = (
                next_point[0] - direction_x * control_distance,
                next_point[1] - direction_y * control_distance,
            )

        else:
            # 中间段，前后方向的平均
            prev_direction_x = current_point[0] - path[i - 1][0]
            prev_direction_y = current_point[1] - path[i - 1][1]
            prev_length = math.hypot(prev_direction_x, prev_direction_y)

            next_direction_x = path[i + 1][0] - next_point[0]
            next_direction_y = path[i + 1][1] - next_point[1]
            next_length = math.hypot(next_direction_x, next_direction_y)

            if prev_length > 0 and next_length > 0:
                avg_direction_x = (prev_direction_x / prev_length + next_direction_x / next_length) / 2.0
                avg_direction_y = (prev_direction_y / prev_length + next_direction_y / next_length) / 2.0
                avg_length = math.hypot(avg_direction_x, avg_direction_y)
                if avg_length > 0:
                    avg_direction_x /= avg_length
                    avg_direction_y /= avg_length
                else:
                    avg_direction_x = next_point[0] - current_point[0]
                    avg_direction_y = next_point[1] - current_point[1]
                    length = math.hypot(avg_direction_x, avg_direction_y)
                    if length > 0:
                        avg_direction_x /= length
                        avg_direction_y /= length
            else:
                avg_direction_x = next_point[0] - current_point[0]
                avg_direction_y = next_point[1] - current_point[1]
                length = math.hypot(avg_direction_x, avg_direction_y)
                if length > 0:
                    avg_direction_x /= length
                    avg_direction_y /= length

            segment_length = math.hypot(next_point[0] - current_point[0], next_point[1] - current_point[1])
            control_distance = min(segment_length * control_weight, max_control_distance)
            control_point = (
                current_point[0] + avg_direction_x * control_distance,
                current_point[1] + avg_direction_y * control_distance,
            )

        # 生成贝塞尔曲线点
        segment_length = math.hypot(next_point[0] - current_point[0], next_point[1] - current_point[1])
        num_points = max(3, int(segment_length / max(point_spacing, 1e-6)))
        for j in range(num_points):
            t = j / (num_points - 1)
            bx = (1 - t) ** 2 * current_point[0] + 2 * (1 - t) * t * control_point[0] + t ** 2 * next_point[0]
            by = (1 - t) ** 2 * current_point[1] + 2 * (1 - t) * t * control_point[1] + t ** 2 * next_point[1]

            if safety_checker(bx, by):
                smoothed_points.append((bx, by))
            else:
                if j == 0:
                    smoothed_points.append(current_point)
                elif j == num_points - 1:
                    smoothed_points.append(next_point)

    # 去重并确保起终点
    if not smoothed_points:
        return list(path)

    final_path: List[Point] = [smoothed_points[0]]
    for pt in smoothed_points[1:]:
        dx = pt[0] - final_path[-1][0]
        dy = pt[1] - final_path[-1][1]
        if math.hypot(dx, dy) > min_spacing:
            final_path.append(pt)

    if final_path[-1] != path[-1]:
        final_path.append(path[-1])

    if logger is not None:
        try:
            logger.info(f'贝塞尔曲线平滑：原始{len(path)}个点 -> 平滑后{len(final_path)}个点')
        except Exception:
            pass

    return final_path


