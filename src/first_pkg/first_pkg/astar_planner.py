#!/usr/bin/env python3
"""
A* 与 双向A* 路径规划模块

将搜索逻辑从节点中抽离，便于独立复用与维护。
返回路径默认为世界坐标（若提供 map_to_world_fn）。
"""

import math
import heapq
from typing import Callable, Iterable, List, Optional, Tuple

MapArray = Iterable  # 期望为二维可索引数组，如 numpy.ndarray[y][x]
Point = Tuple[int, int]
WorldPoint = Tuple[float, float]


class NodeAStar:
    def __init__(self, x: int, y: int, g_cost: float = 0.0, h_cost: float = 0.0, parent: Optional["NodeAStar"] = None, direction: str = "forward"):
        self.x = x
        self.y = y
        self.g_cost = g_cost
        self.h_cost = h_cost
        # 与原实现一致的权重
        self.f_cost = g_cost + 1.5 * h_cost
        self.parent = parent
        self.direction = direction

    def __lt__(self, other: "NodeAStar") -> bool:
        return self.f_cost < other.f_cost


def heuristic(x1: int, y1: int, x2: int, y2: int, weight: float = 3.0) -> float:
    """
    加权启发函数：切比雪夫距离 × 权重
    
    切比雪夫距离 = max(|x2-x1|, |y2-y1|)
    - 更适合允许对角线移动的路径规划
    - 比曼哈顿距离更准确地反映实际移动成本
    
    权重=3.0 的优化效果：
    - 提高搜索速度：更快朝向目标方向
    - 减少展开节点数：优先考虑距离目标更近的节点
    - 路径质量：略微降低最优性，但显著提升速度
    
    注意：权重>1时算法变为启发式搜索，不保证最优解
    """
    return weight * max(abs(x2 - x1), abs(y2 - y1))


def get_neighbors(node: NodeAStar, map_array: MapArray, is_node_accessible: Callable[[int, int, int], bool]) -> List[Point]:
    neighbors: List[Point] = []
    max_x = len(map_array[0])
    max_y = len(map_array)
    for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
        nx = node.x + dx
        ny = node.y + dy
        if 0 <= nx < max_x and 0 <= ny < max_y:
            map_value = map_array[ny][nx]
            if is_node_accessible(nx, ny, map_value):
                neighbors.append((nx, ny))
    return neighbors


def _reconstruct_path_from(goal_node: NodeAStar) -> List[Point]:
    path: List[Point] = []
    current = goal_node
    while current:
        path.append((current.x, current.y))
        current = current.parent
    path.reverse()
    return path


def _reconstruct_bidirectional_path(forward_node: NodeAStar, backward_node: NodeAStar) -> List[Point]:
    forward_path: List[Point] = []
    cur = forward_node
    while cur:
        forward_path.append((cur.x, cur.y))
        cur = cur.parent
    forward_path.reverse()

    backward_path: List[Point] = []
    cur = backward_node
    while cur:
        backward_path.append((cur.x, cur.y))
        cur = cur.parent

    if forward_path and backward_path and forward_path[-1] == backward_path[0]:
        backward_path = backward_path[1:]

    return forward_path + backward_path


def standard_astar(
    map_array: MapArray,
    start: Point,
    goal: Point,
    is_node_accessible: Callable[[int, int, int], bool],
    map_to_world_fn: Optional[Callable[[int, int], WorldPoint]] = None,
    logger: Optional[object] = None,
) -> List[WorldPoint]:
    start_node = NodeAStar(start[0], start[1], g_cost=0.0, h_cost=heuristic(start[0], start[1], goal[0], goal[1], weight=3.0))

    open_list: List[NodeAStar] = []
    heapq.heappush(open_list, start_node)
    closed_set = set()
    node_dict = {(start[0], start[1]): start_node}

    max_iterations = 1000000
    iteration_count = 0

    while open_list and iteration_count < max_iterations:
        iteration_count += 1
        current = heapq.heappop(open_list)
        if (current.x, current.y) == goal:
            path_map = _reconstruct_path_from(current)
            if map_to_world_fn is None:
                return [(float(x), float(y)) for x, y in path_map]
            return [map_to_world_fn(x, y) for x, y in path_map]

        closed_set.add((current.x, current.y))

        for nx, ny in get_neighbors(current, map_array, is_node_accessible):
            if (nx, ny) in closed_set:
                continue

            dx = nx - current.x
            dy = ny - current.y
            base_cost = math.sqrt(2) if abs(dx) == abs(dy) else 1.0
            map_value = map_array[ny][nx]
            risk_multiplier = 1.5 if map_value == -1 else 1.0
            new_g = current.g_cost + base_cost * risk_multiplier

            if (nx, ny) in node_dict and new_g >= node_dict[(nx, ny)].g_cost:
                continue

            neighbor = NodeAStar(nx, ny, g_cost=new_g, h_cost=heuristic(nx, ny, goal[0], goal[1], weight=3.0), parent=current)
            node_dict[(nx, ny)] = neighbor
            heapq.heappush(open_list, neighbor)

    if logger is not None:
        try:
            logger.warn(f'标准A*未找到路径，迭代次数: {iteration_count}')
        except Exception:
            pass
    return []


def bidirectional_astar(
    map_array: MapArray,
    start: Point,
    goal: Point,
    is_node_accessible: Callable[[int, int, int], bool],
    map_to_world_fn: Optional[Callable[[int, int], WorldPoint]] = None,
    logger: Optional[object] = None,
) -> List[WorldPoint]:
    forward_start = NodeAStar(start[0], start[1], g_cost=0.0, h_cost=heuristic(start[0], start[1], goal[0], goal[1], weight=3.0), direction='forward')
    backward_start = NodeAStar(goal[0], goal[1], g_cost=0.0, h_cost=heuristic(goal[0], goal[1], start[0], start[1], weight=3.0), direction='backward')

    forward_open: List[NodeAStar] = []
    backward_open: List[NodeAStar] = []
    forward_closed = {}
    backward_closed = {}
    forward_nodes = {(start[0], start[1]): forward_start}
    backward_nodes = {(goal[0], goal[1]): backward_start}

    heapq.heappush(forward_open, forward_start)
    heapq.heappush(backward_open, backward_start)

    max_iterations = 1000000
    iteration_count = 0
    best_connection = None
    best_cost = float('inf')

    if logger is not None:
        try:
            logger.info(f'双向A*算法开始搜索，最大迭代次数: {max_iterations}')
        except Exception:
            pass

    while (forward_open or backward_open) and iteration_count < max_iterations:
        iteration_count += 1
        if forward_open and (not backward_open or len(forward_open) <= len(backward_open)):
            current = heapq.heappop(forward_open)
            cur_pos = (current.x, current.y)
            if cur_pos in backward_closed:
                connection_cost = current.g_cost + backward_closed[cur_pos].g_cost
                if connection_cost < best_cost:
                    best_cost = connection_cost
                    best_connection = (current, backward_closed[cur_pos])
                    break
            forward_closed[cur_pos] = current
            # 扩展邻居
            for nx, ny in get_neighbors(current, map_array, is_node_accessible):
                if (nx, ny) in forward_closed:
                    continue
                dx = nx - current.x
                dy = ny - current.y
                base_cost = math.sqrt(2) if abs(dx) == abs(dy) else 1.0
                map_value = map_array[ny][nx]
                risk_multiplier = 1.5 if map_value == -1 else 1.0
                new_g = current.g_cost + base_cost * risk_multiplier
                if (nx, ny) in forward_nodes and new_g >= forward_nodes[(nx, ny)].g_cost:
                    continue
                neighbor = NodeAStar(nx, ny, g_cost=new_g, h_cost=heuristic(nx, ny, goal[0], goal[1], weight=3.0), parent=current, direction='forward')
                forward_nodes[(nx, ny)] = neighbor
                heapq.heappush(forward_open, neighbor)
        elif backward_open:
            current = heapq.heappop(backward_open)
            cur_pos = (current.x, current.y)
            if cur_pos in forward_closed:
                connection_cost = current.g_cost + forward_closed[cur_pos].g_cost
                if connection_cost < best_cost:
                    best_cost = connection_cost
                    best_connection = (forward_closed[cur_pos], current)
                    break
            backward_closed[cur_pos] = current
            # 扩展邻居（目标为起点）
            for nx, ny in get_neighbors(current, map_array, is_node_accessible):
                if (nx, ny) in backward_closed:
                    continue
                dx = nx - current.x
                dy = ny - current.y
                base_cost = math.sqrt(2) if abs(dx) == abs(dy) else 1.0
                map_value = map_array[ny][nx]
                risk_multiplier = 1.5 if map_value == -1 else 1.0
                new_g = current.g_cost + base_cost * risk_multiplier
                if (nx, ny) in backward_nodes and new_g >= backward_nodes[(nx, ny)].g_cost:
                    continue
                neighbor = NodeAStar(nx, ny, g_cost=new_g, h_cost=heuristic(nx, ny, start[0], start[1], weight=3.0), parent=current, direction='backward')
                backward_nodes[(nx, ny)] = neighbor
                heapq.heappush(backward_open, neighbor)

    if best_connection is None:
        if logger is not None:
            try:
                logger.warn(f'双向A*未找到路径，总迭代: {iteration_count}')
            except Exception:
                pass
        return []

    forward_node, backward_node = best_connection
    path_map = _reconstruct_bidirectional_path(forward_node, backward_node)
    if map_to_world_fn is None:
        return [(float(x), float(y)) for x, y in path_map]
    return [map_to_world_fn(x, y) for x, y in path_map]


