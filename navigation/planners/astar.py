"""
NexusOS Autonomous Navigation - A* (A-Star) Global Path Planner.
High-throughput 8-connected grid planner with costmap clearance weighting and path smoothing.
"""

from __future__ import annotations
import math
import heapq
from typing import List, Tuple, Optional, Dict
import numpy as np

from navigation.costmap import Costmap2D


class AStarPlanner:
    """
    A* Global Pathfinding Algorithm optimized for robotics costmaps.
    Features:
      - 8-neighbor connectivity with diagonal Euclidean weights
      - Clearance-aware cost evaluation (penalizes paths near walls/corners)
      - Tie-breaking heuristic for straight path generation
      - String-pulling (Line-of-Sight) path shortcut smoothing
    """

    def __init__(self, costmap: Costmap2D, clearance_weight: float = 0.05):
        self.costmap = costmap
        self.clearance_weight = float(clearance_weight)

        # 8-connected neighbors (dx, dy, step_cost)
        SQRT2 = math.sqrt(2.0)
        self.motions = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (-1, -1, SQRT2)
        ]

    def plan(
        self,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float],
        smooth: bool = True
    ) -> List[Tuple[float, float]]:
        """
        Compute optimal collision-free path from start to goal in world coordinates.
        Returns list of (x, y) waypoints.
        """
        start_idx = self.costmap.world_to_map(*start_world)
        goal_idx = self.costmap.world_to_map(*goal_world)

        if not start_idx or not goal_idx:
            return []

        # Check if start or goal are strictly in lethal obstacles
        if not self.costmap.is_collision_free(*start_world) or not self.costmap.is_collision_free(*goal_world):
            # Try to find nearest free neighbor for goal
            goal_idx = self._find_nearest_free_cell(goal_idx)
            if not goal_idx:
                return []

        # Priority queue stores tuples: (f_score, g_score, (mx, my))
        open_set: List[Tuple[float, float, Tuple[int, int]]] = []
        heapq.heappush(open_set, (0.0, 0.0, start_idx))

        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_scores: Dict[Tuple[int, int], float] = {start_idx: 0.0}

        gx, gy = goal_idx

        while open_set:
            _, current_g, current = heapq.heappop(open_set)

            if current == goal_idx:
                # Reconstruct raw grid path
                raw_path = self._reconstruct_path(came_from, current)
                world_path = [self.costmap.map_to_world(mx, my) for (mx, my) in raw_path]
                if smooth and len(world_path) > 2:
                    return self.smooth_path(world_path)
                return world_path

            cx, cy = current

            for dx, dy, step_cost in self.motions:
                nx, ny = cx + dx, cy + dy

                if not (0 <= nx < self.costmap.size_x and 0 <= ny < self.costmap.size_y):
                    continue

                cell_cost = int(self.costmap.costmap[ny, nx])
                if cell_cost >= Costmap2D.INSCRIBED_INFLATED:
                    continue  # Lethal / Inscribed obstacle

                # Cost includes distance + clearance penalty
                extra_cost = (cell_cost / 255.0) * self.clearance_weight * step_cost
                tentative_g = current_g + step_cost + extra_cost

                neighbor = (nx, ny)
                if tentative_g < g_scores.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_scores[neighbor] = tentative_g

                    # Octile heuristic with tie-breaking
                    h = self._heuristic(nx, ny, gx, gy)
                    f = tentative_g + h * 1.001
                    heapq.heappush(open_set, (f, tentative_g, neighbor))

        return []  # No path found

    def _heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """Octile distance heuristic for 8-connected grid."""
        dx = abs(x1 - x2)
        dy = abs(y1 - y2)
        return (dx + dy) + (math.sqrt(2.0) - 2.0) * min(dx, dy)

    def _find_nearest_free_cell(self, center: Tuple[int, int], search_radius: int = 5) -> Optional[Tuple[int, int]]:
        cx, cy = center
        for r in range(1, search_radius + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < self.costmap.size_x and 0 <= ny < self.costmap.size_y:
                        if self.costmap.costmap[ny, nx] < Costmap2D.INSCRIBED_INFLATED:
                            return (nx, ny)
        return None

    def _reconstruct_path(
        self, came_from: Dict[Tuple[int, int], Tuple[int, int]], current: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        String-pulling algorithm: Prunes redundant intermediate waypoints
        if direct line-of-sight between non-adjacent waypoints is collision-free.
        """
        if len(path) <= 2:
            return path

        smoothed: List[Tuple[float, float]] = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            furthest_idx = len(path) - 1
            while furthest_idx > current_idx + 1:
                if self._is_line_free(path[current_idx], path[furthest_idx]):
                    break
                furthest_idx -= 1

            smoothed.append(path[furthest_idx])
            current_idx = furthest_idx

        return smoothed

    def _is_line_free(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> bool:
        """Ray-trace line segment and verify no lethal obstacle intersection."""
        dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        steps = max(2, int(math.ceil(dist / (self.costmap.resolution * 0.5))))

        for i in range(steps + 1):
            t = i / float(steps)
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            if not self.costmap.is_collision_free(x, y):
                return False
        return True
