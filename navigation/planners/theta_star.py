"""
NexusOS Autonomous Navigation - Theta* (Any-Angle) Global Path Planner.
Computes true straight-line paths bypassing grid step orientation constraints.
"""

from __future__ import annotations
import math
import heapq
from typing import List, Tuple, Optional, Dict
import numpy as np

from navigation.costmap import Costmap2D


class ThetaStarPlanner:
    """
    Theta* Any-Angle Path Planner.
    Extends A* by evaluating Line-of-Sight (LOS) from parent(current) directly to neighbor.
    Generates shorter, smoother trajectories with zero zigzag grid artifacts.
    """

    def __init__(self, costmap: Costmap2D):
        self.costmap = costmap
        SQRT2 = math.sqrt(2.0)
        self.motions = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (-1, -1, SQRT2)
        ]

    def plan(
        self,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float]
    ) -> List[Tuple[float, float]]:
        start_idx = self.costmap.world_to_map(*start_world)
        goal_idx = self.costmap.world_to_map(*goal_world)

        if not start_idx or not goal_idx:
            return []

        if not self.costmap.is_collision_free(*start_world) or not self.costmap.is_collision_free(*goal_world):
            return []

        open_set: List[Tuple[float, float, Tuple[int, int]]] = []
        heapq.heappush(open_set, (0.0, 0.0, start_idx))

        parent: Dict[Tuple[int, int], Tuple[int, int]] = {start_idx: start_idx}
        g_scores: Dict[Tuple[int, int], float] = {start_idx: 0.0}

        gx, gy = goal_idx

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current == goal_idx:
                raw_path = self._reconstruct_path(parent, start_idx, goal_idx)
                return [self.costmap.map_to_world(mx, my) for (mx, my) in raw_path]

            cx, cy = current
            p_curr = parent[current]

            for dx, dy, step_dist in self.motions:
                nx, ny = cx + dx, cy + dy

                if not (0 <= nx < self.costmap.size_x and 0 <= ny < self.costmap.size_y):
                    continue

                if self.costmap.costmap[ny, nx] >= Costmap2D.INSCRIBED_INFLATED:
                    continue

                neighbor = (nx, ny)

                # Line of Sight check between parent(current) and neighbor (Theta* key step)
                if self._line_of_sight(p_curr, neighbor):
                    # Path 2: Direct line from parent(current) to neighbor
                    los_dist = math.hypot(nx - p_curr[0], ny - p_curr[1])
                    tentative_g = g_scores[p_curr] + los_dist

                    if tentative_g < g_scores.get(neighbor, float("inf")):
                        g_scores[neighbor] = tentative_g
                        parent[neighbor] = p_curr
                        h = math.hypot(nx - gx, ny - gy)
                        heapq.heappush(open_set, (tentative_g + h, tentative_g, neighbor))
                else:
                    # Path 1: Standard A* step from current to neighbor
                    tentative_g = g_scores[current] + step_dist

                    if tentative_g < g_scores.get(neighbor, float("inf")):
                        g_scores[neighbor] = tentative_g
                        parent[neighbor] = current
                        h = math.hypot(nx - gx, ny - gy)
                        heapq.heappush(open_set, (tentative_g + h, tentative_g, neighbor))

        return []

    def _line_of_sight(self, s1: Tuple[int, int], s2: Tuple[int, int]) -> bool:
        """Bresenham-style raycast on costmap grid to verify collision-free line-of-sight."""
        x0, y0 = s1
        x1, y1 = s2

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        cx, cy = x0, y0
        while True:
            if not (0 <= cx < self.costmap.size_x and 0 <= cy < self.costmap.size_y):
                return False
            if self.costmap.costmap[cy, cx] >= Costmap2D.INSCRIBED_INFLATED:
                return False

            if cx == x1 and cy == y1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy

        return True

    def _reconstruct_path(
        self, parent: Dict[Tuple[int, int], Tuple[int, int]], start: Tuple[int, int], goal: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        path = [goal]
        curr = goal
        while curr != start:
            curr = parent[curr]
            path.append(curr)
        path.reverse()
        return path
