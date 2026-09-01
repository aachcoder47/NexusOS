"""
NexusOS Autonomous Navigation - D* Lite Incremental Dynamic Replanner.
Rapidly repairs paths in real-time when new obstacles are detected without full graph re-expansion.
"""

from __future__ import annotations
import math
import heapq
from typing import List, Tuple, Dict, Optional, Set
import numpy as np

from navigation.costmap import Costmap2D


class DStarLitePlanner:
    """
    D* Lite dynamic path replanner.
    Performs backward search from goal to start and incrementally updates rhs values
    when obstacle map cells change.
    """

    def __init__(self, costmap: Costmap2D):
        self.costmap = costmap
        self.km = 0.0
        self.s_start: Tuple[int, int] = (0, 0)
        self.s_goal: Tuple[int, int] = (0, 0)
        self.s_last: Tuple[int, int] = (0, 0)

        self.g: Dict[Tuple[int, int], float] = {}
        self.rhs: Dict[Tuple[int, int], float] = {}
        self.U: List[Tuple[Tuple[float, float], Tuple[int, int]]] = []
        self.open_set_items: Set[Tuple[int, int]] = set()

        SQRT2 = math.sqrt(2.0)
        self.motions = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (-1, -1, SQRT2)
        ]

    def _h(self, s1: Tuple[int, int], s2: Tuple[int, int]) -> float:
        """Heuristic function."""
        return math.hypot(s1[0] - s2[0], s1[1] - s2[1])

    def _calculate_key(self, s: Tuple[int, int]) -> Tuple[float, float]:
        g_val = self.g.get(s, float("inf"))
        rhs_val = self.rhs.get(s, float("inf"))
        min_val = min(g_val, rhs_val)
        return (min_val + self._h(self.s_start, s) + self.km, min_val)

    def _get_cost(self, s1: Tuple[int, int], s2: Tuple[int, int]) -> float:
        if not (0 <= s2[0] < self.costmap.size_x and 0 <= s2[1] < self.costmap.size_y):
            return float("inf")
        if self.costmap.costmap[s2[1], s2[0]] >= Costmap2D.INSCRIBED_INFLATED:
            return float("inf")
        return math.hypot(s1[0] - s2[0], s1[1] - s2[1])

    def initialize(self, start_world: Tuple[float, float], goal_world: Tuple[float, float]) -> bool:
        s_s = self.costmap.world_to_map(*start_world)
        s_g = self.costmap.world_to_map(*goal_world)
        if not s_s or not s_g:
            return False

        self.s_start = s_s
        self.s_goal = s_g
        self.s_last = self.s_start
        self.km = 0.0

        self.g.clear()
        self.rhs.clear()
        self.U.clear()
        self.open_set_items.clear()

        self.rhs[self.s_goal] = 0.0
        k = self._calculate_key(self.s_goal)
        heapq.heappush(self.U, (k, self.s_goal))
        self.open_set_items.add(self.s_goal)

        return True

    def _update_vertex(self, u: Tuple[int, int]):
        if u != self.s_goal:
            min_rhs = float("inf")
            for dx, dy, _ in self.motions:
                s_prime = (u[0] + dx, u[1] + dy)
                cost = self._get_cost(u, s_prime)
                g_val = self.g.get(s_prime, float("inf"))
                if cost < float("inf") and g_val < float("inf"):
                    val = cost + g_val
                    if val < min_rhs:
                        min_rhs = val
            self.rhs[u] = min_rhs

        if u in self.open_set_items:
            self.open_set_items.remove(u)

        if self.g.get(u, float("inf")) != self.rhs.get(u, float("inf")):
            k = self._calculate_key(u)
            heapq.heappush(self.U, (k, u))
            self.open_set_items.add(u)

    def _compute_shortest_path(self, max_steps: int = 5000) -> bool:
        steps = 0
        while self.U and steps < max_steps:
            steps += 1
            k_old, u = heapq.heappop(self.U)
            if u not in self.open_set_items:
                continue
            self.open_set_items.remove(u)

            k_new = self._calculate_key(u)
            if k_old < k_new:
                heapq.heappush(self.U, (k_new, u))
                self.open_set_items.add(u)
                continue

            g_u = self.g.get(u, float("inf"))
            rhs_u = self.rhs.get(u, float("inf"))

            if g_u > rhs_u:
                self.g[u] = rhs_u
                for dx, dy, _ in self.motions:
                    s = (u[0] + dx, u[1] + dy)
                    if 0 <= s[0] < self.costmap.size_x and 0 <= s[1] < self.costmap.size_y:
                        self._update_vertex(s)
            else:
                self.g[u] = float("inf")
                self._update_vertex(u)
                for dx, dy, _ in self.motions:
                    s = (u[0] + dx, u[1] + dy)
                    if 0 <= s[0] < self.costmap.size_x and 0 <= s[1] < self.costmap.size_y:
                        self._update_vertex(s)

            if not self.U:
                break
            top_key = self.U[0][0]
            start_key = self._calculate_key(self.s_start)
            if top_key >= start_key and self.rhs.get(self.s_start, float("inf")) == self.g.get(self.s_start, float("inf")):
                break

        return self.rhs.get(self.s_start, float("inf")) < float("inf")

    def plan(
        self, start_world: Tuple[float, float], goal_world: Tuple[float, float]
    ) -> List[Tuple[float, float]]:
        if not self.initialize(start_world, goal_world):
            return []

        if not self._compute_shortest_path():
            return []

        # Extract path
        curr = self.s_start
        path_grid = [curr]
        visited = {curr}

        while curr != self.s_goal and len(path_grid) < 1000:
            min_c = float("inf")
            next_s = None
            for dx, dy, _ in self.motions:
                s = (curr[0] + dx, curr[1] + dy)
                if 0 <= s[0] < self.costmap.size_x and 0 <= s[1] < self.costmap.size_y:
                    cost = self._get_cost(curr, s)
                    g_val = self.g.get(s, float("inf"))
                    if cost + g_val < min_c:
                        min_c = cost + g_val
                        next_s = s

            if next_s is None or next_s in visited or min_c == float("inf"):
                break

            curr = next_s
            visited.add(curr)
            path_grid.append(curr)

        return [self.costmap.map_to_world(mx, my) for (mx, my) in path_grid]
