"""
NexusOS Autonomous Navigation - RRT* (Rapidly-exploring Random Tree Star).
Asymptotically optimal sampling-based path planner for complex environments.
"""

from __future__ import annotations
import math
import random
from typing import List, Tuple, Optional

from navigation.costmap import Costmap2D


class RRTNode:
    def __init__(self, x: float, y: float, parent: Optional[RRTNode] = None, cost: float = 0.0):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost


class RRTStarPlanner:
    """
    RRT* Sampling-based Global Planner with tree rewiring for asymptotic path optimality.
    """

    def __init__(
        self,
        costmap: Costmap2D,
        step_size: float = 0.5,
        search_radius: float = 1.5,
        max_iterations: int = 1200,
        goal_sample_rate: float = 0.15
    ):
        self.costmap = costmap
        self.step_size = float(step_size)
        self.search_radius = float(search_radius)
        self.max_iterations = int(max_iterations)
        self.goal_sample_rate = float(goal_sample_rate)

    def plan(
        self,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float]
    ) -> List[Tuple[float, float]]:
        if not self.costmap.is_collision_free(*start_world) or not self.costmap.is_collision_free(*goal_world):
            return []

        start_node = RRTNode(start_world[0], start_world[1], None, 0.0)
        goal_node = RRTNode(goal_world[0], goal_world[1], None, float("inf"))
        nodes: List[RRTNode] = [start_node]

        min_x = self.costmap.origin_x
        max_x = self.costmap.origin_x + self.costmap.width_m
        min_y = self.costmap.origin_y
        max_y = self.costmap.origin_y + self.costmap.height_m

        for _ in range(self.max_iterations):
            # Sample point with goal biasing
            if random.random() < self.goal_sample_rate:
                rnd_x, rnd_y = goal_world
            else:
                rnd_x = random.uniform(min_x, max_x)
                rnd_y = random.uniform(min_y, max_y)

            # Find nearest node
            nearest = min(nodes, key=lambda n: math.hypot(n.x - rnd_x, n.y - rnd_y))

            # Steer towards random sample
            theta = math.atan2(rnd_y - nearest.y, rnd_x - nearest.x)
            dist = math.hypot(rnd_x - nearest.x, rnd_y - nearest.y)
            step = min(self.step_size, dist)
            new_x = nearest.x + step * math.cos(theta)
            new_y = nearest.y + step * math.sin(theta)

            # Collision check
            if not self._is_segment_free((nearest.x, nearest.y), (new_x, new_y)):
                continue

            # Find near neighbors within search radius
            near_nodes = [n for n in nodes if math.hypot(n.x - new_x, n.y - new_y) <= self.search_radius]

            # Choose best parent with minimal cost
            min_cost = nearest.cost + step
            best_parent = nearest

            for near in near_nodes:
                d = math.hypot(near.x - new_x, near.y - new_y)
                if near.cost + d < min_cost and self._is_segment_free((near.x, near.y), (new_x, new_y)):
                    min_cost = near.cost + d
                    best_parent = near

            new_node = RRTNode(new_x, new_y, best_parent, min_cost)
            nodes.append(new_node)

            # Rewire tree
            for near in near_nodes:
                d = math.hypot(near.x - new_x, near.y - new_y)
                if new_node.cost + d < near.cost and self._is_segment_free((new_x, new_y), (near.x, near.y)):
                    near.parent = new_node
                    near.cost = new_node.cost + d

            # Check if close to goal
            dist_to_goal = math.hypot(new_x - goal_world[0], new_y - goal_world[1])
            if dist_to_goal <= self.step_size:
                if self._is_segment_free((new_x, new_y), goal_world):
                    if new_node.cost + dist_to_goal < goal_node.cost:
                        goal_node.parent = new_node
                        goal_node.cost = new_node.cost + dist_to_goal

        if goal_node.parent is not None:
            # Reconstruct path
            path = [(goal_world[0], goal_world[1])]
            curr = goal_node.parent
            while curr is not None:
                path.append((curr.x, curr.y))
                curr = curr.parent
            path.reverse()
            return path

        return []

    def _is_segment_free(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> bool:
        dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        steps = max(2, int(dist / (self.costmap.resolution * 0.5)))
        for i in range(steps + 1):
            t = i / float(steps)
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            if not self.costmap.is_collision_free(x, y):
                return False
        return True
