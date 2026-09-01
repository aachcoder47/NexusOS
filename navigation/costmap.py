"""
NexusOS Autonomous Navigation - Multi-Layer 2D Occupancy Costmap.
Provides Static Map Layer, Dynamic LiDAR Point Cloud Layer, and Exponential Clearance Inflation.
"""

from __future__ import annotations
import math
from typing import List, Tuple, Dict, Any, Optional
import numpy as np


class Costmap2D:
    """
    2D Occupancy Grid Costmap with multi-layer obstacle fusion and inflation.
    Cost values:
      - 0: Free space
      - 1..252: Clearance cost gradient
      - 253: Inscribed robot footprint obstacle
      - 254: Lethal obstacle
      - 255: Unknown / unmapped
    """

    LETHAL_OBSTACLE = 254
    INSCRIBED_INFLATED = 253
    FREE_SPACE = 0
    UNKNOWN_SPACE = 255

    def __init__(
        self,
        width_m: float = 30.0,
        height_m: float = 30.0,
        resolution: float = 0.1,           # 10 cm per grid cell
        origin_x: float = -15.0,           # Lower-left corner in world coordinates
        origin_y: float = -15.0,
        robot_radius: float = 0.35,        # 35 cm robot footprint
        inflation_radius: float = 0.8,     # 80 cm safety inflation
        cost_scaling_factor: float = 3.0   # Exponential decay constant
    ):
        self.width_m = float(width_m)
        self.height_m = float(height_m)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.robot_radius = float(robot_radius)
        self.inflation_radius = float(inflation_radius)
        self.cost_scaling_factor = float(cost_scaling_factor)

        self.size_x = int(math.ceil(self.width_m / self.resolution))
        self.size_y = int(math.ceil(self.height_m / self.resolution))

        # Layers
        self.static_layer = np.zeros((self.size_y, self.size_x), dtype=np.uint8)
        self.dynamic_layer = np.zeros((self.size_y, self.size_x), dtype=np.uint8)
        self.costmap = np.zeros((self.size_y, self.size_x), dtype=np.uint8)

        # Precompute inflation kernel
        self._inflation_kernel = self._generate_inflation_kernel()

    def _generate_inflation_kernel(self) -> List[Tuple[int, int, int]]:
        """Precompute circular inflation cost offsets."""
        kernel = []
        cell_radius = int(math.ceil(self.inflation_radius / self.resolution))
        inscribed_cells = self.robot_radius / self.resolution

        for dy in range(-cell_radius, cell_radius + 1):
            for dx in range(-cell_radius, cell_radius + 1):
                dist_cells = math.sqrt(dx * dx + dy * dy)
                dist_m = dist_cells * self.resolution

                if dist_m > self.inflation_radius:
                    continue

                if dist_m <= self.robot_radius:
                    cost = self.INSCRIBED_INFLATED
                else:
                    # Exponential cost decay: cost = 253 * exp(-k * (dist - r))
                    decay = math.exp(-self.cost_scaling_factor * (dist_m - self.robot_radius))
                    cost = int(round(self.INSCRIBED_INFLATED * decay))
                    cost = max(1, min(self.INSCRIBED_INFLATED - 1, cost))

                kernel.append((dx, dy, cost))
        return kernel

    def world_to_map(self, wx: float, wy: float) -> Optional[Tuple[int, int]]:
        """Convert world coordinates (meters) to costmap grid indices (mx, my)."""
        mx = int((wx - self.origin_x) / self.resolution)
        my = int((wy - self.origin_y) / self.resolution)
        if 0 <= mx < self.size_x and 0 <= my < self.size_y:
            return (mx, my)
        return None

    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        """Convert costmap grid indices to world coordinates (meters)."""
        wx = self.origin_x + (mx + 0.5) * self.resolution
        wy = self.origin_y + (my + 0.5) * self.resolution
        return (round(wx, 3), round(wy, 3))

    def set_static_obstacle(self, wx: float, wy: float, width: float, height: float):
        """Set a rectangular static obstacle in the static map layer."""
        min_pt = self.world_to_map(wx - width * 0.5, wy - height * 0.5)
        max_pt = self.world_to_map(wx + width * 0.5, wy + height * 0.5)

        if not min_pt or not max_pt:
            return

        x0, y0 = max(0, min_pt[0]), max(0, min_pt[1])
        x1, y1 = min(self.size_x, max_pt[0] + 1), min(self.size_y, max_pt[1] + 1)
        self.static_layer[y0:y1, x0:x1] = self.LETHAL_OBSTACLE
        self.update_inflation()

    def update_from_lidar(self, point_cloud_world: List[Dict[str, float]]):
        """Update dynamic obstacle layer from real-time LiDAR point cloud scans."""
        # Clear previous dynamic layer
        self.dynamic_layer.fill(0)

        for pt in point_cloud_world:
            grid_idx = self.world_to_map(pt["x"], pt["y"])
            if grid_idx:
                mx, my = grid_idx
                self.dynamic_layer[my, mx] = self.LETHAL_OBSTACLE

        self.update_inflation()

    def update_inflation(self):
        """Combine static + dynamic layers and inflate obstacles for collision-free planning."""
        # Combine lethal obstacles
        base_obstacles = np.maximum(self.static_layer, self.dynamic_layer)
        self.costmap = np.copy(base_obstacles)

        # Inflate lethal cells
        lethal_y, lethal_x = np.where(base_obstacles >= self.LETHAL_OBSTACLE)

        for lx, ly in zip(lethal_x, lethal_y):
            for dx, dy, cost in self._inflation_kernel:
                nx = lx + dx
                ny = ly + dy
                if 0 <= nx < self.size_x and 0 <= ny < self.size_y:
                    if self.costmap[ny, nx] < cost:
                        self.costmap[ny, nx] = cost

    def get_cost(self, wx: float, wy: float) -> int:
        """Get the cost value at world position (wx, wy)."""
        idx = self.world_to_map(wx, wy)
        if not idx:
            return self.LETHAL_OBSTACLE
        return int(self.costmap[idx[1], idx[0]])

    def is_collision_free(self, wx: float, wy: float) -> bool:
        """Return True if position wx, wy is free of lethal / inscribed obstacle costs."""
        return self.get_cost(wx, wy) < self.INSCRIBED_INFLATED

    def to_dict(self) -> Dict[str, Any]:
        """Export costmap metadata and downsampled occupancy for visualization."""
        # Downsample for lightweight JSON wire transmission
        step = max(1, int(0.5 / self.resolution))  # 0.5m grid step
        downsampled = self.costmap[::step, ::step]

        return {
            "width_m": self.width_m,
            "height_m": self.height_m,
            "resolution": self.resolution,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "robot_radius": self.robot_radius,
            "grid_shape": [int(downsampled.shape[0]), int(downsampled.shape[1])],
            "obstacle_count": int(np.count_nonzero(self.costmap >= self.INSCRIBED_INFLATED))
        }
