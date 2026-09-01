"""
NexusOS Unit Tests - Autonomous Navigation & Pathfinding Stack.
Tests Costmaps, A*, Theta*, RRT*, D* Lite, DWA, Pure Pursuit, and Navigator State Machine.
"""

import unittest
import math

from navigation.costmap import Costmap2D
from navigation.planners.astar import AStarPlanner
from navigation.planners.theta_star import ThetaStarPlanner
from navigation.planners.rrt_star import RRTStarPlanner
from navigation.planners.d_star_lite import DStarLitePlanner
from navigation.local_planners.dwa import DWALocalPlanner
from navigation.local_planners.pure_pursuit import PurePursuitController
from navigation.navigator import AutonomousNavigator, NavigationState


class TestCostmap2D(unittest.TestCase):
    def test_costmap_inflation(self):
        costmap = Costmap2D(width_m=10.0, height_m=10.0, resolution=0.1, origin_x=-5.0, origin_y=-5.0)

        # Place obstacle at center (0, 0)
        costmap.set_static_obstacle(0.0, 0.0, width=1.0, height=1.0)

        # Center should be lethal obstacle (254)
        center_cost = costmap.get_cost(0.0, 0.0)
        self.assertEqual(center_cost, Costmap2D.LETHAL_OBSTACLE)

        # Point 0.6m away is inside inscribed inflation zone
        inscribed_cost = costmap.get_cost(0.6, 0.0)
        self.assertGreaterEqual(inscribed_cost, Costmap2D.INSCRIBED_INFLATED)

        # Point 0.9m away has exponential decay clearance gradient
        gradient_cost = costmap.get_cost(0.9, 0.0)
        self.assertGreater(gradient_cost, 0)
        self.assertLess(gradient_cost, Costmap2D.INSCRIBED_INFLATED)

        # Point 4.0m away should be free space (0)
        far_cost = costmap.get_cost(4.0, 0.0)
        self.assertEqual(far_cost, Costmap2D.FREE_SPACE)


class TestGlobalPlanners(unittest.TestCase):
    def setUp(self):
        self.costmap = Costmap2D(width_m=20.0, height_m=20.0, resolution=0.1, origin_x=-10.0, origin_y=-10.0)
        # Add a wall with a doorway: Wall at x = 0, y from -8 to +8 except doorway at y in [-1, 1]
        self.costmap.set_static_obstacle(0.0, 4.0, width=0.4, height=6.0)
        self.costmap.set_static_obstacle(0.0, -4.0, width=0.4, height=6.0)

    def test_astar_planner(self):
        planner = AStarPlanner(self.costmap)
        start = (-4.0, 0.0)
        goal = (4.0, 0.0)

        path = planner.plan(start, goal, smooth=True)
        self.assertTrue(len(path) >= 2)
        self.assertAlmostEqual(path[0][0], start[0], delta=0.2)
        self.assertAlmostEqual(path[-1][0], goal[0], delta=0.2)

        # Verify all waypoints are collision-free
        for pt in path:
            self.assertTrue(self.costmap.is_collision_free(pt[0], pt[1]))

    def test_theta_star_planner(self):
        planner = ThetaStarPlanner(self.costmap)
        start = (-4.0, 0.0)
        goal = (4.0, 0.0)

        path = planner.plan(start, goal)
        self.assertTrue(len(path) >= 2)
        self.assertAlmostEqual(path[-1][0], goal[0], delta=0.2)

    def test_rrt_star_planner(self):
        planner = RRTStarPlanner(self.costmap, max_iterations=600)
        start = (-3.0, 0.0)
        goal = (3.0, 0.0)

        path = planner.plan(start, goal)
        self.assertTrue(len(path) >= 2)

    def test_d_star_lite_planner(self):
        planner = DStarLitePlanner(self.costmap)
        start = (-3.0, 0.0)
        goal = (3.0, 0.0)

        path = planner.plan(start, goal)
        self.assertTrue(len(path) >= 2)


class TestLocalPlanners(unittest.TestCase):
    def setUp(self):
        self.costmap = Costmap2D(width_m=10.0, height_m=10.0, resolution=0.1, origin_x=-5.0, origin_y=-5.0)

    def test_dwa_local_planner(self):
        planner = DWALocalPlanner(self.costmap, max_speed=1.0)
        pose = (0.0, 0.0, 0.0)
        twist = (0.0, 0.0)
        target = (3.0, 0.0)

        v_cmd, w_cmd, best_traj, _ = planner.compute_velocity_command(pose, twist, target)
        self.assertGreater(v_cmd, 0.0)
        self.assertAlmostEqual(w_cmd, 0.0, delta=0.3)
        self.assertTrue(len(best_traj) > 0)

    def test_pure_pursuit_controller(self):
        controller = PurePursuitController(target_speed=1.0)
        pose = (0.0, 0.0, 0.0)
        path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]

        v_cmd, w_cmd, lookahead_pt, reached = controller.compute_control(pose, 0.5, path)
        self.assertGreater(v_cmd, 0.0)
        self.assertFalse(reached)
        self.assertIsNotNone(lookahead_pt)


class TestAutonomousNavigator(unittest.TestCase):
    def test_navigator_full_cycle(self):
        costmap = Costmap2D(width_m=15.0, height_m=15.0, resolution=0.1, origin_x=-7.5, origin_y=-7.5)
        nav = AutonomousNavigator(costmap=costmap, planner_type="A_STAR", local_planner_type="PURE_PURSUIT")

        start = (0.0, 0.0, 0.0)
        goal = (3.0, 2.0, 0.0)

        # Set goal
        success = nav.set_goal(start, goal)
        self.assertTrue(success)
        self.assertEqual(nav.state, NavigationState.NAVIGATING)

        # Step navigation update
        v, w = nav.update((0.0, 0.0, 0.0), (0.0, 0.0))
        self.assertGreater(v, 0.0)

        # Simulate arrival at goal
        v_arr, w_arr = nav.update((3.0, 2.0, 0.0), (0.0, 0.0))
        self.assertEqual(nav.state, NavigationState.GOAL_REACHED)
        self.assertEqual(v_arr, 0.0)
        self.assertEqual(w_arr, 0.0)


if __name__ == "__main__":
    unittest.main()
