# tests/test_mission_planner.py

import unittest
import numpy as np

from ilas.mission.mission_planner import MissionPlanner

class TestMissionPlanner(unittest.TestCase):

    def test_mission_planner_init(self):
        config = {}
        planner = MissionPlanner(config)
        self.assertIsNotNone(planner)

    def test_generate_plan_no_obstacles(self):
        config = {'map_resolution': 1}
        planner = MissionPlanner(config)

        start = np.array([0, 0, 0])
        goal = np.array([3, 3, 3])
        map_data = {'map': np.zeros((5, 5, 5))}

        plan = planner.generate_plan(start, goal, map_data)
        self.assertIsNotNone(plan)
        path_length = np.sum(np.linalg.norm(np.diff(plan, axis=0), axis=1))
        manhattan_distance = np.sum(np.abs(goal - start))
        self.assertLessEqual(path_length, manhattan_distance + planner.map_resolution)
        self.assertTrue(np.allclose(plan[0], start))
        self.assertTrue(np.allclose(plan[-1], goal))

    def test_generate_plan_with_obstacle(self):
        config = {'map_resolution': 1}
        planner = MissionPlanner(config)

        start = np.array([0, 0, 0])
        goal = np.array([3, 3, 3])

        # Create a map with an obstacle in the middle
        slam_map = np.zeros((5, 5, 5))
        slam_map[1:3, 1:3, 1:3] = 1 # Obstacle
        map_data = {'map': slam_map}

        plan = planner.generate_plan(start, goal, map_data)
        self.assertIsNotNone(plan)

        # Check that the path does not go through the obstacle
        for waypoint in plan:
            grid_pos = (waypoint / planner.map_resolution).astype(int)
            self.assertEqual(slam_map[grid_pos[0], grid_pos[1], grid_pos[2]], 0)

    def test_adapt_plan(self):
        config = {'map_resolution': 1}
        planner = MissionPlanner(config)

        start = np.array([0, 0, 0])
        goal = np.array([3, 3, 3])
        map_data = {'map': np.zeros((5, 5, 5))}

        plan = planner.generate_plan(start, goal, map_data)

        current_position = np.array([1, 1, 1])

        new_plan = planner.adapt_plan(plan, current_position, map_data)
        self.assertIsNone(new_plan)

if __name__ == '__main__':
    unittest.main()
