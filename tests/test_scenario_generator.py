import unittest

from autoware.map_service import MapLoader, MapService
from scenoRITA.components.scenario_generator import ScenarioGenerator
from scenoRITA.representation import ObstacleType


class TestScenarioGenerator(unittest.TestCase):
    def setUp(self):
        self.mapLoader = MapLoader("ces2024_demo")
        self.mapService = MapService.instance()
        self.scenario_generator = ScenarioGenerator(self.mapService)

    def test_generate_obstacle_type(self):
        self.assertIn(
            self.scenario_generator.generate_obstacle_type(),
            [ObstacleType.VEHICLE, ObstacleType.PEDESTRIAN, ObstacleType.BICYCLE]
        )

    def test_generate_obstacle_route(self):
        print(self.scenario_generator.generate_obstacle_route(ObstacleType.PEDESTRIAN))

    def test_generate_ego_car(self):
        print(self.scenario_generator.generate_ego_car())

    def test_generate_obstacle(self):
        ego_car = self.scenario_generator.generate_ego_car()
        print(self.scenario_generator.generate_obstacle(ego_car))

    def tearDown(self):
        del self.mapLoader


if __name__ == '__main__':
    unittest.main()
