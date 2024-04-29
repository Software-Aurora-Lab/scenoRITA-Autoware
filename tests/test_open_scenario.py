import os
import unittest
from pathlib import Path

from ruamel.yaml import YAML
from datetime import datetime

from autoware.map_service import MapLoader, MapService
from autoware.open_scenario import OpenScenario
from autoware.scenario.validation.scenario_helper import validate
from config import PROJECT_ROOT
from scenoRITA.components.scenario_generator import ScenarioGenerator
from scenoRITA.representation import ObstacleType


class TestOpenScenario(unittest.TestCase):
    def setUp(self):
        self.map_name = "YTU Davutpasa campus"
        if not os.path.exists(os.path.join(PROJECT_ROOT, "tests", "output", self.map_name)):
            os.makedirs(os.path.join(PROJECT_ROOT, "tests", "output", self.map_name))
        self.mapLoader = MapLoader(self.map_name)
        self.mapService = MapService.instance()
        self.scenario_generator = ScenarioGenerator(self.mapService)

    def test_generate_veh_obstacle_route(self):
        print(self.scenario_generator.generate_obstacle_route(ObstacleType.CAR))

    def test_generate_ped_obstacle_route(self):
        print(self.scenario_generator.generate_obstacle_route(ObstacleType.PEDESTRIAN))

    def test_generate_bic_obstacle_route(self):
        print(self.scenario_generator.generate_obstacle_route(ObstacleType.BICYCLE))

    def test_generate_ego_car(self):
        self.scenario_generator.generate_ego_car()

    def test_generate_obstacle(self):
        ego_car = self.scenario_generator.generate_ego_car()
        obs = self.scenario_generator.generate_obstacle(ego_car)
        print(obs)
        osc = OpenScenario(1, 2, ego_car, [obs], self.map_name)
        _obs = osc.get_corresponding_entity(obs)
        print(_obs.scenario_obj())
        print(_obs.storyboard_obj())


    def test_generate_2_obs(self):
        ego_car = self.scenario_generator.generate_ego_car()
        obses = self.generate_num_of_obstacles(2, ego_car)

        osc = OpenScenario(0, 0, ego_car, obses, self.map_name)
        self.output_scenarios(osc, 2)

        self.validate_scenario(Path(f"./output/{self.map_name}"))

    def generate_num_of_obstacles(self, num: int, ego_car):
        return [self.scenario_generator.generate_obstacle(ego_car) for _ in range(num)]

    def output_scenarios(self, scenario, num):
        j = scenario.get_scenario_profile()
        file_name = f"./output/{self.map_name}/gen_{num}_obs_{datetime.now().strftime('%Y_%m_%d_%H_%M_%S_%f')}.yml"
        fp = open(file_name, 'w+')
        yaml = YAML()
        yaml.default_flow_style = False
        yaml.dump(j, fp)
        fp.close()

    def test_generate_15_obs_scenario(self):
        ego_car = self.scenario_generator.generate_ego_car()
        obses = self.generate_num_of_obstacles(15, ego_car)

        osc = OpenScenario(0, 0, ego_car, obses, self.map_name)
        self.output_scenarios(osc, 15)

        self.validate_scenario(Path(f"./output/{self.map_name}"))

    def test_generate_5_obs_scenario(self):
        ego_car = self.scenario_generator.generate_ego_car()
        obses = self.generate_num_of_obstacles(5, ego_car)

        osc = OpenScenario(0, 0, ego_car, obses, self.map_name)
        self.output_scenarios(osc, 5)

        self.validate_scenario(Path(f"./output/{self.map_name}"))

    def test_generate_multiple_2_obs(self):
        for i in range(10):
            self.test_generate_2_obs()

    def test_ego_car_init_position(self):
        print(MapService.instance().get_junction_lanes())
        print(MapService.instance().get_non_junction_lanes())
        for i in range(10):
            ego_car = self.scenario_generator.generate_ego_car()
            print(ego_car.initial_position)

    def validate_scenario(self, scenario):
        try:
            validate(scenario)
        except:
            self.fail("Scenario validation failed")

    def tearDown(self):
        del self.mapLoader


if __name__ == '__main__':
    unittest.main()
