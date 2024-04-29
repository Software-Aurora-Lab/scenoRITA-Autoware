import unittest
from pathlib import Path
from scenoRITA.components.grading_metrics import grade_scenario
from autoware.map_service import load_map_service


class TestGradingMetrics(unittest.TestCase):

    def test_simple(self):
        load_map_service("Shalun with road shoulders")
        print(grade_scenario("test", Path(
            "/home/lori/Downloads/UC-23-004-0001-With2Obstacles-ShalunRS_case_ceef11db-a76c-5e79-98b6-8f611e90a1c9.bag")))

    def test_multiple_obs(self):
        load_map_service("Gebze (Turkey)")
        print(grade_scenario("test", Path(
            "/home/lori/Downloads/test_perception")))


if __name__ == '__main__':
    unittest.main()
