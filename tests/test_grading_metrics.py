import unittest
import shutil
from pathlib import Path

import pytest

from download_rosbags import download_and_extract_zip
from scenoRITA.components.grading_metrics import grade_scenario
from scenoRITA.components.oracles import RecordAnalyzer
from scenoRITA.components.oracles.impl.UnsafeLaneChange import UnsafeLaneChange
from autoware.map_service import MapLoader, load_map_service


class TestGradingMetrics(unittest.TestCase):

    # @pytest.fixture(autouse=True)
    # def run_before_and_after_tests(self):
    #     # self.dir_name = "test_created_rosbag2"
    #     # self.oracle_record_dir = None
    #
    #     yield
    #
    #     assert self.oracle_record_dir is not None
    #     shutil.rmtree(str(self.oracle_record_dir.parent))

    def test_simple(self):
        load_map_service("Shalun with road shoulders")
        print(grade_scenario("test", Path(
            "/home/lori/Downloads/UC-23-004-0001-With2Obstacles-ShalunRS_case_ceef11db-a76c-5e79-98b6-8f611e90a1c9.bag")))


if __name__ == '__main__':
    unittest.main()
