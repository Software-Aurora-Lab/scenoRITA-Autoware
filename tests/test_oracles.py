import unittest
import shutil
import pytest

from download_rosbags import download_and_extract_zip
from scenoRITA.components.oracles import RecordAnalyzer
from scenoRITA.components.oracles.impl.Comfort import Comfort
from scenoRITA.components.oracles.impl.Collision import Collision
from scenoRITA.components.oracles.impl.Speeding import Speeding
from scenoRITA.components.oracles.impl.UnsafeLaneChange import UnsafeLaneChange
from autoware.map_service import MapLoader


class TestOracles(unittest.TestCase):

    @pytest.fixture(autouse=True)
    def run_before_and_after_tests(self):
        self.dir_name = "test_created_rosbag2"
        self.oracle_record_dir = None

        yield

        assert self.oracle_record_dir is not None
        shutil.rmtree(str(self.oracle_record_dir.parent))

    def test_comfort_oracle(self):
        self.oracle_record_dir = download_and_extract_zip("1YQ76YZ0BY2wUeGmMxxXcoMRozJU8Mv-A", self.dir_name)

        target_oracles = [Comfort()]
        analyzer = RecordAnalyzer(record_path=str(self.oracle_record_dir), oracles=target_oracles)
        violations = analyzer.analyze()

        self.assertEqual(1, len(violations))
        violation = violations[0]
        self.assertEqual(violation.main_type, 'DecelOracle')
        self.assertLessEqual(violation.features['accel'], -2.5, "accel value should be less or equal than -2.5")

    def test_collision_oracle_w_collision(self):
        # v = 8.4, collision happens, but not a rear-end collision
        self.oracle_record_dir = download_and_extract_zip("1HlEGtGsIRhJW_-83q4qebMJ1_-0oToel", self.dir_name)

        target_oracles = [Collision()]
        analyzer = RecordAnalyzer(record_path=str(self.oracle_record_dir), oracles=target_oracles)
        violations = analyzer.analyze()

        self.assertEqual(0, len(violations))

    def test_collision_oracle_wo_collision(self):
        # v = 14
        self.oracle_record_dir = download_and_extract_zip("169_pr49O5mqRca9yFmudVCdOAI2T5UTz", self.dir_name)

        target_oracles = [Collision()]
        analyzer = RecordAnalyzer(record_path=str(self.oracle_record_dir), oracles=target_oracles)
        violations = analyzer.analyze()
        self.assertEqual(0, len(violations))

    def test_collision_oracle_wo_collision_2(self):
        # https://drive.google.com/file/d/1xMtLNKDnPfSMrDxTVHCwkqNR1dOB-_uy/view?usp=sharing
        self.oracle_record_dir = download_and_extract_zip("1xMtLNKDnPfSMrDxTVHCwkqNR1dOB-_uy", self.dir_name)

        target_oracles = [Collision()]
        analyzer = RecordAnalyzer(record_path=str(self.oracle_record_dir), oracles=target_oracles)
        violations = analyzer.analyze()

        self.assertEqual(0, len(violations))

    def test_speeding_oracle(self):
        self.oracle_record_dir = download_and_extract_zip("1YQ76YZ0BY2wUeGmMxxXcoMRozJU8Mv-A", self.dir_name)

        MapLoader("awf_cicd_virtual_E_dev")
        target_oracles = [Speeding()]
        analyzer = RecordAnalyzer(record_path=str(self.oracle_record_dir), oracles=target_oracles)
        violations = analyzer.analyze()
        self.assertEqual(0, len(violations))

    def test_unsafe_lane_change_oracle_1(self):
        # https://evaluation.tier4.jp/evaluation/reports/efb82cc7-9bfd-590a-8880-0d99a4513d14/tests/ce9f8aff-7bec-5678-bbf3-a5c0f5903dd0/1f056ceb-f573-547d-b17a-171277c4603d/8eb974c8-d941-5776-8c1c-60875e343be9?project_id=awf
        self.oracle_record_dir = download_and_extract_zip("1Rr-ZRFMZmhFAfQdtVOsDRcNZQesEqdVj", self.dir_name)

        MapLoader("ces2024_demo")
        target_oracles = [UnsafeLaneChange()]
        analyzer = RecordAnalyzer(record_path=str(self.oracle_record_dir), oracles=target_oracles)
        violations = analyzer.analyze()
        self.assertEqual(1, len(violations))
        self.assertEqual(violations[0].main_type, "UnsafeLaneChangeOracle")

    def test_unsafe_lane_change_oracle_2(self):
        # https://drive.google.com/file/d/1dtrtQS-wYJ32tnMlP_8J_2ue4uPWoxEo/view?usp=drive_link
        self.oracle_record_dir = download_and_extract_zip("1dtrtQS-wYJ32tnMlP_8J_2ue4uPWoxEo", self.dir_name)

        MapLoader("awf_cicd_virtual_G_dev")
        target_oracles = [UnsafeLaneChange()]
        analyzer = RecordAnalyzer(record_path=str(self.oracle_record_dir), oracles=target_oracles)
        violations = analyzer.analyze()
        self.assertEqual(1, len(violations))
        self.assertEqual(violations[0].main_type, "UnsafeLaneChangeOracle")

    def test_unsafe_lane_change_oracle_3(self):
        # https://drive.google.com/file/d/1UpULPnQqugb9XI7GploNrt9p4dqPX7aD/view?usp=sharing
        self.oracle_record_dir = download_and_extract_zip("1UpULPnQqugb9XI7GploNrt9p4dqPX7aD", self.dir_name)

        MapLoader("morai_virtual_highway_2")
        target_oracles = [UnsafeLaneChange()]
        analyzer = RecordAnalyzer(record_path=str(self.oracle_record_dir), oracles=target_oracles)
        violations = analyzer.analyze()
        self.assertEqual(1, len(violations))
        self.assertEqual(violations[0].main_type, "UnsafeLaneChangeOracle")


if __name__ == '__main__':
    unittest.main()
