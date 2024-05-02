from scenoRITA.components.oracles.BasicMetric import BasicMetric
from scenoRITA.components.oracles.MetricManager import MetricManager
from autoware.rosbag_reader import ROSBagReader
from scenoRITA.components.oracles.OracleInterrupt import OracleInterrupt
from typing import List


class RecordAnalyzer:
    record_path: str

    def __init__(self, record_path: str, oracles: List[BasicMetric]) -> None:
        self.oracle_manager = MetricManager()
        self.record_path = record_path
        self.oracles = oracles
        self.register_oracles()

    def register_oracles(self):
        for o in self.oracles:
            self.oracle_manager.register_oracle(o)

    def topic_names(self):
        return [
            '/localization/acceleration',
            '/localization/kinematic_state',
            '/perception/object_recognition/ground_truth/objects',
            '/planning/mission_planning/route',
        ]

    def analyze(self):
        has_localization = False
        has_ground_truth = False
        reader = ROSBagReader(self.record_path)
        for topic, message, t in reader.read_messages():
            if not has_localization and topic == '/localization/kinematic_state':
                has_localization = True
            if not has_ground_truth and topic == '/perception/object_recognition/ground_truth/objects':
                has_ground_truth = True
            if topic in self.topic_names():
                msg = reader.deserialize_msg(message, topic)
                try:
                    self.oracle_manager.on_new_message(topic, msg, t)
                except OracleInterrupt:
                    break
        assert has_localization, "No localization in record"
        assert has_ground_truth, "No ground truth in record"
        del reader
        return self.get_results()

    def get_results(self):
        return self.oracle_manager.get_results()
