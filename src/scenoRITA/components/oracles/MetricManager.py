from collections import defaultdict
from typing import Dict, List
from scenoRITA.components.oracles.BasicMetric import BasicMetric
from scenoRITA.components.oracles.MetricHelper import MetricHelper


class MetricManager:
    __topic_oracle_mapping: Dict[str, List[BasicMetric]]
    __registered_oracles: List[BasicMetric]
    oh: MetricHelper

    def __init__(self) -> None:
        self.__topic_oracle_mapping = defaultdict(lambda: list())
        self.__registered_oracles = list()
        self.oh = MetricHelper()

    def register_oracle(self, oracle: BasicMetric):
        self.__registered_oracles.append(oracle)
        oracle.set_oh(self.oh)
        for topic in oracle.get_interested_topics():
            self.__topic_oracle_mapping[topic].append(oracle)

    def on_new_message(self, topic, message, t):
        if topic == '/planning/mission_planning/route':
            self.oh.set_routing_plan(message)
        if topic == '/localization/kinematic_state':
            self.oh.add_ego_pose_pt(message.pose.pose.position)
        for oracle in self.__topic_oracle_mapping[topic]:
            oracle.on_new_message(topic, message, t)

    def get_results(self):
        result = list()
        for oracle in self.__registered_oracles:
            if not self.oh.has_routing_plan() or not self.oh.has_enough_ego_poses():
                continue
            result.extend(oracle.get_result())

        return result

    def get_counts_wrt_oracle(self) -> dict:
        result = dict()
        for oracle in self.__registered_oracles:
            result[oracle.__class__.__name__] = len(oracle.get_result())
        return result
