from datetime import datetime
from itertools import groupby
from typing import List
from scenoRITA.components.oracles.BasicMetric import BasicMetric
from scenoRITA.components.oracles.Violation import Violation
from autoware.utils import calculate_velocity


class Speeding(BasicMetric):
    MINIMUM_DURATION = 0.0
    TOLERANCE = 0.1

    def __init__(self):
        super().__init__()
        self.speed_limits = self.map_service.get_speed_limits()
        self.obs_fitness = float("inf")
        self.trace = list()

    def get_interested_topics(self) -> List[str]:
        return ['/localization/kinematic_state']

    def on_new_message(self, topic: str, message, t):
        ego_position = message.pose.pose.position
        ego_velocity = calculate_velocity(message.twist.twist.linear)

        if not self.mh.has_routing_plan():
            return

        current_lane = self.map_service.get_current_lanelet(ego_position)[0]
        if current_lane is None:
            self.trace.append((False, t, -1, dict()))
        else:
            lane_speed_limit = float(current_lane.attributes['speed_limit'])
            self.obs_fitness = min(self.obs_fitness, lane_speed_limit - ego_velocity)
            if ego_velocity > lane_speed_limit * (1 + Speeding.TOLERANCE):
                features = self.get_basic_info_from_localization(message)
                features['speed_limit'] = lane_speed_limit
                features['lane_id'] = current_lane.id
                self.trace.append((True, t, lane_speed_limit, features))
            else:
                self.trace.append((False, t, -1, dict()))

    def get_result(self):
        violations = list()
        for k, v in groupby(self.trace, key=lambda x: (x[0], x[2])):
            traces = list(v)
            start_time = datetime.fromtimestamp(traces[0][1] / 1000000000)
            end_time = datetime.fromtimestamp(traces[-1][1] / 1000000000)
            delta_t = (end_time - start_time).total_seconds()

            if k[0]:
                features = dict(traces[0][3])
                features['duration'] = delta_t
                violations.append(
                    Violation(
                        'Speeding',
                        features,
                        str(features['speed'])
                    )
                )

        return violations

    def get_fitness(self):
        return self.obs_fitness
