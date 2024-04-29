from typing import List, Optional
from datetime import datetime
from itertools import groupby
from shapely.geometry import Polygon
from scenoRITA.components.oracles.Violation import Violation
from scenoRITA.components.oracles.BasicMetric import BasicMetric
from autoware.utils import generate_adc_polygon, quaternion_2_heading, get_real_time_from_msg
from functools import wraps
import time

from scenoRITA.components.oracles.OracleInterrupt import OracleInterrupt


def timeit(func):
    @wraps(func)
    def timeit_wrapper(*args, **kwargs):
        start = time.perf_counter()
        result = func(*args, **kwargs)
        end = time.perf_counter()
        total = end - start
        # print(func.__name__, total)
        return result

    return timeit_wrapper


class UnsafeLaneChange(BasicMetric):
    MINIMUM_DURATION = 5.0
    PRUNE_DISTANCE = 350
    FAST = False

    def __init__(self):
        super().__init__()
        self.count = 0
        self.boundaries = self.map_service.get_lane_boundaries()
        self.boundary_ids = sorted(self.boundaries.keys())
        self.__data = list()
        self.cached_data = dict()
        self.fitness: Optional[float] = None

        self.searchable_boundary_ids = set(self.boundary_ids)

    @timeit
    def on_new_message(self, topic: str, message, t):
        if not self.mh.has_routing_plan():
            return
        self.count += 1
        if UnsafeLaneChange.FAST and self.count % 15 != 0:
            return
        t = get_real_time_from_msg(message.header)
        position = message.pose.pose.position
        if (position.x, position.y) in self.cached_data:
            bid = self.cached_data[(position.x, position.y)]
            features = self.get_basic_info_from_localization(message)
            if bid == '':
                self.__data.append((False, t, '', {}))
            elif features['speed'] > 0:
                features['boundary_id'] = self.boundary_ids.index(bid)
                self.__data.append((True, t, bid, features))
            return
        ego_heading = quaternion_2_heading(message.pose.pose.orientation)

        ego_pts = generate_adc_polygon(message.pose.pose.position, ego_heading)
        ego_polygon = Polygon([[x.x, x.y] for x in ego_pts])
        pending_removal_boundary_ids = set()
        for index, bid in enumerate(self.searchable_boundary_ids):
            distance = ego_polygon.distance(self.boundaries[bid])
            if distance == 0:
                # intersection found
                features = self.get_basic_info_from_localization(message)
                features['boundary_id'] = self.boundary_ids.index(bid)
                if features["speed"] > 0:
                    self.__data.append((True, t, bid, features))
                    self.cached_data[(position.x, position.y)] = bid
                else:
                    self.__data.append((False, t, '', {}))
                return
            if distance > UnsafeLaneChange.PRUNE_DISTANCE:
                pending_removal_boundary_ids.add(bid)
        self.searchable_boundary_ids = self.searchable_boundary_ids - pending_removal_boundary_ids
        # no intersection
        self.__data.append((False, t, '', {}))
        self.cached_data[(position.x, position.y)] = ''

    def get_interested_topics(self) -> List[str]:
        return ["/localization/kinematic_state"]

    def get_result(self):
        self.fitness = 0.0
        violations = list()
        for k, v in groupby(self.__data, key=lambda x: (x[0], x[2])):
            intersects, b_id = k
            traces = list(v)
            if intersects and len(traces) > 1:
                start_time = datetime.fromtimestamp(traces[0][1] / 1000000000)
                end_time = datetime.fromtimestamp(traces[-1][1] / 1000000000)
                delta_t = (end_time - start_time).total_seconds()

                self.fitness = max(self.fitness, delta_t)

                if delta_t > self.MINIMUM_DURATION:
                    features = dict(traces[0][3])
                    features['duration'] = delta_t
                    violations.append(
                        Violation(
                            'UnsafeLaneChange',
                            features,
                            str(features['boundary_id'])
                        )
                    )
        return violations

    def get_fitness(self):
        assert OracleInterrupt("Fitness is not calculated yet.")
        return self.fitness
