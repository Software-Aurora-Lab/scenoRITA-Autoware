from typing import List, Optional
from datetime import datetime
from itertools import groupby
from shapely.geometry import Polygon
from scenoRITA.components.oracles.Violation import Violation
from scenoRITA.components.oracles.BasicMetric import BasicMetric
from autoware.utils import generate_adc_polygon, quaternion_2_heading
from functools import wraps
import time


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
    FAST = True

    def __init__(self):
        super().__init__()
        self.count = 0
        self.__data = list()
        self.fitness: Optional[float] = None

    @timeit
    def on_new_message(self, topic: str, message, t):
        if not self.mh.has_routing_plan():
            return
        self.count += 1
        if UnsafeLaneChange.FAST and self.count % 15 != 0:
            return

        ego_position = message.pose.pose.position
        ego_pose = message.pose.pose
        features = self.get_basic_info_from_localization(message)
        ego_heading = quaternion_2_heading(message.pose.pose.orientation)

        ego_pts = generate_adc_polygon(ego_position, ego_heading)
        ego_polygon = Polygon([[x.x, x.y] for x in ego_pts])

        ego_lanes = self.map_service.get_veh_current_lane(ego_pose)
        if len(ego_lanes) == 0:
            self.__data.append((False, t, '', {}))
            return

        if features['speed'] == 0:
            self.__data.append((False, t, '', {}))
            return

        for lane in ego_lanes:
            lb, rb = self.map_service.get_lane_boundaries_by_id(lane.id)
            if not lb.intersects(ego_polygon) and not rb.intersects(ego_polygon):
                self.__data.append((False, t, '', {}))
                return

        lb_1, rb_1 = self.map_service.get_lane_boundaries_by_id(ego_lanes[0].id)
        if lb_1.intersects(ego_polygon):
            bid = self.map_service.get_lane_by_id(ego_lanes[0].id).leftBound.id
        else:
            bid = self.map_service.get_lane_by_id(ego_lanes[0].id).rightBound.id

        features['boundary_id'] = bid
        self.__data.append((True, t, bid, features))

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
        assert self.fitness is not None, "Fitness is not calculated yet."
        return self.fitness
