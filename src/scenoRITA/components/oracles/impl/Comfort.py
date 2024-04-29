from datetime import datetime
from itertools import groupby
from typing import List, Tuple
from dataclasses import dataclass
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped

from scenoRITA.components.oracles.BasicMetric import BasicMetric
from scenoRITA.components.oracles.Violation import Violation
from autoware.utils import calculate_velocity, calculate_time_delta, calculate_accel


@dataclass
class AutowareLocalization:
    pose_with_velocity: Odometry
    accl: AccelWithCovarianceStamped

    def get_linear_velocity(self) -> float:
        return calculate_velocity(self.pose_with_velocity.twist.twist.linear)

    def get_linear_acceleration(self) -> float:
        return calculate_accel(self.accl.accel.accel.linear)


class Comfort(BasicMetric):
    pose_with_velocities: List[Tuple[float, Odometry]]
    accelerations: List[Tuple[float, AccelWithCovarianceStamped]]
    violations: List[Violation]

    MAX_ACCL = 4.0
    MAX_DCCL = -4.0
    TOLERANCE = 0.0
    MINIMUM_DURATION = 0.0

    def __init__(self):
        super().__init__()
        self.fitness = [0.0, 0.0]  # [max_accel, max_decel]
        self.pose_with_velocities = []
        self.accelerations = []
        self.violations = []
        self.trace = list()

    def get_interested_topics(self):
        return [
            '/localization/acceleration',
            '/localization/kinematic_state'
        ]

    def on_new_message(self, topic: str, message, t):
        if topic == '/localization/acceleration':
            self.accelerations.append((t, message))
        elif topic == '/localization/kinematic_state':
            self.pose_with_velocities.append((t, message))
        else:
            raise ValueError('Invalid topic')

    def merge_localizations(self) -> List[AutowareLocalization]:
        pose_w_vel = self.pose_with_velocities
        accel = self.accelerations

        faster = accel if pose_w_vel[0][0] > accel[0][0] else pose_w_vel
        later = pose_w_vel if faster == accel else accel

        offset = 0
        TIME_DELTA_THRESHOLD_IN_NANOSEC = 200000
        for (idx, (t, msg)) in enumerate(faster):
            delta = calculate_time_delta(msg.header.stamp, later[0][1].header.stamp)
            if abs(delta) < TIME_DELTA_THRESHOLD_IN_NANOSEC:
                offset = idx
                break

        result = []
        for (idx, (t, msg)) in enumerate(faster):
            if idx + offset < len(later):
                if isinstance(msg, AccelWithCovarianceStamped):
                    result.append((t, AutowareLocalization(pose_with_velocity=later[idx + offset][1], accl=msg)))
                else:
                    result.append((t, AutowareLocalization(pose_with_velocity=msg, accl=later[idx + offset][1])))

        return result

    def get_result(self):
        if len(self.pose_with_velocities) == 0 or len(self.accelerations) == 0:
            return []

        localizations = self.merge_localizations()

        for (_, prev_), (t2, next_) in zip(localizations, localizations[1:]):
            accel_value = next_.get_linear_acceleration()

            prev_velocity = prev_.get_linear_velocity()
            next_velocity = next_.get_linear_velocity()
            direction = next_velocity - prev_velocity

            accel = accel_value * -1. if direction < 0 else accel_value
            if direction >= 0:
                self.fitness[0] = max(self.fitness[0], accel)
            else:
                self.fitness[1] = min(self.fitness[1], accel)
            features = self.get_basic_info_from_localization(next_.pose_with_velocity)
            features['accel'] = accel
            if accel > Comfort.MAX_ACCL * (1 + Comfort.TOLERANCE):
                self.trace.append((1, t2, features))
            elif accel < Comfort.MAX_DCCL * (1 + Comfort.TOLERANCE):
                self.trace.append((-1, t2, features))
            else:
                self.trace.append((0, t2, None))

        violations = list()
        for k, v in groupby(self.trace, key=lambda x: x[0]):
            traces = list(v)
            start_time = datetime.fromtimestamp(traces[0][1] / 1000000000)
            end_time = datetime.fromtimestamp(traces[-1][1] / 1000000000)
            delta_t = (end_time - start_time).total_seconds()
            if delta_t <= Comfort.MINIMUM_DURATION:
                continue
            if k == 1:
                features = dict(traces[0][2])
                features['duration'] = delta_t
                features['type'] = 1.0
                violations.append(Violation(
                    'FastAccel',
                    features,
                    str(features['accel'])
                ))
            elif k == -1:
                features = dict(traces[0][2])
                features['duration'] = delta_t
                features['type'] = -1.0
                violations.append(Violation(
                    'HardBraking',
                    features,
                    str(features['accel'])
                ))

        return violations

    def get_fitness(self):
        return self.fitness
