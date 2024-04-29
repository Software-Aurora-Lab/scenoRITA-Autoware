from collections import defaultdict
from typing import List, Optional, Tuple, Dict

from autoware_auto_perception_msgs.msg import PredictedObjects, PredictedObject
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from scenoRITA.components.oracles.BasicMetric import BasicMetric
from shapely.geometry import Polygon, LineString, Point
from scenoRITA.components.oracles.Violation import Violation
from autoware.utils import generate_adc_polygon, quaternion_2_heading, obstacle_to_polygon, calculate_velocity, obs_hash
from scenoRITA.components.oracles.OracleInterrupt import OracleInterrupt


class Collision(BasicMetric):
    last_localization: Optional[Odometry]
    last_perception: Optional[PredictedObjects]
    distances: List[Tuple[float, int, str]]

    def __init__(self):
        super().__init__()
        self.last_localization = None
        self.last_perception = None
        self.excluded_obs = set()
        self.violations = list()
        self.distance_traveled = 0.0

        self.obs_fitness: Dict[int, float] = defaultdict(lambda: float("inf"))

    def get_interested_topics(self):
        return [
            '/localization/kinematic_state',
            '/perception/object_recognition/objects'
        ]

    def on_new_message(self, topic: str, message, t):
        if topic == '/localization/kinematic_state':
            if self.last_localization is not None and self.mh.has_routing_plan():
                prev_point = Point(self.last_localization.pose.pose.position.x,
                                   self.last_localization.pose.pose.position.y,
                                   0.0)
                cur_point = Point(message.pose.pose.position.x,
                                  message.pose.pose.position.y,
                                  0.0)
                self.distance_traveled += prev_point.distance(cur_point)

            self.last_localization = message
        else:
            self.last_perception = message

        if self.last_localization is None or self.last_perception is None:
            return

        adc_pose: Pose = self.last_localization.pose.pose

        adc_heading = quaternion_2_heading(adc_pose.orientation)
        adc_polygon_pts = generate_adc_polygon(adc_pose.position, adc_heading)
        adc_polygon = Polygon([[x.x, x.y] for x in adc_polygon_pts])

        adc_front_line_string = LineString(
            [[x.x, x.y] for x in (adc_polygon_pts[0], adc_polygon_pts[3])])
        adc_rear_line_string = LineString(
            [[x.x, x.y] for x in (adc_polygon_pts[1], adc_polygon_pts[2])])

        collision_detected = False

        objs = self.last_perception.objects
        for obs in objs:
            obs_id = obs_hash(obs.shape.dimensions.x, obs.shape.dimensions.y, obs.shape.dimensions.z)
            if obs_id in self.excluded_obs:
                continue

            obs_polygon = obstacle_to_polygon(obs)

            obs_polygon_area = obs_polygon.area

            # rear-end collision occurred (obs is behind the ADC)
            if obs_polygon.distance(adc_rear_line_string) == 0.0:
                if self.distance_traveled == 0.0:
                    continue
                self.excluded_obs.add(obs_id)
                self.obs_fitness[obs_id] = float("inf")
                collision_detected = True
                continue

            distance = adc_front_line_string.distance(obs_polygon)
            self.obs_fitness[obs_id] = min(distance, self.obs_fitness[obs_id])

            # front-end collision occurred (obs is in front of the ADC)
            if distance == 0.0 and calculate_velocity(self.last_localization.twist.twist.linear) > 0.000:
                if self.distance_traveled == 0.0:
                    continue

                obs_lane = self.map_service.get_nearest_lanes_w_range(self.last_localization.pose.pose, 10)
                obs_in_lane = False
                for lane in obs_lane:
                    lb, rb = self.map_service.get_lane_boundaries_by_id(lane.id)

                    lx, ly = lb.xy
                    rx, ry = rb.xy
                    x_es = lx + rx[::-1]
                    y_es = ly + ry[::-1]
                    lane_polygon = Polygon([[x, y] for x, y in zip(x_es, y_es)])

                    if not lane_polygon.intersects(obs_polygon):
                        continue

                    intersection_area = lane_polygon.intersection(obs_polygon).area
                    if abs(intersection_area - obs_polygon_area) < 1e-3:
                        obs_in_lane = True
                        break

                # add violation if obs is in lane
                if obs_in_lane:
                    features = BasicMetric.get_basic_info_from_localization(self.last_localization)
                    features['obs_x'] = obs.kinematics.initial_pose_with_covariance.pose.position.x
                    features['obs_y'] = obs.kinematics.initial_pose_with_covariance.pose.position.y
                    features['obs_heading'] = quaternion_2_heading(
                        obs.kinematics.initial_pose_with_covariance.pose.orientation)
                    features['obs_speed'] = calculate_velocity(
                        obs.kinematics.initial_twist_with_covariance.twist.linear)
                    features['obs_type'] = obs.classification[0].label
                    features['obs_id'] = obs_id
                    self.violations.append(
                        Violation(
                            'Collision',
                            features,
                            str(features['obs_id'])
                        )
                    )
                else:
                    self.obs_fitness[obs_id] = float("inf")
                self.excluded_obs.add(obs_id)
                collision_detected = True
                continue

            # other collision
            if adc_polygon.distance(obs_polygon) == 0.0:
                if self.distance_traveled == 0.0:
                    continue
                collision_detected = True
                self.obs_fitness[obs_id] = float("inf")
                continue

        if collision_detected:
            raise OracleInterrupt()

    def get_result(self):
        return self.violations

    def get_fitness(self):
        return self.obs_fitness
