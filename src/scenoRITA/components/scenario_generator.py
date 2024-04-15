import math
import random
from typing import List, Tuple, Optional

from shapely.geometry import Point, Polygon

from autoware.map_service import MapService, PositionEstimate
from geometry_msgs.msg import Point

from autoware.open_scenario import OpenScenario
from autoware.utils import generate_adc_polygon, generate_polygon, get_s_from_start_lst, get_rounded_rand

from ..representation import (
    EgoCar,
    Obstacle,
    ObstacleMotion,
    ObstaclePosition,
    ObstacleType,
    Scenario,
)

ObstacleConstraints = {
    ObstacleType.VEHICLE: {
        "speed": (2.0, 14.0),
        "width": (1.5, 2.5),
        "length": (4.0, 14.5),
        "height": (1.5, 4.7),
    },
    ObstacleType.BICYCLE: {
        "speed": (1.6, 8.3),
        "width": (0.5, 1.0),
        "length": (1.0, 2.5),
        "height": (1.0, 2.5),
    },
    ObstacleType.PEDESTRIAN: {
        "speed": (1.25, 2.9),
        "width": (0.24, 0.67),
        "length": (0.2, 0.45),
        "height": (0.97, 1.87),
    },
}


# todo: leave or delete
# def cut(line: LineString, distance: float):
#     # Cuts a line in two at a distance from its starting point
#     if distance <= 0.0 or distance >= line.length:
#         return [LineString(line)]
#     coords = list(line.coords)
#     for i, p in enumerate(coords):
#         pd = line.project(Point(p))
#         if pd == distance:
#             return [LineString(coords[: i + 1]), LineString(coords[i:])]
#         if pd > distance:
#             cp = line.interpolate(distance)
#             return [
#                 LineString(coords[:i] + [(cp.x, cp.y)]),
#                 LineString([(cp.x, cp.y)] + coords[i:]),
#             ]
#     return [LineString(line)]


class ScenarioGenerator:
    def __init__(self, map_service: MapService) -> None:
        self.map_service = map_service
        self.obs_types = {ObstacleType.VEHICLE, ObstacleType.BICYCLE, ObstacleType.PEDESTRIAN}
        self.obs_avail_lids = {_type: set(self.get_avail_lanes(_type)) for _type in self.obs_types}

    def generate_obstacle_route(self, obs_type) -> Tuple[Optional[ObstaclePosition], Optional[ObstaclePosition]]:
        while True:
            candidate_lanes = self.get_avail_lanes(obs_type)
            if not candidate_lanes:
                self.obs_types.remove(obs_type)
                return None, None
            initial_lane_id = random.choice(list(self.obs_avail_lids[obs_type]))

            paths = self.get_shortest_pth_filtered(obs_type, initial_lane_id)
            if len(paths) > 1:
                chosen_path = random.choice(paths)
                print("pth length", len(chosen_path))
                initial_central_curve = self.map_service.get_center_line_lst_by_id(
                    initial_lane_id
                )
                initial_xes, _ = initial_central_curve.xy
                initial_index = random.randint(0, len(initial_xes) - 2)

                final_lane_id = chosen_path[-1].id
                final_central_curve = self.map_service.get_center_line_lst_by_id(
                    final_lane_id
                )
                final_xes, _ = final_central_curve.xy
                final_index = random.randint(0, len(final_xes) - 2)

                return (
                    ObstaclePosition(initial_lane_id, initial_index,
                                     get_s_from_start_lst(initial_central_curve, initial_index)),
                    ObstaclePosition(final_lane_id, final_index,
                                     get_s_from_start_lst(final_central_curve, final_index))
                )
            elif len(paths) == 1:
                init_cc = self.map_service.get_center_line_lst_by_id(initial_lane_id)
                xes, _ = init_cc.xy
                final_ind = random.randint(0, len(xes))
                return (ObstaclePosition(initial_lane_id, 0, 0.0),
                        ObstaclePosition(initial_lane_id, final_ind, get_s_from_start_lst(init_cc, final_ind)))
            else:
                self.obs_avail_lids[obs_type].remove(initial_lane_id)

    def get_shortest_pth_filtered(self, obs_type: ObstacleType, src_id: int):
        # todo: 没有必要全部求出来了，直接用reachableSet, 挑一个出来，再单独求就好了
        pth = self.get_shortest_pth(obs_type, src_id)
        match obs_type:
            case ObstacleType.VEHICLE:
                return list(filter(lambda x: x is not None and len(x) > 1, pth.values()))
            case ObstacleType.BICYCLE:
                return list(filter(lambda x: x is not None and len(x) > 0, pth.values()))
            case ObstacleType.PEDESTRIAN:
                return list(filter(lambda x: x is not None and len(x) > 0, pth.values()))
            case _:
                raise NotImplementedError(f"Unknown obstacle type {obs_type}")

    def get_shortest_pth(self, obs_type, src_id, tgt_id=None):
        match obs_type:
            case ObstacleType.VEHICLE:
                if tgt_id:
                    return self.map_service.get_vehicle_shortest_path_src_tgt(src_id, tgt_id)
                else:
                    return self.map_service.get_vehicle_shortest_path_src(src_id)
            case ObstacleType.BICYCLE:
                if tgt_id:
                    return self.map_service.get_bicycle_shortest_path_src_tgt(src_id, tgt_id)
                else:
                    return self.map_service.get_bicycle_shortest_path_src(src_id)
            case ObstacleType.PEDESTRIAN:
                if tgt_id:
                    return self.map_service.get_pedestrian_shortest_path_src_tgt(src_id, tgt_id)
                else:
                    return self.map_service.get_pedestrian_shortest_path_src(src_id)
            case _:
                raise NotImplementedError(f"Unknown obstacle type {obs_type}")

    def generate_obstacle_type(self) -> ObstacleType:
        return random.choice(list(self.obs_types))

    def generate_obstacle_motion(self) -> ObstacleMotion:
        return random.choice([ObstacleMotion.DYNAMIC, ObstacleMotion.STATIC])

    def generate_obstacle_speed(self, obs_type: ObstacleType) -> float:
        return get_rounded_rand(*ObstacleConstraints[obs_type]["speed"])

    def generate_obstacle_dimensions(self, obs_type: ObstacleType) -> Tuple[float, float, float]:
        return (
            get_rounded_rand(*ObstacleConstraints[obs_type]["width"]),
            get_rounded_rand(*ObstacleConstraints[obs_type]["length"]),
            get_rounded_rand(*ObstacleConstraints[obs_type]["height"]),
        )

    def generate_obstacle(self, ego_car: EgoCar) -> Obstacle:
        """
        Generate an obstacle with random properties but avoid
            being initialized overlapping with the ego car.
        :param ego_car: The ego car.
        :return: The generated obstacle.
        """
        obs_type = self.generate_obstacle_type()
        width, length, height = self.generate_obstacle_dimensions(obs_type)  # generate a random obstacle

        # Avoid generating obstacles that overlap with the ego car.
        while True:
            initial, final = self.generate_obstacle_route(obs_type)
            if initial is None or final is None:
                obs_type = self.generate_obstacle_type()
                width, length, height = self.generate_obstacle_dimensions(obs_type)
                continue
            ego_initial = ego_car.initial_position

            ep, et = self.map_service.get_lane_coord_and_heading(
                ego_initial.lane_id, ego_initial.s
            )
            ego_polygon_pts = generate_adc_polygon(Point(x=ep.x, y=ep.y, z=0.0), et)
            ego_polygon = Polygon([[x.x, x.y] for x in ego_polygon_pts])

            obs_x, obs_y = self.generate_obs_reference_path(initial, final, obs_type)
            obs_theta = math.atan2(obs_y[1] - obs_y[0], obs_x[1] - obs_x[0])

            obs_ply_pts = generate_polygon(Point(x=obs_x[0], y=obs_y[0], z=0.0), obs_theta, length, width)
            obstacle_polygon = Polygon([[x.x, x.y] for x in obs_ply_pts])

            if not ego_polygon.intersects(obstacle_polygon):
                break

        result = Obstacle(
            id=random.randint(100000, 999999),
            initial_position=initial,
            final_position=final,
            type=obs_type,
            speed=self.generate_obstacle_speed(obs_type),
            width=width,
            length=length,
            height=height,
            motion=self.generate_obstacle_motion(),
        )

        return result

    def generate_ego_car(self) -> EgoCar:
        generate_junction_scenario = random.random() < 0.9
        if generate_junction_scenario:
            junction_lanes = self.map_service.get_junction_lanes()
            while True:
                chosen_junction_lane = random.choice(junction_lanes)
                predecessors = self.map_service.get_predecessors_for_lane(
                    chosen_junction_lane
                )
                successors = self.map_service.get_successors_for_lane(
                    chosen_junction_lane
                )
                if len(predecessors) == 0 or len(successors) == 0:
                    junction_lanes.remove(chosen_junction_lane)
                    continue

                has_valid_predecessor = False
                k_min_lane_length = 5.0
                initial_lane_id = -1
                while True:
                    if len(predecessors) == 0:
                        break
                    initial_lane_id = random.choice(predecessors)
                    predecessors.remove(initial_lane_id)
                    if self.map_service.get_length_of_lane(initial_lane_id) >= k_min_lane_length:
                        has_valid_predecessor = True
                        break

                if not has_valid_predecessor:
                    junction_lanes.remove(chosen_junction_lane)
                    continue

                final_lane_id = random.choice(successors)
                # assert initial_lane_id != -1
                initial_lane_length = self.map_service.get_length_of_lane(
                    initial_lane_id
                )
                return EgoCar(
                    PositionEstimate(
                        initial_lane_id, max(1.5, initial_lane_length - 10.0)
                    ),
                    PositionEstimate(
                        final_lane_id,
                        self.map_service.get_length_of_lane(final_lane_id),
                    ),
                )
        else:
            options = self.map_service.get_non_junction_lanes()
            while True:
                lane_id = random.choice(options)
                lane_length = self.map_service.get_length_of_lane(lane_id)
                descendants = self.map_service.find_descendants(lane_id)
                if len(descendants) > 0:
                    target_lane_id = random.choice(list(descendants))
                    return EgoCar(
                        PositionEstimate(
                            lane_id, min(1.5, lane_length)
                        ),  # at the end of the lane
                        PositionEstimate(
                            target_lane_id,
                            self.map_service.get_length_of_lane(lane_id),
                        ),
                    )
                options.remove(lane_id)

    # todo: check the functionality
    def generate_scenario(
            self, gen_id: int, sce_id: int, min_obs: int, max_obs: int
    ) -> OpenScenario:
        num_obs = random.randint(min_obs, max_obs)
        ego_car = self.generate_ego_car()
        return OpenScenario(
            generation_id=gen_id,
            scenario_id=sce_id,
            ego_car=ego_car,
            obstacles=[self.generate_obstacle(ego_car) for _ in range(num_obs)],
            map_name=self.map_service.map_name,
        )

    def generate_obs_reference_path(
            self, initial: ObstaclePosition, final: ObstaclePosition, obs_type: ObstacleType
    ) -> Tuple[List[float], List[float]]:
        path = self.get_shortest_pth(obs_type, initial.lane_id, final.lane_id)
        x_es = []
        y_es = []
        for lane in path:
            central_curve = self.map_service.get_center_line_lst_by_id(lane.id)
            if lane.id == initial.lane_id:
                x_es.extend(central_curve.xy[0][initial.index:])
                y_es.extend(central_curve.xy[1][initial.index:])
            else:
                if central_curve.xy[0][0] == x_es[-1]:
                    # The first point of the central curve is the
                    # same as the last point of the previous curve.
                    x_es.extend(central_curve.xy[0][1:])
                    y_es.extend(central_curve.xy[1][1:])
                else:
                    x_es.extend(central_curve.xy[0])
                    y_es.extend(central_curve.xy[1])
        return x_es, y_es

    def get_avail_lanes(self, obs_type: ObstacleType):
        match obs_type:
            case ObstacleType.VEHICLE:
                return self.map_service.get_vehicle_lanes()
            case ObstacleType.BICYCLE:
                return self.map_service.get_bicycle_lanes()
            case ObstacleType.PEDESTRIAN:
                return self.map_service.get_pedestrian_lanes()
            case _:
                raise NotImplementedError("Unknown obstacle type")

# todo:
# def generate_traffic_light_detection_msgs(
#     self, scenario_length: int, frequency: int, reference_time: float
# ) -> List[TrafficLightDetection]:
#     result: List[TrafficLightDetection] = list()
#     dt = 1.0 / frequency
#     current_time = 0.0
#     sequence_num = 0
#     while current_time < scenario_length:
#         traffic_light_detection = TrafficLightDetection(
#             header=Header(
#                 timestamp_sec=reference_time + current_time,
#                 module_name=PROJECT_NAME,
#                 sequence_num=sequence_num,
#             )
#         )
#         for signal_id in self.map_service.signal_table.keys():
#             detection = traffic_light_detection.traffic_light.add()
#             detection.id = signal_id
#             detection.color = TrafficLight.GREEN
#             detection.confidence = 1.0
#         result.append(traffic_light_detection)
#         current_time += dt
#         sequence_num += 1
#     return result

# def generate_perception_obstacle(
#     self,
#     obstacle: Obstacle,
#     scenario_length: int,
#     frequency: int,
#     reference_time: float,
# ) -> List[PerceptionObstacle]:
#     rx, ry = self.generate_obs_reference_path(
#         obstacle.initial_position, obstacle.final_position
#     )
#     reference_linestring = LineString([(x, y) for x, y in zip(rx, ry)])
#     k_max_reference_length = obstacle.speed * scenario_length  # meters
#     if reference_linestring.length > k_max_reference_length:
#         the_cut = cut(reference_linestring, k_max_reference_length)
#         rx, ry = the_cut[0].xy
#
#     planner_sample_distance = 0.5
#     cx, cy, cyaw, ck, _ = cubic_spline_planner.calc_spline_course(
#         rx, ry, ds=planner_sample_distance
#     )
#     state = stanley_controller.State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)
#     last_idx = len(cx) - 1
#
#     x = [state.x]
#     y = [state.y]
#     yaw = [state.yaw]
#     v = [state.v]
#     t = [0.0]
#     if obstacle.motion == ObstacleMotion.DYNAMIC:
#         target_speed = [obstacle.speed for _ in range(len(cx))]
#         for i in range(len(target_speed)):
#             if abs(ck[i]) > 0.1:
#                 target_speed[i] = min(target_speed[i], 4.0)
#     else:
#         target_speed = [0.0 for _ in range(len(cx))]
#     L = obstacle.length
#     target_idx, _ = stanley_controller.calc_target_index(state, cx, cy, L)
#     dt = 1.0 / frequency
#     current_time = 0.0
#
#     # while current_time < scenario_length and last_idx > target_idx:
#     for i in range(int(scenario_length * frequency)):
#         di, target_idx = stanley_controller.stanley_control(
#             state, cx, cy, cyaw, target_idx, L
#         )
#         if target_idx == last_idx:
#             # reached to goal, stop
#             current_time += dt
#             x.append(x[-1])
#             y.append(y[-1])
#             yaw.append(yaw[-1])
#             v.append(0.0)
#             t.append(current_time)
#         else:
#             # have not reached to goal, keep going
#             # distance needed to decelerate to zero
#             # v^2 = u^2 + 2as
#             # 2as = v^2 - u^2
#             # s = (v^2 - u^2) / 2a
#             k_decelerate_distance = obstacle.speed**2 / (2 * 4.0)
#             k_decelerate_index = int(
#                 k_decelerate_distance / planner_sample_distance
#             )
#             if last_idx - target_idx < k_decelerate_index:
#                 prev_v = v[-1]
#                 target_v = round(
#                     target_speed[target_idx]
#                     * (last_idx - target_idx)
#                     / k_decelerate_index,
#                     2,
#                 )
#                 target_speed[target_idx] = min(prev_v, target_v)
#             ai = stanley_controller.pid_control(target_speed[target_idx], state.v)
#             state.update(ai, di, L, dt)
#             current_time += dt
#
#             x.append(state.x)
#             y.append(state.y)
#             yaw.append(state.yaw)
#             v.append(state.v)
#             t.append(current_time)
#
#     obstacle_messages: List[PerceptionObstacle] = list()
#     for i in range(len(x)):
#         _velocity = Point3D(
#             x=math.cos(yaw[i]) * v[i], y=math.sin(yaw[i]) * v[i], z=0.0
#         )
#         delta_v = v[i] - v[i - 1] if i > 0 else 0.0
#         delta_t = t[i] - t[i - 1] if i > 0 else 0.0
#         if delta_t == 0.0:
#             _acceleration = Point3D(x=0, y=0, z=0)
#         else:
#             _acceleration = Point3D(
#                 x=math.cos(yaw[i]) * delta_v / delta_t,
#                 y=math.sin(yaw[i]) * delta_v / delta_t,
#                 z=0.0,
#             )
#         polygon_points = [
#             Point3D(x=p[0], y=p[1], z=0.0)
#             for p in generate_polygon(
#                 x[i], y[i], 0.0, yaw[i], obstacle.length, obstacle.width
#             )
#         ]
#         obs_type = self.get_perception_obstacle_type(obstacle.type)
#         obstacle_messages.append(
#             PerceptionObstacle(
#                 id=obstacle.id,
#                 position=Point3D(x=x[i], y=y[i], z=0.0),
#                 theta=yaw[i],
#                 length=obstacle.length,
#                 width=obstacle.width,
#                 height=obstacle.height,
#                 velocity=_velocity,
#                 acceleration=_acceleration,
#                 type=obs_type,
#                 timestamp=reference_time + t[i],
#                 tracking_time=1.0,
#                 polygon_point=polygon_points,
#             )
#         )
#     return obstacle_messages

# def generate_perception_obstacles_msgs(
#     self,
#     obstacles: List[Obstacle],
#     scenario_length: int,
#     frequency: int,
#     reference_time: float,
# ) -> List[PerceptionObstacles]:
#     obstacle_messages: List[List[PerceptionObstacle]] = list()
#     for obs in obstacles:
#         obstacle_messages.append(
#             self.generate_perception_obstacle(
#                 obs, scenario_length, frequency, reference_time
#             )
#         )
#
#     msg_length = len(obstacle_messages[0])
#     result = list()
#     for i in range(msg_length):
#         new_msg = PerceptionObstacles(
#             header=Header(
#                 timestamp_sec=obstacle_messages[0][i].timestamp,
#                 module_name=PROJECT_NAME,
#                 sequence_num=i,
#             ),
#             perception_obstacle=[p[i] for p in obstacle_messages],
#         )
#         result.append(new_msg)
#     return result
