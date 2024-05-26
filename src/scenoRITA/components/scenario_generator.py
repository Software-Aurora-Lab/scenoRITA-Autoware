import math
import random
from typing import List, Tuple, Optional, Dict, Set

from shapely.geometry import Point, Polygon

from autoware.map_service import MapService
from geometry_msgs.msg import Point

from autoware.open_scenario import OpenScenario
from autoware.utils import generate_adc_polygon, generate_polygon, get_s_from_start_lst, get_rounded_rand, \
    get_obstacle_specific_type, get_obstacle_type

from ..representation import (
    EgoCar,
    Obstacle,
    ObstacleMotion,
    ObstaclePosition,
    ObstacleType, PositionEstimate,
)

ObstacleConstraints = {
    ObstacleType.CAR: {
        "speed": (2.0, 14.0),
        "width": (1.8, 1.8),
        "length": (4.0, 4.0),
        "height": (2.5, 2.5),
    },
    ObstacleType.BUS: {
        "speed": (2.0, 14.0),
        "width": (2.5, 2.5),
        "length": (12.0, 12.0),
        "height": (2.5, 2.5),
    },
    ObstacleType.TRUCK: {
        "speed": (2.0, 14.0),
        "width": (2.5, 2.5),
        "length": (8.4, 8.4),
        "height": (2.5, 2.5),
    },
    ObstacleType.MOTORCYCLE: {
        "speed": (2.0, 14.0),
        "width": (0.8, 0.8),
        "length": (2.2, 2.2),
        "height": (2.5, 2.5)
    },
    ObstacleType.BICYCLE: {
        "speed": (1.6, 8.3),
        "width": (0.8, 0.8),
        "length": (2.0, 2.0),
        "height": (2.5, 2.5),
    },
    ObstacleType.PEDESTRIAN: {
        "speed": (1.25, 2.9),
        "width": (0.8, 0.8),
        "length": (0.8, 0.8),
        "height": (2.0, 2.0),
    },
}


class ScenarioGenerator:
    def __init__(self, map_service: MapService) -> None:
        self.map_service = map_service
        self.obs_types = {"vehicle", "bicycle", "pedestrian"}
        self.obs_avail_lids: Dict[str, Set[int]] = {_type: set(self.map_service.get_avail_lanes(_type)) for _type in
                                                    self.obs_types}

    def generate_obstacle_route(self, obs_type: ObstacleType, ego_car: EgoCar, initial_lane_id: int = -1) -> Tuple[
        Optional[ObstaclePosition], Optional[ObstaclePosition]]:
        while True:
            _t = get_obstacle_type(obs_type)  # "vehicle" or "bicycle" or "pedestrian"
            if _t not in self.obs_types:
                # logger.info("No available routes for obstacle type {}", obs_type)
                if self.obs_avail_lids.__contains__(_t):
                    del self.obs_avail_lids[_t]
                return None, None
            candidate_lanes = self.obs_avail_lids[_t]
            if not candidate_lanes:
                self.obs_types.remove(_t)
                # logger.info("No available routes for obstacle type {}", _t)
                return None, None

            if initial_lane_id == -1 or initial_lane_id not in self.obs_avail_lids[_t]:
                cons_a = self.map_service.get_nearest_lanes_with_range(ego_car.initial_position.lane_id,
                                                                       ego_car.initial_position.s, 100.0)
                cons_b = self.map_service.get_nearest_lanes_with_range(ego_car.final_position.lane_id,
                                                                    ego_car.final_position.s, 100.0)
                candidate_lanes = (set(cons_a) | set(cons_b)) & (self.obs_avail_lids[_t])
                if not candidate_lanes:
                    initial_lane_id = random.choice(list(self.obs_avail_lids[_t]))
                else:
                    initial_lane_id = random.choice(list(candidate_lanes))

            reachable_lanes_wo_lc = self.map_service.get_reachable_descendants(initial_lane_id,
                                                                               _t, allow_lane_change=False)
            reachable_lanes_wo_lc_wo_self = reachable_lanes_wo_lc - {initial_lane_id}

            if len(reachable_lanes_wo_lc_wo_self) > 0:
                final_lane_id = random.choice(list(reachable_lanes_wo_lc))
                initial_central_curve = self.map_service.get_center_line_lst_by_id(
                    initial_lane_id
                )
                initial_xes, _ = initial_central_curve.xy
                initial_index = random.randint(0, len(initial_xes) - 2)

                final_central_curve = self.map_service.get_center_line_lst_by_id(
                    final_lane_id
                )
                final_xes, _ = final_central_curve.xy
                final_index = random.randint(0, len(final_xes) - 2) # fixme: if the final lane is the same as the initial lane, the final index should be greater than the initial index

                return (
                    ObstaclePosition(initial_lane_id, initial_index,
                                     get_s_from_start_lst(initial_central_curve, initial_index)),
                    ObstaclePosition(final_lane_id, final_index,
                                     get_s_from_start_lst(final_central_curve, final_index))
                )
            else:
                # reachable_lanes_wo_lc == {initial_lane_id}
                init_cc = self.map_service.get_center_line_lst_by_id(initial_lane_id)
                xes, _ = init_cc.xy
                final_ind = random.randint(1, len(xes) - 1)
                return (ObstaclePosition(initial_lane_id, 0, 0.5),
                        ObstaclePosition(initial_lane_id, final_ind, get_s_from_start_lst(init_cc, final_ind)))

    def get_obs_shortest_pth(self, obs_type: ObstacleType, src_id: int, tgt_id: int):
        top_type = get_obstacle_type(obs_type)
        match top_type:
            case "vehicle":
                return self.map_service.get_vehicle_shortest_path_src_tgt(src_id, tgt_id, False)
            case "bicycle":
                return self.map_service.get_bicycle_shortest_path_src_tgt(src_id, tgt_id, False)
            case "pedestrian":
                return self.map_service.get_pedestrian_shortest_path_src_tgt(src_id, tgt_id, False)
            case _:
                raise NotImplementedError(f"Unknown obstacle type {obs_type}")

    def generate_obstacle_type(self) -> ObstacleType:
        top_type = random.choice(list(self.obs_types))  # choose vehicle/pedestrian/bicycle
        specific_type = random.choice(get_obstacle_specific_type(top_type))  # choose subtype
        return specific_type

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

    def generate_obstacle(self, ego_car: EgoCar, obs_type: ObstacleType = None) -> Obstacle:
        """
        Generate an obstacle with random properties but avoid
            being initialized overlapping with the ego car.
        :param ego_car: The ego car.
        :param obs_type: The obstacle type.
        :return: The generated obstacle.
        """
        obs_type = self.generate_obstacle_type() if obs_type is None else obs_type
        width, length, height = self.generate_obstacle_dimensions(obs_type)  # generate a random obstacle

        # Avoid generating obstacles that overlap with the ego car.
        while True:
            initial, final = self.generate_obstacle_route(obs_type, ego_car)
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
                final_lane_cg_neighbours = self.map_service.get_changable_neighbours(final_lane_id)
                if len(final_lane_cg_neighbours) != 0:
                    if random.random() < 0.8:
                        # print("randomly choosing a lane from the changeable neighbours")
                        candidate = random.choice(final_lane_cg_neighbours)
                        if self.map_service.get_length_of_lane(candidate) >= k_min_lane_length:
                            final_lane_id = candidate

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
                descendants = self.map_service.get_reachable_descendants(lane_id)
                if len(descendants) > 0:
                    target_lane_id = random.choice(list(descendants))
                    return EgoCar(
                        PositionEstimate(
                            lane_id, min(1.5, lane_length)
                        ),  # at the end of the lane
                        PositionEstimate(
                            target_lane_id,
                            self.map_service.get_length_of_lane(target_lane_id),
                        ),
                    )
                options.remove(lane_id)  # if this lane cannot reach any other lane, remove it from the options

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
        path = self.get_obs_shortest_pth(obs_type, initial.lane_id, final.lane_id)
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
