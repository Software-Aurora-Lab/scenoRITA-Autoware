# NOTES when running scenoRITA

## map_service.py

As for now, we set the traffic rules using the lanelet2 built-in value (Germany). The best way is to define a custom [traffic rule class](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_traffic_rules/README.md) and use it for routing.
```py
        tr_veh = ll2.traffic_rules.create(ll2.traffic_rules.Locations.Germany,
                                          ll2.traffic_rules.Participants.Vehicle)
        tr_ped = ll2.traffic_rules.create(ll2.traffic_rules.Locations.Germany,
                                          ll2.traffic_rules.Participants.Pedestrian)
        tr_bic = ll2.traffic_rules.create(ll2.traffic_rules.Locations.Germany,
                                          ll2.traffic_rules.Participants.Bicycle)

        map_parser = MapService.instance(ll_map=ll2_map,
                                         projector=self.projector,
                                         rg_veh=RoutingGraph(ll2_map, tr_veh),
                                         rg_ped=RoutingGraph(ll2_map, tr_ped),
                                         rg_bic=RoutingGraph(ll2_map, tr_bic),
                                         map_name=map_name)
```

## autoware/utils.py

Define the size of the ego car in `autoware/utils.py`

```py
# VEHICLE CONFIGS FOR AUTOWARE
AUTOWARE_VEHICLE_LENGTH = 4.89
AUTOWARE_VEHICLE_WIDTH = 1.64+0.128*2
AUTOWARE_VEHICLE_HEIGHT = 2.5
AUTOWARE_VEHICLE_back_edge_to_center = 1.1
```

## scenario_generator.py

When converting genetic representation to the input for other simulators (like CARLA), you should create a custom class to replace the `OpenScenario` class.  This custom class, which we defined, is used to convert the representation to the [TIER IV Scenario Format version 2.0](https://tier4.github.io/scenario_simulator_v2-docs/developer_guide/TIERIVScenarioFormatVersion2/)

```py
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
```

As for now, due to the nature of Scenario Simulator v2, we do not alter the sizes for each obstacle type. If you choose another simulator, you can change this code based on the simulator.


```py
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
```

Regarding the generation of the ego car, you can find an explanation in this [note](https://github.com/orgs/autowarefoundation/discussions/4687#discussioncomment-9357644) about why certain lines of code were added.

```py
                final_lane_id = -1
                need_to_cut = True
                while True:
                    if len(successors) == 0:
                        break
                    final_lane_id = random.choice(successors)
                    final_lane_cg_neighbours = self.map_service.get_changable_neighbours(final_lane_id)
                    if len(final_lane_cg_neighbours) != 0:
                        if random.random() < 0.8:
                            # force lane change due to limited execution time
                            candidate = random.choice(final_lane_cg_neighbours)
                            if self.map_service.get_length_of_lane(candidate) >= k_min_lane_length:
                                final_lane_id = candidate
                    # out of lanes condition
                    if self.map_service.get_length_of_lane(final_lane_id) - 6.0 <= 0:
                        if len(self.map_service.get_successors_for_lane(final_lane_id)) > 0:
                            need_to_cut = False
                            break
                        else:
                            successors.remove(final_lane_id)  # remove this candidate, out of lanes
                    else:
                        if len(self.map_service.get_successors_for_lane(final_lane_id)) > 0:
                            need_to_cut = False  # no need to cut since there is another successor lane
                        else:
                            need_to_cut = True  # need to cut since there is no other successor lane
                        break
```

## autoware/autoware_record/record.py

You can refer to [autoware_record](https://github.com/lethal233/autoware_record) for instructions on its usage. This is an updated version of `rosbag_reader.py`. Currently, we have completed unit testing of `autoware_record`, but integration testing with scenoRITA is still pending. Therefore, we are using `rosbag_reader.py` to read all the records for now.

The most different part is that when using `rosbag_reader.py`, you should deserialize the message manually after calling `read_messages()`. In `src/scenoRITA/components/oracles/__init__.py`:

```py
        for topic, message, t in reader.read_messages():
            if not has_localization and topic == '/localization/kinematic_state':
                has_localization = True
            if not has_ground_truth and topic == '/perception/object_recognition/ground_truth/objects':
                has_ground_truth = True
            if topic in self.topic_names():
                msg = reader.deserialize_msg(message, topic) <----------- 
                try:
                    self.oracle_manager.on_new_message(topic, msg, t)
                except OracleInterrupt:
                    break
```

However, when using the updated `autoware/autoware_record/record.py`, please refer to [Speed Up Read](https://github.com/lethal233/autoware_record/blob/main/README.md#speed-up-read-recommended) for instruction to use. In this case, you do not need to manually call `deserialize_msg()` anymore.
