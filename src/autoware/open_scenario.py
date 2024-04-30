from dataclasses import dataclass
from datetime import datetime
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Dict, Any, List

from ruamel.yaml import YAML

from config import ADS_MAP_DIR, PROJECT_NAME
from scenoRITA.representation import Obstacle, ObstacleMotion, ObstacleType, EgoCar
from autoware.utils import obs_hash


@dataclass(slots=True)
class Dimension:
    length: float
    width: float
    height: float


OriginalDim = {
    ObstacleType.CAR: Dimension(4.0, 1.8, 2.5),
    ObstacleType.BUS: Dimension(12.0, 2.5, 2.5),
    ObstacleType.TRUCK: Dimension(8.4, 2.5, 2.5),
    ObstacleType.MOTORCYCLE: Dimension(2.2, 0.8, 2.5),
    ObstacleType.BICYCLE: Dimension(2.0, 0.8, 2.5),
    ObstacleType.PEDESTRIAN: Dimension(0.8, 0.8, 2.0)
}


class Entity(ABC):
    name: str

    def __init__(self, _id, dimensions):
        self._id = _id
        self.dimensions = dimensions

    @abstractmethod
    def scenario_obj(self):
        ...

    @abstractmethod
    def storyboard_obj(self):
        ...


class MiscObject(Entity):

    def __init__(self, _id, dimensions):
        super().__init__(_id, dimensions)

    def scenario_obj(self):
        pass

    def storyboard_obj(self):
        return None


class Vehicle(Entity, ABC):
    def __init__(self, _id, dimensions, args):
        super().__init__(_id, dimensions)
        self.entity = args

    def scenario_obj(self):
        ...

    def storyboard_obj(self):
        ...

    @staticmethod
    def dummy_ego_car():
        return {
            "name": "ego",
            "Vehicle": {
                "name": "",
                "vehicleCategory": "car",
                "BoundingBox": {
                    "Center": {
                        "x": 1.355,
                        "y": 0,
                        "z": 1.25
                    },
                    "Dimensions": {
                        "length": 4.77,
                        "width": 1.83,
                        "height": 2.5
                    }
                },
                "Performance": {
                    "maxSpeed": 50,
                    "maxAcceleration": "INF",
                    "maxDeceleration": "INF"
                },
                "Axles": {
                    "FrontAxle": {
                        "maxSteering": 0.5236,
                        "wheelDiameter": 0.78,
                        "trackWidth": 1.63,
                        "positionX": 1.385,
                        "positionZ": 0.39
                    },
                    "RearAxle": {
                        "maxSteering": 0.5236,
                        "wheelDiameter": 0.6,
                        "trackWidth": 0.78,
                        "positionX": 0,
                        "positionZ": 0.39
                    }
                },
                "Properties": {
                    "Property": []
                }
            },
            "ObjectController": {
                "Controller": {
                    "name": "",
                    "Properties": {
                        "Property": [
                            {
                                "name": "isEgo",
                                "value": 'true'
                            }
                        ]
                    }
                }
            }
        }


class Car(Vehicle):
    def __init__(self, _id, dimensions, entity, ego=False):
        super().__init__(_id, dimensions, entity)
        self.name = f"veh{_id}"
        self.ego = ego
        self.entity = entity

    def scenario_obj(self):
        vehicle = {
            "name": "",
            "vehicleCategory": "car",
            "BoundingBox": {
                "Center": {"x": 0.0, "y": 0.0, "z": 0.0},
                "Dimensions": {"length": 0.0, "width": 0.0, "height": 0.0}
            },
            "Performance": {
                "maxSpeed": 50,
                "maxAcceleration": "INF",
                "maxDeceleration": "INF"
            },
            "Axles": {
                "FrontAxle": {
                    "maxSteering": 0.5236, "wheelDiameter": 0, "trackWidth": 0, "positionX": 0, "positionZ": 0
                },
                "RearAxle": {
                    "maxSteering": 0.5236, "wheelDiameter": 0, "trackWidth": 0, "positionX": 0, "positionZ": 0
                }
            },
            "Properties": {"Property": []}
        }

        controller = {
            "Controller": {
                "name": "",
                "Properties": {"Property": []}
            }
        }

        if self.ego:
            vehicle["BoundingBox"]["Center"].update({"x": 1.355, "z": 1.25})
            vehicle["BoundingBox"]["Dimensions"].update({"length": 4.77, "width": 1.83, "height": 2.5})
            vehicle["Axles"]["FrontAxle"].update(
                {"wheelDiameter": 0.78, "trackWidth": 1.63, "positionX": 1.385, "positionZ": 0.39})
            vehicle["Axles"]["RearAxle"].update({"wheelDiameter": 0.6, "trackWidth": 0.78, "positionZ": 0.39})
            controller["Controller"]["Properties"]["Property"].append({"name": "isEgo", "value": 'true'})
        else:
            vehicle["BoundingBox"]["Center"]["z"] = self.dimensions.height / 2
            vehicle["BoundingBox"]["Dimensions"].update(
                {"length": self.dimensions.length, "width": self.dimensions.width, "height": self.dimensions.height})
            vehicle["Axles"]["FrontAxle"].update(
                {"wheelDiameter": 0.6, "trackWidth": 1.8, "positionX": self.dimensions.length / 2, "positionZ": 0.3})
            vehicle["Axles"]["RearAxle"].update({"wheelDiameter": 0.6, "trackWidth": 1.8, "positionZ": 0.3})

        return {
            "name": "ego" if self.ego else self.name,
            "Vehicle": vehicle,
            "ObjectController": controller
        }

    def storyboard_obj(self):
        if isinstance(self.entity, EgoCar):
            return {
                "entityRef": "ego",
                "PrivateAction": [
                    {
                        "TeleportAction": {
                            "Position": {
                                "LanePosition": {
                                    "roadId": "",
                                    "laneId": str(self.entity.initial_position.lane_id),
                                    "s": self.entity.initial_position.s,
                                    "offset": 0,
                                    "Orientation": {
                                        "type": "relative",
                                        "h": 0,
                                        "p": -0.0,
                                        "r": 0
                                    }
                                }
                            }
                        }
                    },
                    {
                        "RoutingAction": {
                            "AcquirePositionAction": {
                                "Position": {
                                    "LanePosition": {
                                        "roadId": "",
                                        "laneId": str(self.entity.final_position.lane_id),
                                        "s": self.entity.final_position.s // 1,
                                        "offset": 0,
                                        "Orientation": {
                                            "type": "relative",
                                            "h": 0,
                                            "p": -0.0,
                                            "r": 0
                                        }
                                    }
                                }
                            }
                        }
                    }
                ]
            }
        elif isinstance(self.entity, Obstacle):
            return storyboard_transform(self.name, self.entity)
        else:
            raise NotImplementedError("args must be EgoCar or Obstacle")


class Bus(Vehicle):
    def __init__(self, _id, dimensions, entity, ego=False):
        super().__init__(_id, dimensions, entity)
        self.name = f"bus{_id}"
        self.ego = ego
        self.entity = entity

    def scenario_obj(self):
        bus = {
            "name": "",
            "vehicleCategory": "bus",
            "BoundingBox": {
                "Center": {"x": 0.0, "y": 0.0, "z": 0.0},
                "Dimensions": {"length": 0.0, "width": 0.0, "height": 0.0}
            },
            "Performance": {
                "maxSpeed": 50,
                "maxAcceleration": "INF",
                "maxDeceleration": "INF"
            },
            "Axles": {
                "FrontAxle": {
                    "maxSteering": 0.5236, "wheelDiameter": 0.6, "trackWidth": 2.5, "positionX": 0, "positionZ": 0
                },
                "RearAxle": {
                    "maxSteering": 0.5236, "wheelDiameter": 0.6, "trackWidth": 2.5, "positionX": 0, "positionZ": 0
                }
            },
            "Properties": {"Property": []}
        }

        controller = {
            "Controller": {
                "name": "",
                "Properties": {"Property": []}
            }
        }
        bus["BoundingBox"]["Center"]["z"] = self.dimensions.height / 2
        bus["BoundingBox"]["Dimensions"].update(
            {"length": self.dimensions.length, "width": self.dimensions.width, "height": self.dimensions.height})
        bus["Axles"]["FrontAxle"].update(
            {"wheelDiameter": 0.6, "trackWidth": 2.5, "positionX": self.dimensions.length / 2, "positionZ": 0.3})
        bus["Axles"]["RearAxle"].update({"wheelDiameter": 0.6, "trackWidth": 2.5, "positionZ": 0.3})

        return {
            "name": self.name,
            "Vehicle": bus,
            "ObjectController": controller
        }

    def storyboard_obj(self):
        if isinstance(self.entity, Obstacle):
            return storyboard_transform(self.name, self.entity)
        else:
            raise NotImplementedError("args must be EgoCar or Obstacle")


class Truck(Vehicle):
    def __init__(self, _id, dimensions, entity, ego=False):
        super().__init__(_id, dimensions, entity)
        self.name = f"tru{_id}"
        self.ego = ego
        self.entity = entity

    def scenario_obj(self):
        truck = {
            "name": "",
            "vehicleCategory": "truck",
            "BoundingBox": {
                "Center": {"x": 0.0, "y": 0.0, "z": 0.0},
                "Dimensions": {"length": 0.0, "width": 0.0, "height": 0.0}
            },
            "Performance": {
                "maxSpeed": 50,
                "maxAcceleration": "INF",
                "maxDeceleration": "INF"
            },
            "Axles": {
                "FrontAxle": {
                    "maxSteering": 0.5236, "wheelDiameter": 0.6, "trackWidth": 2.2, "positionX": 0, "positionZ": 0
                },
                "RearAxle": {
                    "maxSteering": 0.5236, "wheelDiameter": 0.6, "trackWidth": 2.2, "positionX": 0.0, "positionZ": 0.0
                }
            },
            "Properties": {"Property": []}
        }

        controller = {
            "Controller": {
                "name": "",
                "Properties": {"Property": []}
            }
        }
        truck["BoundingBox"]["Center"]["z"] = self.dimensions.height / 2
        truck["BoundingBox"]["Dimensions"].update(
            {"length": self.dimensions.length, "width": self.dimensions.width, "height": self.dimensions.height})
        truck["Axles"]["FrontAxle"].update(
            {"wheelDiameter": 0.6, "trackWidth": 2.2, "positionX": self.dimensions.length / 2, "positionZ": 0.3})
        truck["Axles"]["RearAxle"].update(
            {"wheelDiameter": 0.6, "trackWidth": 2.2, "positionX": 0.0, "positionZ": 0.3})

        return {
            "name": self.name,
            "Vehicle": truck,
            "ObjectController": controller
        }

    def storyboard_obj(self):
        if isinstance(self.entity, Obstacle):
            return storyboard_transform(self.name, self.entity)
        else:
            raise NotImplementedError("args must be EgoCar or Obstacle")


class Motorcycle(Vehicle):
    def __init__(self, _id, dimensions, entity, ego=False):
        super().__init__(_id, dimensions, entity)
        self.name = f"moto{_id}"
        self.ego = ego
        self.entity = entity

    def scenario_obj(self):
        motorcycle = {
            "name": "",
            "vehicleCategory": "motorbike",
            "BoundingBox": {
                "Center": {"x": 0.0, "y": 0.0, "z": 0.0},
                "Dimensions": {"length": 0.0, "width": 0.0, "height": 0.0}
            },
            "Performance": {
                "maxSpeed": 50,
                "maxAcceleration": "INF",
                "maxDeceleration": "INF"
            },
            "Axles": {
                "FrontAxle": {
                    "maxSteering": 0.5236, "wheelDiameter": 0.6, "trackWidth": 1.8, "positionX": 0, "positionZ": 0
                },
                "RearAxle": {
                    "maxSteering": 0.5236, "wheelDiameter": 0.6, "trackWidth": 1.8, "positionX": 0.0, "positionZ": 0.0
                }
            },
            "Properties": {"Property": []}
        }

        controller = {
            "Controller": {
                "name": "",
                "Properties": {"Property": []}
            }
        }
        motorcycle["BoundingBox"]["Center"]["z"] = self.dimensions.height / 2
        motorcycle["BoundingBox"]["Dimensions"].update(
            {"length": self.dimensions.length, "width": self.dimensions.width, "height": self.dimensions.height})
        motorcycle["Axles"]["FrontAxle"].update(
            {"wheelDiameter": 0.6, "trackWidth": 1.8, "positionX": self.dimensions.length / 2, "positionZ": 0.3})
        motorcycle["Axles"]["RearAxle"].update(
            {"wheelDiameter": 0.6, "trackWidth": 1.8, "positionX": 0.0, "positionZ": 0.3})

        return {
            "name": self.name,
            "Vehicle": motorcycle,
            "ObjectController": controller
        }

    def storyboard_obj(self):
        if isinstance(self.entity, Obstacle):
            return storyboard_transform(self.name, self.entity)
        else:
            raise NotImplementedError("args must be EgoCar or Obstacle")


class Bicycle(Vehicle):
    def __init__(self, _id, dimensions, args):
        super().__init__(_id, dimensions, args)
        self.name = f"bic{_id}"
        self.args = args

    def scenario_obj(self):
        return {
            "name": self.name,
            "Vehicle": {
                "name": "",
                "vehicleCategory": "bicycle",
                "BoundingBox": {
                    "Center": {
                        "x": 0,
                        "y": 0,
                        "z": self.dimensions.height / 2
                    },
                    "Dimensions": {
                        "length": self.dimensions.length,
                        "width": self.dimensions.width,
                        "height": self.dimensions.height
                    }
                },
                "Performance": {
                    "maxSpeed": 50,
                    "maxAcceleration": "INF",
                    "maxDeceleration": "INF"
                },
                "Axles": {
                    "FrontAxle": {
                        "maxSteering": 0.5236,
                        "wheelDiameter": 0.6,
                        "trackWidth": 0.8,
                        "positionX": self.dimensions.length / 2,
                        "positionZ": 0.3  # todo: wheelDiameter/2
                    },
                    "RearAxle": {
                        "maxSteering": 0.5236,
                        "wheelDiameter": 0.6,
                        "trackWidth": 0.6,
                        "positionX": 0,
                        "positionZ": 0.3
                    }
                },
                "Properties": {
                    "Property": []
                }
            },
            "ObjectController": {
                "Controller": {
                    "name": "",
                    "Properties": {
                        "Property": []
                    }
                }
            }
        }

    def storyboard_obj(self):
        if not isinstance(self.args, Obstacle):
            raise NotImplementedError("args must be Obstacle")
        else:
            return storyboard_transform(self.name, self.args)


class Pedestrian(Entity):
    def __init__(self, _id, dimensions, args, mass=60):
        super().__init__(_id, dimensions)
        self.name = f"ped{_id}"
        self.mass = mass
        self.args = args

    def scenario_obj(self):
        return {
            "name": self.name,
            "Pedestrian": {
                "name": "Pedestrian",
                "mass": self.mass,
                "model": "",
                "pedestrianCategory": "pedestrian",
                "BoundingBox": {
                    "Center": {
                        "x": 0,
                        "y": 0,
                        "z": self.dimensions.height / 2
                    },
                    "Dimensions": {
                        "length": self.dimensions.length,
                        "width": self.dimensions.width,
                        "height": self.dimensions.height
                    }
                },
                "Properties": {
                    "Property": []
                }
            }
        }

    def storyboard_obj(self):
        if not isinstance(self.args, Obstacle):
            raise NotImplementedError("args must be Obstacle")
        else:
            return storyboard_transform(self.name, self.args)


class OpenScenario:
    def __init__(self, generation_id, scenario_id, ego_car, obstacles, map_name, scenario_modifiers=None):
        self.generation_id = generation_id
        self.scenario_id = scenario_id
        self.ego_car = ego_car
        self.obstacles = obstacles
        self.reassign_obs_ids()
        self.map_name = map_name
        self.scenario_modifiers = scenario_modifiers
        self.obstacle_entity = [self.get_corresponding_entity(x) for x in self.obstacles]

    def get_id(self) -> str:
        return f"gen_{self.generation_id}_sce_{self.scenario_id}"

    def reassign_obs_ids(self) -> bool:
        _type_cnt = {}
        for obs in self.obstacles:
            if not _type_cnt.__contains__(obs.type):
                _type_cnt[obs.type] = 1
            else:
                _type_cnt[obs.type] += 1
        cnt_values = set(_type_cnt.values())
        if cnt_values == {1}:
            for obs in self.obstacles:
                # if there is only one obstacle in this obstacle type, no need to add additional bit
                original_length = OriginalDim[obs.type].length
                original_width = OriginalDim[obs.type].width
                obs.id = obs_hash(original_length, original_width, obs.height)
            return False

        for obs in self.obstacles:
            cnt = _type_cnt[obs.type]
            original_length = OriginalDim[obs.type].length
            original_width = OriginalDim[obs.type].width
            if cnt == 1:
                # if there is only one obstacle in this obstacle type, no need to add additional bit
                obs.id = obs_hash(original_length, original_width, obs.height)
            else:
                # if there are multiple ones, add 0.0(cnt) to the length / weight
                obs.length = round(original_length + ((cnt - 1) / 100), 2)
                obs.width = round(original_width + ((cnt - 1) / 100), 2)
                obs.id = obs_hash(obs.length, obs.width, obs.height)
            _type_cnt[obs.type] -= 1

        return True

    def get_corresponding_entity(self, obs) -> Entity:
        dim = Dimension(obs.length, obs.width, obs.height)
        if obs.type == ObstacleType.CAR:
            return Car(obs.id, dim, obs)
        elif obs.type == ObstacleType.BUS:
            return Bus(obs.id, dim, obs)
        elif obs.type == ObstacleType.TRUCK:
            return Truck(obs.id, dim, obs)
        elif obs.type == ObstacleType.MOTORCYCLE:
            return Motorcycle(obs.id, dim, obs)
        elif obs.type == ObstacleType.PEDESTRIAN:
            return Pedestrian(obs.id, dim, obs)
        elif obs.type == ObstacleType.BICYCLE:
            return Bicycle(obs.id, dim, obs)
        else:
            raise NotImplementedError("Obstacle type not supported")

    def get_scenario_modifiers(self):
        return {
            "ScenarioModifier": []
        } if self.scenario_modifiers is None else self.scenario_modifiers

    def get_dummy_header(self) -> Dict[str, Any]:
        return {
            "FileHeader": {
                "revMajor": 1,
                "revMinor": 1,
                "date": datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z",
                "description": "",
                "author": f"Generated by {PROJECT_NAME}"
            },
            "ParameterDeclarations": {
                "ParameterDeclaration": []
            },
            "CatalogLocations": {
                "CatalogLocation": []
            },
            "RoadNetwork": {
                "LogicFile": {
                    # "filepath": f"{self.map_name}.osm"
                    "filepath": f"{ADS_MAP_DIR}/{self.map_name}/lanelet2_map.osm"
                },
                "SceneGraphFile": {
                    # "filepath": f"{self.map_name}.pcd"
                    "filepath": f"{ADS_MAP_DIR}/{self.map_name}/pointcloud_map.pcd"
                },
                "TrafficSignals": {
                    "TrafficSignalController": []
                }
            }
        }

    def get_scenario_profile(self) -> Dict[str, Any]:
        osc = self.get_dummy_header()
        entities = [Car("ego", None, self.ego_car, ego=True)]
        entities.extend(self.obstacle_entity)

        osc['Entities'] = {
            "ScenarioObject": [x.scenario_obj() for x in entities]
        }
        private_actions = [x.storyboard_obj() for x in entities]

        osc['Storyboard'] = {
            "Init": {
                "Actions": {
                    "Private": private_actions
                }
            },
            "Story": self.get_dummy_story(),
            "StopTrigger": {
                "ConditionGroup": []
            }
        }
        osc['Storyboard']['Story'][0]['Act'].extend(self.obstacle_reach_stop_actions())
        osc['Storyboard']['Story'][0]['Act'][0]['ManeuverGroup'][0]['Maneuver'][0]['Event'][1]['StartTrigger'][
            'ConditionGroup'].extend(self.__generate_end_collision_condition())
        return {
            "ScenarioModifiers": self.get_scenario_modifiers(),
            "OpenSCENARIO": osc
        }

    def __generate_end_collision_condition(self):
        end_cond = []
        for x in self.obstacle_entity:
            end_cond.append(
                {
                    "Condition": [
                        {
                            "name": "",
                            "delay": 0,
                            "conditionEdge": "none",
                            "ByEntityCondition": {
                                "TriggeringEntities": {
                                    "triggeringEntitiesRule": "any",
                                    "EntityRef": [
                                        {
                                            "entityRef": "ego"
                                        }
                                    ]
                                },
                                "EntityCondition": {
                                    "CollisionCondition": {
                                        "EntityRef": {
                                            "entityRef": f"{x.name}"
                                        }
                                    }
                                }
                            }
                        }
                    ]
                }
            )
        return end_cond

    def get_dummy_story(self) -> List[Dict[str, Any]]:
        return [
            {
                "name": "",
                "Act": [
                    {
                        "name": "_EndCondition",
                        "ManeuverGroup": [
                            {
                                "maximumExecutionCount": 1,
                                "name": "",
                                "Actors": {
                                    "selectTriggeringEntities": False,
                                    "EntityRef": [
                                        {
                                            "entityRef": "ego"
                                        }
                                    ]
                                },
                                "Maneuver": [
                                    {
                                        "name": "",
                                        "Event": [
                                            {
                                                "name": "",
                                                "priority": "parallel",
                                                "StartTrigger": {
                                                    "ConditionGroup": [
                                                        {
                                                            "Condition": [
                                                                {
                                                                    "name": "",
                                                                    "delay": 0,
                                                                    "conditionEdge": "none",
                                                                    "ByEntityCondition": {
                                                                        "TriggeringEntities": {
                                                                            "triggeringEntitiesRule": "any",
                                                                            "EntityRef": [
                                                                                {
                                                                                    "entityRef": "ego"
                                                                                }
                                                                            ]
                                                                        },
                                                                        "EntityCondition": {
                                                                            "ReachPositionCondition": {
                                                                                "Position": {
                                                                                    "LanePosition": {
                                                                                        "roadId": "",
                                                                                        "laneId": str(
                                                                                            self.ego_car.final_position.lane_id),
                                                                                        "s": self.ego_car.final_position.s // 1,
                                                                                        "offset": 0,
                                                                                        "Orientation": {
                                                                                            "type": "relative",
                                                                                            "h": 0,
                                                                                            "p": -0.0,
                                                                                            "r": 0
                                                                                        }
                                                                                    }
                                                                                },
                                                                                "tolerance": 1
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            ]
                                                        }
                                                    ]
                                                },
                                                "Action": [
                                                    {
                                                        "name": "",
                                                        "UserDefinedAction": {
                                                            "CustomCommandAction": {
                                                                "type": "exitSuccess"
                                                            }
                                                        }
                                                    }
                                                ]
                                            },
                                            {
                                                "name": "",
                                                "priority": "parallel",
                                                "StartTrigger": {
                                                    "ConditionGroup": [
                                                        {
                                                            "Condition": [
                                                                {
                                                                    "name": "",
                                                                    "delay": 0,
                                                                    "conditionEdge": "none",
                                                                    "ByValueCondition": {
                                                                        "SimulationTimeCondition": {
                                                                            "value": 180,
                                                                            "rule": "greaterThan"
                                                                        }
                                                                    }
                                                                }
                                                            ]
                                                        }
                                                    ]
                                                },
                                                "Action": [
                                                    {
                                                        "name": "",
                                                        "UserDefinedAction": {
                                                            "CustomCommandAction": {
                                                                "type": "exitFailure"
                                                            }
                                                        }
                                                    }
                                                ]
                                            }
                                        ]
                                    }
                                ]
                            }
                        ],
                        "StartTrigger": {
                            "ConditionGroup": [
                                {
                                    "Condition": [
                                        {
                                            "name": "",
                                            "delay": 0,
                                            "conditionEdge": "none",
                                            "ByValueCondition": {
                                                "SimulationTimeCondition": {
                                                    "value": 0,
                                                    "rule": "greaterThan"
                                                }
                                            }
                                        }
                                    ]
                                }
                            ]
                        }
                    }
                ]
            }
        ]

    def obstacle_reach_stop_actions(self):
        act = []
        for i in range(len(self.obstacles)):
            if self.obstacles[i].motion == ObstacleMotion.STATIC:
                continue
            obs = self.obstacle_entity[i]
            act.append(
                {
                    "name": f"act_{obs.name}_stop",
                    "ManeuverGroup": [
                        {
                            "maximumExecutionCount": 1,
                            "name": f"act_{obs.name}_stop",
                            "Actors": {
                                "selectTriggeringEntities": False,
                                "EntityRef": [
                                    {
                                        "entityRef": obs.name
                                    }
                                ]
                            },
                            "Maneuver": [
                                {
                                    "name": "",
                                    "Event": [
                                        {
                                            "name": f"{obs.name}_goal_reached",
                                            "priority": "parallel",
                                            "StartTrigger": {
                                                "ConditionGroup": [
                                                    {
                                                        "Condition": [
                                                            {
                                                                "name": "",
                                                                "delay": 0,
                                                                "conditionEdge": "none",
                                                                "ByEntityCondition": {
                                                                    "TriggeringEntities": {
                                                                        "triggeringEntitiesRule": "any",
                                                                        "EntityRef": [
                                                                            {
                                                                                "entityRef": obs.name
                                                                            }
                                                                        ]
                                                                    },
                                                                    "EntityCondition": {
                                                                        "ReachPositionCondition": {
                                                                            "Position": {
                                                                                "LanePosition": {
                                                                                    "roadId": "",
                                                                                    "laneId": str(
                                                                                        self.obstacles[
                                                                                            i].final_position.lane_id),
                                                                                    "s": self.obstacles[
                                                                                             i].final_position.s // 1,
                                                                                    "offset": 0,
                                                                                    "Orientation": {
                                                                                        "type": "relative",
                                                                                        "h": 0,
                                                                                        "p": -0.0,
                                                                                        "r": 0
                                                                                    }
                                                                                }
                                                                            },
                                                                            "tolerance": 1
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        ]
                                                    }
                                                ]
                                            },
                                            "Action": [
                                                {
                                                    "name": "",
                                                    "PrivateAction": {
                                                        "LongitudinalAction": {
                                                            "SpeedAction": {
                                                                "SpeedActionDynamics": {
                                                                    "dynamicsDimension": "time",
                                                                    "value": 0,
                                                                    "dynamicsShape": "step"
                                                                },
                                                                "SpeedActionTarget": {
                                                                    "AbsoluteTargetSpeed": {
                                                                        "value": 0
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            ]
                                        }
                                    ]
                                }
                            ]
                        }
                    ],
                    "StartTrigger": {
                        "ConditionGroup": [
                            {
                                "Condition": [
                                    {
                                        "name": "",
                                        "delay": 0,
                                        "conditionEdge": "none",
                                        "ByValueCondition": {
                                            "SimulationTimeCondition": {
                                                "value": 0,
                                                "rule": "greaterThan"
                                            }
                                        }
                                    }
                                ]
                            }
                        ]
                    }
                }
            )
        return act

    def export_to_file(self, file_path: Path):
        sce_pf = self.get_scenario_profile()
        fn = Path(file_path, f"{self.get_id()}.yaml")
        fn.parent.mkdir(parents=True, exist_ok=True)
        fp = open(Path(file_path, f"{self.get_id()}.yaml"), 'w+')
        yaml = YAML()
        yaml.default_flow_style = False
        yaml.dump(sce_pf, fp)
        fp.close()


def storyboard_transform(name: str, args: Obstacle) -> Dict[str, Any]:
    obs_result = {
        "entityRef": name,
        "PrivateAction": [
            {
                "TeleportAction": {
                    "Position": {
                        "LanePosition": {
                            "roadId": "",
                            "laneId": str(args.initial_position.lane_id),
                            "s": round(args.initial_position.s, 3),
                            "offset": 0,
                            "Orientation": {
                                "type": "relative",
                                "h": 0,
                                "p": -0.0,
                                "r": 0
                            }
                        }
                    }
                }
            },
            {
                "LongitudinalAction": {
                    "SpeedAction": {
                        "SpeedActionDynamics": {
                            "dynamicsDimension": "time",
                            "value": 0,
                            "dynamicsShape": "step"
                        },
                        "SpeedActionTarget": {
                            "AbsoluteTargetSpeed": {
                                "value": args.speed if args.motion == ObstacleMotion.DYNAMIC else 0
                            }
                        }
                    }
                }
            }
        ]
    }
    if args.motion == ObstacleMotion.DYNAMIC:
        obs_result['PrivateAction'].append(
            {
                "RoutingAction": {
                    "AcquirePositionAction": {
                        "Position": {
                            "LanePosition": {
                                "roadId": "",
                                "laneId": str(args.final_position.lane_id),
                                "s": args.final_position.s // 1,
                                "offset": 0,
                                "Orientation": {
                                    "type": "relative",
                                    "h": 0,
                                    "p": -0.0,
                                    "r": 0
                                }
                            }
                        }
                    }
                }
            }
        )
    return obs_result
