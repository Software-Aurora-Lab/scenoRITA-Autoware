from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List

from deap import base

from autoware.map_service import PositionEstimate
from autoware_auto_perception_msgs.msg import ObjectClassification


class ObstacleType(Enum):
    VEHICLE = ObjectClassification.CAR
    PEDESTRIAN = ObjectClassification.PEDESTRIAN
    BICYCLE = ObjectClassification.BICYCLE


class ObstacleMotion(Enum):
    STATIC = auto()
    DYNAMIC = auto()


@dataclass(slots=True)
class ObstaclePosition:
    lane_id: int
    index: int
    s: float


class ObstacleFitness(base.Fitness):
    # minimize the distance between the ego car and the obstacle
    # minimize difference between speed and limit
    # maximize ego duration on boundary
    # maximize ego car acceleration
    # minimize ego car deceleration
    weights = (-1.0, -1.0, 1.0, 1.0, -1.0)

    @staticmethod
    def get_fallback_fitness() -> tuple[float, ...]:
        result: List[float] = list()
        for w in ObstacleFitness.weights:
            if w == 1.0:
                result.append(float("-inf"))
            else:
                result.append(float("inf"))
        return tuple(result)


@dataclass(slots=True)
class Obstacle:
    id: int
    initial_position: ObstaclePosition
    final_position: ObstaclePosition
    type: ObstacleType
    speed: float
    width: float
    length: float
    height: float
    motion: ObstacleMotion
    fitness: ObstacleFitness = field(default_factory=ObstacleFitness)


@dataclass(slots=True)
class EgoCar:
    initial_position: PositionEstimate
    final_position: PositionEstimate
