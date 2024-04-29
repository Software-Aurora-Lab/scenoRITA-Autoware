from abc import ABC, abstractmethod
from typing import List

from autoware.map_service import MapService
from scenoRITA.components.oracles.MetricHelper import MetricHelper
from autoware.utils import quaternion_2_heading
from autoware.utils import calculate_velocity
from nav_msgs.msg import Odometry


class BasicMetric(ABC):
    mh: MetricHelper

    def __init__(self):
        self.map_service = MapService.instance()

    @abstractmethod
    def get_interested_topics(self) -> List[str]:
        return list()

    @abstractmethod
    def on_new_message(self, topic: str, message, t):
        pass

    @abstractmethod
    def get_result(self):
        return list()

    @abstractmethod
    def get_fitness(self):
        ...

    @staticmethod
    def get_dummy_basic_info():
        return {
            'x': 0,
            'y': 0,
            'heading': 0,
            'speed': 0
        }

    @staticmethod
    def get_basic_info_from_localization(message: Odometry):
        speed = calculate_velocity(message.twist.twist.linear)
        heading = quaternion_2_heading(message.pose.pose.orientation)
        features = {
            'x': message.pose.pose.position.x,
            'y': message.pose.pose.position.y,
            'heading': heading,
            'speed': speed,
        }
        return features

    def set_oh(self, oh: MetricHelper):
        self.mh = oh
