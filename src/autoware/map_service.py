import xml.etree.ElementTree as ET
from typing import TypeVar, Type, Dict, List

import lanelet2
import lanelet2_extension_python.utility.query as query
from lanelet2.projection import UtmProjector
from lanelet2.core import LaneletMap, Lanelet
from lanelet2.io import Origin
from config import MAPS_DIR, SUPPORTED_MAPS
# from tools.hdmap.VectorMapParser import VectorMapParser
from lanelet2_extension_python.projection import MGRSProjector
from pathlib import Path
from autoware.utils import construct_lane_boundary_linestring


class MapLoader:

    def __init__(self, map_name: str):
        # if map_name not in SUPPORTED_MAPS:
        #     raise RuntimeError(f"Map {map_name} not supported")
        # self.hd_map_path = Path(MAPS_DIR, map_name, "lanelet2_map.osm")
        #
        # if not Path.exists(self.hd_map_path):
        #     raise RuntimeError(f"Requested map {map_name} does not exist")

        self.hd_map_path = f'/home/lori/Desktop/autoware_map/autoware_scenario_data/maps/{map_name}/lanelet2_map.osm'
        origin = self.get_first_reference_origin(self.hd_map_path)
        self.projector = MGRSProjector(origin)

        map_parser = VectorMapParser.instance()
        map_parser.lanelet_map = self.load_map()
        map_parser.projector = self.projector
        self.map_instance = map_parser

    def load_map(self) -> LaneletMap:
        self.lanelet_map = lanelet2.io.load(self.hd_map_path, self.projector)
        return self.lanelet_map

    def get_first_reference_origin(self, hd_map_path: str) -> Origin:
        with open(hd_map_path, 'r') as file:
            xml_data = file.read()

        root = ET.fromstring(xml_data)
        first_node = root.find('.//node')

        lat = float(first_node.get('lat'))
        lon = float(first_node.get('lon'))
        return Origin(lat, lon, 0)


class VectorMapParser:
    _instance = None
    lanelet_map: LaneletMap
    projector: MGRSProjector

    T = TypeVar('T')

    @classmethod
    def instance(cls):
        if cls._instance is None:
            cls._instance = cls.__new__(cls)
        return cls._instance

    def __init__(self):
        raise RuntimeError('Call instance() instead')

    def get_attributes(self, key: str, attribute_type: Type[T]) -> Dict[int, T]:
        """
        - attribute_key: ['location', 'one_way', 'participant:vehicle', 'speed_limit', 'subtype', 'turn_direction', 'type']
        """
        result = {}
        for lanelet in self.lanelet_map.laneletLayer:
            if key in lanelet.attributes:
                result[lanelet.id] = attribute_type(lanelet.attributes[key])
        return result

    def get_lanelets(self, identifiers: List[int]) -> List[Lanelet]:
        return [lanelet for lanelet in self.lanelet_map.laneletLayer if lanelet.id in identifiers]

    def get_all_intersections(self) -> Dict[int, str]:
        """
        - return: Dict[lanelet id: turn_direction type]
        """
        return self.get_attributes(key='turn_direction', attribute_type=str)

    def get_all_crosswalk(self):
        return query.crosswalkLanelets(query.laneletLayer(self.lanelet_map))

    def get_lane_boundaries(self) -> dict:
        boundaries = dict()
        for lane in self.lanelet_map.laneletLayer:
            lane_id = lane.id
            l, r = construct_lane_boundary_linestring(lane)
            boundaries[f'{lane_id}_L'] = l
            boundaries[f'{lane_id}_R'] = r
        return boundaries


if __name__ == '__main__':
    MapLoader("ces2024_demo")
    for s in VectorMapParser.instance().get_all_crosswalk():
        print(s.attributes)
    # print()
    # for s in VectorMapParser.instance().lanelet_map.laneletLayer:
    #     print(s.regulatoryElements)
    # print(VectorMapParser.instance().get_all_crosswalk())
