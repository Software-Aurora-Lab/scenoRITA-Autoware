import os

from autoware.map_service import load_map_service
from config import ADS_MAP_DIR


def get_map_simple_complexity(map_name: str):
    file_size = os.path.getsize(f"{ADS_MAP_DIR}/{map_name}/lanelet2_map.osm")
    print("Map: ", map_name)
    map_service = load_map_service(map_name)
    map_service.get_junction_lanes()

    print("All lanes: ", len(map_service.all_ln_ids))
    print("Pedestrian Lanes: ", len(map_service.get_pedestrian_lanes()))
    print("Junction Lanes: ", len(map_service.get_junction_lanes()))
    print("file size: ", file_size / 1024, "KB")


if __name__ == '__main__':
    get_map_simple_complexity("Shalun with road shoulders")
    # count_complexity("awf_cicd_virtualmap")
    # count_complexity("Nishi-Shinjuku")
    # count_complexity("Hsinchu city (Taiwan)")

# Output:
# Map:  Hsinchu city (Taiwan)
# All lanes:  788
# Pedestrian Lanes:  4
# Junction Lanes:  83

# Map:  Shalun with road shoulders
# All lanes:  263
# Pedestrian Lanes:  28
# Junction Lanes:  150


# Map:  Nishi-Shinjuku
# All lanes:  979
# Pedestrian Lanes:  92
# Junction Lanes:  387

# Map:  awf_cicd_virtualmap
# All lanes:  764
# Pedestrian Lanes:  68
# Junction Lanes:  288
