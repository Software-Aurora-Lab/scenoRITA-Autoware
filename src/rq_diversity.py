import multiprocessing as mp
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Set, Tuple

import matplotlib as mpl
import numpy as np
import pandas as pd
from autoware_auto_perception_msgs.msg import TrackedObjects, TrackedObject
from nav_msgs.msg import Odometry

from autoware.rosbag_reader import ROSBagReader
from loguru import logger
from matplotlib import pyplot as plt

from autoware.map_service import load_map_service
from autoware.rq_utils import get_routing_msg
from config import PROJECT_ROOT

mpl.rcParams["figure.dpi"] = 900


@dataclass(slots=True)
class LocationAnalysis:
    ego_locations: Set[Tuple[float, float]]
    obs_locations: Set[Tuple[float, float]]


def get_color(alpha: float):
    colors = [
        "#ffffff",
        "#eaf8f2",
        "#a1ddea",
        "#61bdf7",
        "#4f8ba3",
        "#9e5bd5",
        "#c72e7d",
    ]
    values = np.linspace(0.0, 1.0, len(colors))
    return colors[np.searchsorted(values, alpha)]


def analysis_worker(record_path: Path) -> LocationAnalysis:
    logger.info(f"Processing {record_path.name}")
    record_file = ROSBagReader(str(record_path))
    ego_coordinates: Set[Tuple[float, float]] = set()
    obs_coordinates: Set[Tuple[float, float]] = set()

    has_routing_msg = False
    for topic, msg, t in record_file.read_messages():
        if not has_routing_msg:
            if topic == "/planning/mission_planning/route":
                has_routing_msg = True
            else:
                continue
        if topic == "/localization/kinematic_state":
            msg: Odometry = record_file.deserialize_msg(msg, topic)
            ego_coord = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            ego_coordinates.add(ego_coord)
        elif topic == "/perception/object_recognition/ground_truth/objects":
            msg: TrackedObjects = record_file.deserialize_msg(msg, topic)
            for obs in msg.objects:
                obs: TrackedObject
                obs_coord = (obs.kinematics.pose_with_covariance.pose.position.x,
                             obs.kinematics.pose_with_covariance.pose.position.y)
                obs_coordinates.add(obs_coord)
    return LocationAnalysis(ego_coordinates, obs_coordinates)


def generator_adapter(generator):
    for g in generator:
        yield g.parent


def plot_experiment_heatmap(map_name: str, record_root: Path, output_path: Path):
    plt.cla()
    plt.clf()
    map_service = load_map_service(map_name)
    min_x, min_y, max_x, max_y = (
        float("inf"),
        float("inf"),
        float("-inf"),
        float("-inf"),
    )

    # plot map for 2 subplots
    for i in range(1, 3):
        plt.subplot(1, 2, i)
        map_service.get_vehicle_lanes()
        for lane_id in map_service.all_ln_ids:
            central_curve = map_service.get_center_line_lst_by_id(lane_id)
            plt.plot(*central_curve.xy, "k", alpha=0.1)
            minx, miny, maxx, maxy = central_curve.bounds
            min_x = min(min_x, minx)
            min_y = min(min_y, miny)
            max_x = max(max_x, maxx)
            max_y = max(max_y, maxy)
        plt.xticks([])
        plt.yticks([])

    # analyze files
    k_split = 100
    x_ranges = np.linspace(min_x, max_x, k_split)
    y_ranges = np.linspace(min_y, max_y, k_split)
    ego_heat_map_values = np.zeros((len(x_ranges) + 1, len(y_ranges) + 1))
    obs_heat_map_values = np.zeros((len(x_ranges) + 1, len(y_ranges) + 1))

    with mp.Pool(mp.cpu_count()) as pool:
        results = pool.map(analysis_worker, generator_adapter(record_root.rglob("*.db3")))
        for result in results:
            for ego_coord in result.ego_locations:
                x_index = np.searchsorted(x_ranges, ego_coord[0])
                y_index = np.searchsorted(y_ranges, ego_coord[1])
                ego_heat_map_values[x_index, y_index] += 1
            for obs_coord in result.obs_locations:
                x_index = np.searchsorted(x_ranges, obs_coord[0])
                y_index = np.searchsorted(y_ranges, obs_coord[1])
                obs_heat_map_values[x_index, y_index] += 1

    # plot heat map for ego car
    plt.subplot(1, 2, 1)
    max_value = np.max(ego_heat_map_values)
    min_alpha = 0.5
    for x1, x2 in zip(x_ranges[:-1], x_ranges[1:]):
        x_index = np.searchsorted(x_ranges, (x1 + x2) / 2)
        for y1, y2 in zip(y_ranges[:-1], y_ranges[1:]):
            y_index = np.searchsorted(y_ranges, (y1 + y2) / 2)
            alpha = ego_heat_map_values[x_index, y_index] / max_value
            color = get_color(alpha)

            color = "blue"
            if alpha > 0.0:
                # scale alpha to make it more visible
                alpha = min_alpha + alpha * (1 - min_alpha)

            plt.fill([x1, x1, x2, x2], [y1, y2, y2, y1], color=color, alpha=alpha)

    # plot heat map for obstacles
    plt.subplot(1, 2, 2)
    max_value = np.max(obs_heat_map_values)
    for x1, x2 in zip(x_ranges[:-1], x_ranges[1:]):
        x_index = np.searchsorted(x_ranges, (x1 + x2) / 2)
        for y1, y2 in zip(y_ranges[:-1], y_ranges[1:]):
            y_index = np.searchsorted(y_ranges, (y1 + y2) / 2)
            alpha = obs_heat_map_values[x_index, y_index] / max_value
            color = get_color(alpha)

            color = "red"
            if alpha > 0.0:
                # scale alpha to make it more visible
                alpha = min_alpha + alpha * (1 - min_alpha)

            plt.fill([x1, x1, x2, x2], [y1, y2, y2, y1], color=color, alpha=alpha)

    # save figure
    plt.savefig(output_path, bbox_inches="tight")
    del map_service


def check_routing_msgs(root: Path) -> None:
    records_meta_data = list(root.rglob("metadata.yaml"))
    import re

    def extract_numbers(s):
        return tuple(int(num) for num in re.findall(r'\d+', s))

    with mp.Manager() as manager:
        worker_num = mp.cpu_count()
        pool = mp.Pool(worker_num)
        task_queue = manager.Queue()
        result_queue = manager.Queue()
        for index, meta_data_pth in enumerate(records_meta_data):
            task_queue.put((0, index, meta_data_pth))
        for _ in range(worker_num):
            task_queue.put(None)

        pool.starmap(
            get_routing_msg,
            [(task_queue, result_queue) for _ in range(worker_num)],
        )
        pool.close()
        pool.join()

        results = []
        while not result_queue.empty():
            results.append(result_queue.get())

        results = sorted(results, key=lambda x: extract_numbers(x[0]))

        pd.DataFrame(results, columns=["scenario", "acceleration", "localization", "ground_truth", "perception",
                                       "routing"]).to_csv(Path(root, "msgs.csv"), index=False)


if __name__ == "__main__":
    # Experiment 1
    scenoRITA_shalun_path = Path(
        fr"{PROJECT_ROOT}/out/0503_155808_Shalun with road shoulders/records"
    )  # 5-15 obstacles
    # Experiment 3
    scenoRITA_nishi_path = Path(
        fr"{PROJECT_ROOT}/out/0504_055929_Nishi-Shinjuku/records"
    )  # 5-15 obstacles
    # Experiment 2
    scenoRITA_hsinchu_path = Path(
        fr"{PROJECT_ROOT}/out/0503_024541_Hsinchu city (Taiwan)/records"
    )  # 5-15 obstacles
    # Experiment 4
    scenoRITA_nishi_path_2 = Path(
        fr"{PROJECT_ROOT}/out/0505_232108_Nishi-Shinjuku/records"
    )
    # Experiment 6
    scenoRITA_virtual_map = Path(
        fr"{PROJECT_ROOT}/out/0508_004054_awf_cicd_virtualmap/records"
    )
    #######################################################################
    # Experiment 5
    scenoRITA_nishi_8hrs = Path(
        fr"{PROJECT_ROOT}/out/0509_111556_Nishi-Shinjuku/records"
    )
    # Experiemnt 7
    scenoRITA_awf_8hrs = Path(
        fr"{PROJECT_ROOT}/out/0509_224655_awf_cicd_virtualmap/records"
    )

    exp_records = [
        # ("Shalun with road shoulders", scenoRITA_shalun_path, "scenoRITA"),
        # ("Nishi-Shinjuku", scenoRITA_nishi_path, "scenoRITA"),
        # ("Nishi-Shinjuku", scenoRITA_nishi_path_2, "scenoRITA"),
        # ("Hsinchu city (Taiwan)", scenoRITA_hsinchu_path, "scenoRITA"),
        # ("Hsinchu city (Taiwan)", scenoRITA_hsinchu_2_hour, "scenoRITA"),
        # ("awf_cicd_virtualmap", scenoRITA_virtual_map, "scenoRITA"),
        #######################################################################
        # ("Nishi-Shinjuku", scenoRITA_nishi_8hrs, "scenoRITA")
        ("awf_cicd_virtualmap", scenoRITA_awf_8hrs, "scenoRITA"),
    ]
    # todo: draw one by one

    for map_name, record_root, approach_name in exp_records:
        if record_root != "" and record_root.exists():
            start = time.perf_counter()
            logger.info(f"Plotting {map_name} {approach_name}")
            plot_experiment_heatmap(
                map_name, record_root, Path(record_root, f"{record_root.parent.name}_{approach_name}.png")
            )
            # check_routing_msgs(record_root)
            minutes = (time.perf_counter() - start) / 60
            logger.info(f"Finished {map_name} {approach_name} in {minutes:.2f} minutes")
