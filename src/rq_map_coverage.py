import time
import multiprocessing as mp
from pathlib import Path
from typing import Dict, Set, Tuple, Optional

from loguru import logger
from nav_msgs.msg import Odometry
from shapely.geometry import LineString, Polygon
from autoware.rosbag_reader import ROSBagReader

from autoware.map_service import load_map_service
from config import PROJECT_ROOT
import pickle


def store_localization_msg(
        task_queue: "mp.Queue[Optional[Tuple[int, int, Path]]]",
        map_service
):
    while True:
        record_path = task_queue.get()
        if record_path is None:
            break
        logger.info(f"Processing {record_path[2].name}")
        ego_coordinates: Set[Tuple[float, float]] = set()
        record = ROSBagReader(str(record_path[2]))
        if not record.has_routing_msg():
            logger.warning(f"Record {record_path[2].name} does not have routing message")
            return
        ego_lanes: Set[int] = set()

        for topic, msg, t in record.read_specific_messages("/localization/kinematic_state"):
            ego_coord = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            if ego_coord == (0.0, 0.0):
                continue
            ego_coordinates.add(ego_coord)
            e_lanes = map_service.get_veh_current_lanelets(
                msg.pose.pose.position
            )
            e_lanes_id = [ll.id for ll in e_lanes]
            ego_lanes = ego_lanes.union(e_lanes_id)
        if len(ego_coordinates) < 2:
            logger.warning(f"Record {record_path[2].name} has less than 2 coordinates")
            return []
        ego_lst = LineString(ego_coordinates)
        with open(record_path[2] / "ego_lst.pkl", "wb") as f:
            pickle.dump(ego_lst, f)

        with open(record_path[2] / "ego_lanes.pkl", "wb") as f:
            pickle.dump(ego_lanes, f)
        logger.info(f"Finished {record_path[2].name}")


def localization_msgs(record_root: Path, map_name: str):
    map_service = load_map_service(map_name)
    records_fp = list(record_root.rglob("*.db3"))
    records_fp = [_.parent for _ in records_fp]
    with mp.Manager() as manager:
        worker_num = mp.cpu_count()
        pool = mp.Pool(worker_num)
        task_queue = manager.Queue()
        for index, rc_fp in enumerate(records_fp):
            task_queue.put((0, index, rc_fp))
        for _ in range(worker_num):
            task_queue.put(None)

        pool.starmap(
            store_localization_msg,
            [(task_queue, map_service) for _ in range(worker_num)],
        )
        pool.close()


def compute_coverage(map_name: str, record_root: Path):
    record_files = sorted(record_root.rglob("*.db3"))
    record_files = [_.parent for _ in record_files]
    map_service = load_map_service(map_name)

    # initialize junction polygons, signal lines and stop sign lines
    junction_polygons: Dict[str, Polygon] = dict()
    signal_lines: Dict[str, LineString] = dict()
    unique_signal_lines: Set[LineString] = set()
    stop_sign_lines: Dict[str, LineString] = dict()
    unique_stop_sign_lines: Set[LineString] = set()

    # Build junction polygons, signal lines and stop sign lines
    for junction_id in map_service.get_junction_lanes():
        junction = map_service.get_lane_by_id(junction_id)
        junction_polygon = Polygon([(x.x, x.y) for x in junction.polygon2d()])
        junction_polygons[junction_id] = junction_polygon

    for signal in map_service.get_signals():
        assert len(signal.parameters['ref_line']) == 1
        signal_linestring = LineString(
            [(x.x, x.y) for x in signal.parameters['ref_line'][0]]
        )
        if signal_linestring not in unique_signal_lines:
            unique_signal_lines.add(signal_linestring)
            signal_lines[signal.id] = signal_linestring
        else:
            logger.warning(f"Duplicate signal {signal.id}")

    for stop_sign_ref_line in map_service.get_stop_signs():
        stop_sign_linestring = LineString(
            [(x.x, x.y) for x in stop_sign_ref_line]
        )
        if stop_sign_linestring not in unique_stop_sign_lines:
            unique_stop_sign_lines.add(stop_sign_linestring)
            stop_sign_lines[stop_sign_ref_line.id] = stop_sign_linestring
        else:
            logger.warning(f"Duplicate stop sign {stop_sign_ref_line.id}")

    # initialize coverage dictionaries
    junction_coverage: Dict[str, int] = dict()
    signal_coverage: Dict[str, int] = dict()
    stop_sign_coverage: Dict[str, int] = dict()

    # compute coverage for each record
    for index, record_file in enumerate(record_files):
        logger.info(f"Processing {record_file.name} ({index + 1}/{len(record_files)})")

        ego_lanes: Set[str] = set()
        # construct ego trajectory line string
        ego_coordinates: Set[Tuple[float, float]] = set()
        record = ROSBagReader(str(record_file))
        if not record.has_routing_msg():
            logger.warning(f"Record {record_file.name} does not have routing message")
            continue
        for _, msg, _ in record.read_specific_messages("/localization/kinematic_state"):
            msg: Odometry
            ego_coord = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            if ego_coord == (0.0, 0.0):
                continue
            ego_coordinates.add(ego_coord)
            e_lanes = map_service.get_veh_current_lanelets(
                msg.pose.pose.position
            )
            ego_lanes = ego_lanes.union(e_lanes)
        if len(ego_coordinates) < 2:
            logger.warning(f"Record {record_file.name} has less than 2 coordinates")
            continue
        ego_lst = LineString(ego_coordinates)

        # ego_lane_overlap_ids: Set[str] = set()
        # for el in ego_lanes:
        # ego_lane_overlap_ids |= set(
        #     [x.id for x in map_service.lane_table[el].overlap_id]
        # )

        # compute coverage for junctions, signals and stop signs
        for junction_id, junction_polygon in junction_polygons.items():
            if junction_polygon.intersects(ego_lst):
                if junction_id not in junction_coverage:
                    junction_coverage[junction_id] = 1
                else:
                    junction_coverage[junction_id] += 1

        for signal_id, signal_linestring in signal_lines.items():
            if signal_linestring.intersects(ego_lst):
                # signal_overlap_ids = [
                #     x.id for x in map_service.signal_table[signal_id].overlap_id
                # ]
                if set(ego_lanes) & set(signal_lines.keys()):
                    if signal_id not in signal_coverage:
                        signal_coverage[signal_id] = 1
                    else:
                        signal_coverage[signal_id] += 1

        for stop_sign_id, stop_sign_linestring in stop_sign_lines.items():
            if stop_sign_linestring.intersects(ego_lst):
                # stop_sign_overlap_ids = [
                #     x.id for x in map_service.stop_sign_table[stop_sign_id].overlap_id
                # ]
                if set(ego_lanes) & set(stop_sign_lines.keys()):
                    if stop_sign_id not in stop_sign_coverage:
                        stop_sign_coverage[stop_sign_id] = 1
                    else:
                        stop_sign_coverage[stop_sign_id] += 1

    # print coverage results
    print(f"Number of junctions: {len(junction_polygons)}")
    print(f"Number of junctions covered: {len(junction_coverage)}")
    print(f"Number of signals: {len(signal_lines)}")
    print(f"Number of signals covered: {len(signal_coverage)}")
    print(f"Number of stop signs: {len(stop_sign_lines)}")
    print(f"Number of stop signs covered: {len(stop_sign_coverage)}")


if __name__ == "__main__":
    scenoRITA_shalun_path = Path(
        fr"{PROJECT_ROOT}/out/0503_155808_Shalun with road shoulders/records"
    )
    scenoRITA_nishi_path = Path(
        fr"{PROJECT_ROOT}/out/0504_055929_Nishi-Shinjuku/records"
    )
    scenoRITA_hsinchu_path = Path(
        fr"{PROJECT_ROOT}/out/0503_024541_Hsinchu city (Taiwan)/records"
    )

    exp_records = [
        ("Shalun with road shoulders", scenoRITA_shalun_path, "scenoRITA"),
        # ("Nishi-Shinjuku", scenoRITA_nishi_path, "scenoRITA"),
        # ("Hsinchu city (Taiwan)", scenoRITA_hsinchu_path, "scenoRITA")
    ]

    for map_name, record_root, approach_name in exp_records:
        if record_root != "" and Path(record_root).exists():
            start = time.perf_counter()
            # compute_coverage(map_name, Path(record_root))
            localization_msgs(record_root, map_name)
            minutes = (time.perf_counter() - start) / 60
            logger.info(f"Finished {map_name} {approach_name} in {minutes:.2f} minutes")
