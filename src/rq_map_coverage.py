import time
import multiprocessing as mp
from pathlib import Path
from typing import Tuple, Optional, List, Any

from loguru import logger
from shapely.geometry import LineString
from autoware.rosbag_reader import ROSBagReader

from config import PROJECT_ROOT
import pickle
import pandas as pd


def store_localization_msg(
        task_queue: "mp.Queue[Optional[Tuple[int, int, Path]]]",
        result_queue: "mp.Queue[Optional[List[Any]]]"
):
    while True:
        record_path = task_queue.get()
        if record_path is None:
            break
        logger.info(f"Processing {record_path[2].name}")
        ego_coordinates: List[Tuple[float, float]] = list()
        record = ROSBagReader(str(record_path[2]))
        if not record.has_routing_msg():
            logger.warning(f"Record {record_path[2].name} does not have routing message")
            result_queue.put([record_path[2].name, False, "No routing messages"])
            continue

        for topic, msg, t in record.read_specific_messages("/localization/kinematic_state"):
            ego_coord = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            if ego_coord == (0.0, 0.0):
                continue
            if len(ego_coordinates) > 0 and ego_coordinates[-1] == ego_coord:
                continue
            ego_coordinates.append(ego_coord)
        if len(ego_coordinates) < 2:
            logger.warning(f"Record {record_path[2].name} has less than 2 coordinates")
            result_queue.put([record_path[2].name, False, "Less than 2 coords"])
            continue
        ego_lst = LineString(ego_coordinates)
        with open(record_path[2] / "ego_lst_debug.pkl", "wb") as f:
            pickle.dump(ego_lst, f)

        logger.info(f"Finished {record_path[2].name}")
        result_queue.put([record_path[2].name, True, ""])


def localization_msgs(record_root: Path):
    assert Path(record_root, "valid_msgs.csv").exists(), "please execute rq_utils.py first"
    to_be_processed_df = pd.read_csv(Path(record_root, "valid_msgs.csv"))
    record_file_list = to_be_processed_df['scenario'].to_list()
    records_fp = [Path(record_root, _, _) for _ in record_file_list]
    with mp.Manager() as manager:
        worker_num = mp.cpu_count()
        pool = mp.Pool(worker_num)
        task_queue = manager.Queue()
        result_queue = manager.Queue()
        for index, rc_fp in enumerate(records_fp):
            task_queue.put((0, index, rc_fp))
        for _ in range(worker_num):
            task_queue.put(None)

        pool.starmap(
            store_localization_msg,
            [(task_queue, result_queue) for _ in range(worker_num)],
        )
        pool.close()
        pool.join()

        results = []
        while not result_queue.empty():
            results.append(result_queue.get())

        pd.DataFrame(results, columns=['scenario', 'valid', 'reason']).to_csv(
            Path(record_root, "final_filtering.csv", index=False))
        logger.info(len(results))


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

    scenoRITA_nishi_path_2 = Path(
        fr"{PROJECT_ROOT}/out/0505_232108_Nishi-Shinjuku/records"
    )

    scenoRITA_awf_cicd_virtualmap = Path(
        fr"{PROJECT_ROOT}/out/0508_004054_awf_cicd_virtualmap/records"
    )

    scenoRITA_nishi_8hrs = Path(
        fr"{PROJECT_ROOT}/out/0509_111556_Nishi-Shinjuku/records"
    )

    scenoRITA_awf_8hrs = Path(
        fr"{PROJECT_ROOT}/out/0509_224655_awf_cicd_virtualmap/records/gen_7_sce_15"
    )

    exp_records = [
        ("Shalun with road shoulders", scenoRITA_shalun_path, "scenoRITA"),
        ("Nishi-Shinjuku", scenoRITA_nishi_path, "scenoRITA"),
        ("Nishi-Shinjuku", scenoRITA_nishi_path_2, "scenoRITA"),
        ("Hsinchu city (Taiwan)", scenoRITA_hsinchu_path, "scenoRITA"),
        ("awf_cicd_virtualmap", scenoRITA_awf_cicd_virtualmap, "scenoRITA"),

        ("Nishi-Shinjuku", scenoRITA_nishi_8hrs, "scenoRITA"),
        ("awf_cicd_virtualmap", scenoRITA_awf_8hrs, "scenoRITA")
    ]

    for map_name, record_root, approach_name in exp_records:
        if record_root != "" and Path(record_root).exists():
            start = time.perf_counter()
            localization_msgs(record_root)
            minutes = (time.perf_counter() - start) / 60
            logger.info(f"Finished {map_name} {approach_name} in {minutes:.2f} minutes")
