import time
import multiprocessing as mp
from pathlib import Path
from typing import Tuple, Optional, List, Any

from loguru import logger
from autoware.rosbag_reader import ROSBagReader

from config import PROJECT_ROOT
import pickle


def similar(record_a, record_b, theta=1.0, FAST=10):
    for i in range(0, min(len(record_a), len(record_b)), FAST):
        if i > len(record_a) - 1 or i > len(record_b) - 1:
            break

        delta_time = record_a[i][2] - record_b[i][2]
        i1 = i
        i2 = i
        while abs(delta_time) > 2.3 * 1e7:
            if delta_time > 0:
                i2 += 1
            else:
                i1 += 1
            if i1 > len(record_a) - 1 or i2 > len(record_b) - 1:
                break
            delta_time = record_a[i1][2] - record_b[i2][2]
        i1 = min(i1, len(record_a) - 1)
        i2 = min(i2, len(record_b) - 1)
        x1, y1 = record_a[i1][0], record_a[i1][1]
        x2, y2 = record_b[i2][0], record_b[i2][1]
        if ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5 > theta:
            return False
    return True


def store_xy_time(
        task_queue: "mp.Queue[Optional[Tuple[int, int, Path]]]",
        result_queue: "mp.Queue[Optional[List[Any]]]"
):
    while True:
        record_path = task_queue.get()
        if record_path is None:
            break
        logger.info(f"Processing {record_path[2].name}")
        record = ROSBagReader(str(record_path[2]))
        if not record.has_routing_msg():
            logger.warning(f"Record {record_path[2].name} does not have routing message")
            result_queue.put([record_path[2].name, False, "No routing messages"])
            continue

        coords = []
        start = -1

        for _, message, t in record.read_specific_messages("/localization/kinematic_state"):
            coord = (message.pose.pose.position.x, message.pose.pose.position.y)
            if coord == (0.0, 0.0):
                continue
            if start == -1:
                start = t
            coords.append((coord[0], coord[1], t - start))

        ego_coordinates_set = set([(x, y) for x, y, _ in coords])
        if len(ego_coordinates_set) < 2:
            logger.warning(f"Record {record_path[2].name} has less than 2 coordinates")
            result_queue.put([record_path[2].name, False, "Less than 2 coords"])
            continue

        with open(record_path[2] / "ego_xy_time.pkl", "wb") as f:
            pickle.dump(coords, f)

        logger.info(f"Finished {record_path[2].name}")
        result_queue.put([record_path[2].name, True, ""])


def xy_time_msgs(record_root: Path):
    import pandas as pd
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
            store_xy_time,
            [(task_queue, result_queue) for _ in range(worker_num)],
        )
        pool.close()
        pool.join()

        results = []
        while not result_queue.empty():
            results.append(result_queue.get())

        logger.info(len(results))


def read_pickle_file(pickle_path: Path):
    return pickle.load(pickle_path.open('rb'))


def compare_similarity(record_path: Path):
    graphs = dict()
    pickle_files = list(sorted(record_path.rglob("ego_xy_time.pkl")))

    n = len(pickle_files)
    for i in range(n):
        graphs[i] = []

    for i in range(n):
        pickle_i = read_pickle_file(pickle_files[i])
        for j in range(i + 1, n):
            pickle_j = read_pickle_file(pickle_files[j])
            result = similar(pickle_i, pickle_j)
            if result:
                graphs[i].append(j)
                graphs[j].append(i)

    for i in range(n):
        print(f"{i}: {pickle_files[i].parent.name}")
    print(graphs)
    return graphs


def unique_scenarios(d: dict):
    uniques = set()
    discard = set()
    for k, v in d.items():
        if k not in discard:
            uniques.add(k)

        for x in v:
            if x not in uniques:
                discard.add(x)
    # shalun, awf, hsinchu, nishi-shijuku
    print(len(uniques))  # 413, 257, 367, 142
    print(len(discard))  # 271, 40, 245, 22


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

    ### after bug fix
    scenoRITA_nishi_8hrs = Path(
        fr"{PROJECT_ROOT}/out/0509_111556_Nishi-Shinjuku/records"
    )

    scenoRITA_awf_8hrs = Path(
        fr"{PROJECT_ROOT}/out/0509_224655_awf_cicd_virtualmap/records"
    )

    exp_records = [
        ("Shalun with road shoulders", scenoRITA_shalun_path, "scenoRITA"),
        ("Nishi-Shinjuku", scenoRITA_nishi_path, "scenoRITA"),
        ("Hsinchu city (Taiwan)", scenoRITA_hsinchu_path, "scenoRITA"),
        ("awf_cicd_virtualmap", scenoRITA_awf_cicd_virtualmap, "scenoRITA")
    ]

    for map_name, record_root, approach_name in exp_records:
        if record_root != "" and Path(record_root).exists():
            start = time.perf_counter()
            xy_time_msgs(record_root)  # todo: first, nohup
            unique_scenarios(compare_similarity(record_root))  # todo: second
            minutes = (time.perf_counter() - start) / 60
            logger.info(f"Finished {map_name} {approach_name} in {minutes:.2f} minutes")
