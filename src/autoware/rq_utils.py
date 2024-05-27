from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
import multiprocessing as mp

import yaml
from loguru import logger
from yaml import SafeLoader
from autoware.rosbag_reader import ROSBagReader
from config import PROJECT_ROOT


def get_routing_msg(
        task_queue: "mp.Queue[Optional[Tuple[int, int, Path]]]",
        result_queue: "mp.Queue[List[Any]]",
):
    while True:
        record_path = task_queue.get()
        if record_path is None:
            break
        logger.info(f"Processing {record_path[2].parent.name}")
        result = read_yaml(record_path[2])
        logger.info(f"Finished {record_path[2].parent.name}")
        if result:
            result_queue.put(result)


def get_duration(
        task_queue: "mp.Queue[Optional[Tuple[int, int, Path]]]",
        result_queue: "mp.Queue[float]",
):
    while True:
        record_path = task_queue.get()
        if record_path is None:
            break
        logger.info(f"Processing {record_path[2].parent.name}")
        result = ROSBagReader(str(record_path[2].parent)).get_duration()
        logger.info(f"Finished {record_path[2].parent.name}")
        if result:
            result_queue.put(result)


def read_yaml(file_path: Path):
    assert file_path.exists(), f"{file_path} does not exist"
    with open(file_path, 'r') as file:
        data = yaml.load(file, Loader=SafeLoader)

    topics: List[Dict[str, Any]] = data['rosbag2_bagfile_information']['topics_with_message_count']

    should_processed_topics = [
        '/localization/acceleration',
        '/localization/kinematic_state',
        '/perception/object_recognition/ground_truth/objects',
        '/perception/object_recognition/objects',
        '/planning/mission_planning/route',
    ]
    results = [-1, -1, -1, -1, -1]
    for topic in topics:
        ind = should_processed_topics.index(topic['topic_metadata']['name'])
        results[ind] = topic['message_count']
    return [file_path.parent.name, *results]


def analyze_msgs_csv(file: Path):
    import pandas as pd
    df = pd.read_csv(file)
    logger.info(f"{file.parent.parent.name} total: {df.shape[0]}")

    def no_rounting_msg_cnt():
        return df[df['routing'] <= 0].shape[0]

    def no_ground_truth_cnt():
        return df[df['ground_truth'] <= 0].shape[0]

    def no_perception_cnt():
        return df[df['perception'] <= 0].shape[0]

    def no_localization_cnt():
        return df[df['localization'] <= 0].shape[0]

    def no_acceleration_cnt():
        return df[df['acceleration'] <= 0].shape[0]

    n_r = no_rounting_msg_cnt()
    n_gt = no_ground_truth_cnt()
    n_p = no_perception_cnt()
    n_l = no_localization_cnt()
    n_a = no_acceleration_cnt()

    assert n_l == n_a
    logger.info(f"No routing msg: {n_r}")
    logger.info(f"No ground truth msg: {n_gt}")
    logger.info(f"No perception msg: {n_p}")

    logger.info(f"No localization msg: {n_l}")
    logger.info(f"No acceleration msg: {n_a}")

    def valid_msgs():
        return df[(df['acceleration'] > 0) &
                  (df['routing'] > 0) &
                  (df['ground_truth'] > 0) &
                  (df['perception'] > 0) &
                  (df['localization'] > 0)]

    valid_sce = valid_msgs()
    logger.info(f"valid msgs cnt (not filtering invalid localization): {valid_sce.shape[0]}")
    valid_sce.to_csv(Path(file.parent, "valid_msgs.csv"), index=False)


if __name__ == '__main__':
    # analyze_msgs_csv(Path("{PROJECT_ROOT}/out/0505_232108_Nishi-Shinjuku/records/msgs.csv"))
    # analyze_msgs_csv(Path("{PROJECT_ROOT}/out/0504_055929_Nishi-Shinjuku/records/msgs.csv"))
    # analyze_msgs_csv(Path("{PROJECT_ROOT}/out/0503_155808_Shalun with road shoulders/records/msgs.csv"))
    # analyze_msgs_csv(Path("{PROJECT_ROOT}/out/0503_024541_Hsinchu city (Taiwan)/records/msgs.csv"))
    # analyze_msgs_csv(Path("{PROJECT_ROOT}/out/0508_004054_awf_cicd_virtualmap/records/msgs.csv"))
    # analyze_msgs_csv(Path("{PROJECT_ROOT}/out/0509_111556_Nishi-Shinjuku/records/msgs.csv"))
    analyze_msgs_csv(
        Path(f"{PROJECT_ROOT}/out/0509_224655_awf_cicd_virtualmap/records/msgs.csv"))
