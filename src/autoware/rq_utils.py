from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
import multiprocessing as mp

import yaml
from loguru import logger
from yaml import SafeLoader


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
