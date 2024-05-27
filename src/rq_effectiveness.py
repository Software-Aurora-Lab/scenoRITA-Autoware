import multiprocessing as mp
import warnings
from pathlib import Path
from typing import List, Optional, Tuple

import pandas as pd
from absl import flags
from loguru import logger
from mylib.clustering import cluster
from scenoRITA.components.grading_metrics import GradingResult, grade_scenario
from utils import PROJECT_ROOT

warnings.filterwarnings("ignore")


def analyze_scenario(
        task_queue: "mp.Queue[Optional[Tuple[int, int, Path]]]",
        result_queue: "mp.Queue[GradingResult]",
) -> None:
    while True:
        record_path = task_queue.get()
        if record_path is None:
            break
        logger.info(f"Processing {record_path[2].name}")
        result = grade_scenario(record_path[2].name, record_path[2])
        logger.info(f"Finished {record_path[2].name}")
        if result:
            result_queue.put(result)


def check_violations(root: Path, map_name: Optional[str]) -> None:
    if len(list(root.glob("*.csv"))) > 0:
        # Already processed
        return
    assert map_name is not None, "Please specify map name"
    records = list(root.rglob("*.db3"))
    records = [x.parent for x in records]
    violation_dfs = dict()
    with mp.Manager() as manager:
        worker_num = mp.cpu_count()
        pool = mp.Pool(worker_num)
        task_queue = manager.Queue()
        result_queue = manager.Queue()
        for index, record_path in enumerate(records):
            task_queue.put((0, index, record_path))
        for _ in range(worker_num):
            task_queue.put(None)

        pool.starmap(
            analyze_scenario,
            [(task_queue, result_queue) for _ in range(worker_num)],
        )
        pool.close()

        results: List[GradingResult] = []
        while not result_queue.empty():
            results.append(result_queue.get())

        for r in results:
            for v in r.violations:
                if v.main_type not in violation_dfs:
                    violation_dfs[v.main_type] = pd.DataFrame(
                        columns=["sce_id"] + list(v.features.keys())
                    )
                target_df = violation_dfs[v.main_type]
                target_df.loc[len(target_df)] = [r.scenario_id] + list(
                    v.features.values()
                )
        for vdf in violation_dfs:
            violation_dfs[vdf].to_csv(Path(root, f"{vdf}.csv"), index=False)


def cluster_violations(root: Path) -> None:
    # summarize results
    priority = {
        "Collision": 1,
        "Speeding": 2,
        "FastAccel": 3,
        "HardBraking": 4,
        "UnsafeLaneChange": 5
    }

    def get_priority(event):
        assert event in priority
        return priority.get(event)

    violation_csvs = list(root.glob("*.csv"))

    if len(violation_csvs) == 0:
        print("No violations found")

    a = []

    for vc in violation_csvs:
        if '_out' in str(vc):
            continue
        a.append(vc)
    violation_csvs = a
    violation_csvs.sort(key=lambda x: get_priority(x.name.split(".")[0]))
    for csv_file in violation_csvs:
        violation_name = csv_file.name[:-4]
        clustered_df = cluster(csv_file)
        num_violations = len(clustered_df)
        num_clusters = len(clustered_df["cluster"].unique())
        print(violation_name, num_violations, num_clusters)
        clustered_df.to_csv(Path(root, f"{violation_name}_out.csv"))


def main(args) -> None:
    del args
    root_dir = Path(flags.FLAGS.dir)
    assert root_dir.exists(), f"{root_dir} does not exist"
    cluster_violations(root_dir)


if __name__ == "__main__":
    scenoRITA_shalun_path = Path(
        fr"{PROJECT_ROOT}/out/0503_155808_Shalun with road shoulders/violations"
    )
    scenoRITA_nishi_path = Path(
        fr"{PROJECT_ROOT}/out/0504_055929_Nishi-Shinjuku/violations"
    )
    scenoRITA_hsinchu_path = Path(
        fr"{PROJECT_ROOT}/out/0503_024541_Hsinchu city (Taiwan)/violations"
    )

    scenoRITA_nishi_path_2 = Path(
        fr"{PROJECT_ROOT}/out/0505_232108_Nishi-Shinjuku/violations"
    )

    scenoRITA_awf_cicd_virtualmap = Path(
        fr"{PROJECT_ROOT}/out/0508_004054_awf_cicd_virtualmap/violations"
    )

    ##############
    scenoRITA_nishi_8hrs = Path(
        fr"{PROJECT_ROOT}/out/0509_111556_Nishi-Shinjuku/violations"
    )

    scenoRITA_awf_8hrs = Path(
        fr"{PROJECT_ROOT}/out/0509_224655_awf_cicd_virtualmap/violations"
    )

    exp_records = [
        ("Shalun with road shoulders", scenoRITA_shalun_path, "scenoRITA"),
        ("Nishi-Shinjuku", scenoRITA_nishi_path, "scenoRITA"),
        ("Nishi-Shinjuku", scenoRITA_nishi_path_2, "scenoRITA"),
        ("Hsinchu city (Taiwan)", scenoRITA_hsinchu_path, "scenoRITA"),
        ("awf_cicd_virtualmap", scenoRITA_awf_cicd_virtualmap, "scenoRITA"),

        ("Nishi-Shinjuku", scenoRITA_nishi_8hrs, "scenoRITA"),  # todo: only keep one
        ("awf_cicd_virtualmap", scenoRITA_awf_8hrs, "scenoRITA")
    ]

    for _, p, _ in exp_records:
        cluster_violations(p)
