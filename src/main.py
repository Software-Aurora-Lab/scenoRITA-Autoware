import multiprocessing as mp
import shutil
import threading
from pathlib import Path
from time import perf_counter
from typing import Dict, List

from absl import app
from absl.flags import FLAGS
from loguru import logger

from autoware.map_service import load_map_service
from environment.container import Container
from config import ADS_ROOT, DOCKER_CONTAINER_NAME, DEFAULT_SCRIPT_PORT, CONTAINER_NUM, PROJECT_ROOT
from mylib.clustering import cluster
from mylib.workers import analysis_worker, generator_worker, player_worker
from prepare import init_prepare
from scenoRITA.components.grading_metrics import GradingResult
from scenoRITA.components.scenario_generator import ScenarioGenerator
from scenoRITA.operators import GeneticOperators
from autoware.open_scenario import OpenScenario
from scenoRITA.representation import ObstacleFitness
from utils import get_output_dir, set_up_gflags, set_up_logging


def evaluate_scenarios(
        containers: List[Container], scenarios: List[OpenScenario]
) -> int:
    num_evaluated = 0
    num_workers = 5
    multi_process_generate = True
    with mp.Manager() as manager:
        # set up queues
        pending_queue = manager.Queue()
        play_queue = manager.Queue()
        analysis_queue = manager.Queue()
        result_queue: "mp.Queue[GradingResult]" = manager.Queue()  # type: ignore
        for scenario in scenarios:
            pending_queue.put(scenario)
        for _ in range(num_workers):
            pending_queue.put(None)

        # set up processes
        if multi_process_generate:
            generator_processes = [
                threading.Thread(
                    target=generator_worker,
                    args=(
                        logger,
                        pending_queue,
                        play_queue,
                        get_output_dir(),
                    ),
                )
                for _ in range(num_workers)
            ]
        else:
            generator_processes = []

        player_processes = [
            threading.Thread(
                target=player_worker,
                args=(
                    containers[x],
                    logger,
                    play_queue,
                    analysis_queue,
                    get_output_dir(),
                    FLAGS.dry_run,
                ),
            )
            for x in range(len(containers))
        ]
        analyzer_processes = [
            mp.Process(
                target=analysis_worker,
                args=(
                    logger,
                    analysis_queue,
                    result_queue,
                    get_output_dir(),
                    FLAGS.dry_run,
                ),
            )
            for _ in range(num_workers)
        ]

        # start processes
        for p in generator_processes + player_processes + analyzer_processes:
            p.start()

        if not multi_process_generate:
            for scenario in scenarios:
                target_dir = get_output_dir()
                target_file = Path(target_dir, "input")
                target_file.parent.mkdir(parents=True, exist_ok=True)
                logger.info(f"{scenario.get_id()}: generate start")
                scenario.export_to_file(target_file)
                logger.info(f"{scenario.get_id()}: generate end")
                play_queue.put(scenario)

        # wait for processes to finish
        for p in generator_processes:
            p.join()
        for _ in range(len(player_processes)):
            play_queue.put(None)
        for p in player_processes:
            p.join()
        for _ in range(len(analyzer_processes)):
            analysis_queue.put(None)
        for p in analyzer_processes:
            p.join()

        # retrieve results
        results: Dict[str, GradingResult] = dict()
        while not result_queue.empty():
            grading_result = result_queue.get()
            results[grading_result.scenario_id] = grading_result

        # update fitness values
        for scenario in scenarios:
            if scenario.get_id() not in results:
                # scenario was not evaluated
                logger.error(f"{scenario.get_id()} was not evaluated")
                continue
            num_evaluated += 1
            grading_result = results[scenario.get_id()]
            for obs in scenario.obstacles:
                if grading_result.fitnesses.__contains__(obs.id):
                    obs.fitness.values = grading_result.fitnesses[obs.id]
                else:
                    obs.fitness.values = ObstacleFitness.get_fallback_fitness()

        # copy records with violations to a separate folder
        for sce_id in results:
            violations_dir = Path(get_output_dir(), "violations")
            violations_dir.mkdir(parents=True, exist_ok=True)
            for violation in results[sce_id].violations:
                # copy record to violations folder
                shutil.copy2(results[sce_id].record, violations_dir)
                violation_csv = Path(violations_dir, f"{violation.main_type}.csv")
                if not violation_csv.exists():
                    with open(violation_csv, "w") as f:
                        header_row = ",".join(violation.features.keys())
                        f.write(f"sce_id,{header_row}\n")
                with open(violation_csv, "a") as f:
                    feature_row = ",".join(map(str, violation.features.values()))
                    f.write(f"{sce_id},{feature_row}\n")
    return num_evaluated


def start_containers() -> List[Container]:
    containers = [Container(ADS_ROOT, PROJECT_ROOT, f'{DOCKER_CONTAINER_NAME}_{x}', str(x), DEFAULT_SCRIPT_PORT + x) for
                  x in range(CONTAINER_NUM)]
    for ctn in containers:
        ctn.start_instance()
        ctn.env_init()
        ctn.setup_env()
    return containers


def main(argv):
    del argv

    init_prepare()

    containers = start_containers()

    set_up_logging(FLAGS.log_level)
    logger.info("Execution ID: " + FLAGS.execution_id)
    logger.info("Map: " + FLAGS.map)
    logger.info("Scenario per Generation: " + str(FLAGS.num_scenario))
    logger.info("Length of experiment: {}h", FLAGS.num_hour)
    logger.info("Obstacle Range: {} - {}", FLAGS.min_obs, FLAGS.max_obs)

    # loading map service
    logger.info(f"Loading map service for {FLAGS.map}")
    map_service = load_map_service(FLAGS.map)

    # genetic algorithm main loop
    scenario_generator = ScenarioGenerator(map_service)
    genetic_operators = GeneticOperators(
        map_service,
        FLAGS.mut_pb,
        FLAGS.cx_pb,
        FLAGS.add_pb,
        FLAGS.del_pb,
        FLAGS.replace_pb,
        FLAGS.min_obs,
        FLAGS.max_obs,
        FLAGS.dry_run,
    )

    ga_start_time = perf_counter()
    expected_end_time = ga_start_time + FLAGS.num_hour * 3600

    scenarios = [
        scenario_generator.generate_scenario(0, x, FLAGS.min_obs, FLAGS.max_obs)
        for x in range(FLAGS.num_scenario)
    ]
    evaluate_scenarios(containers, scenarios)

    generation_counter = 1
    failure_counter = 0
    while perf_counter() < expected_end_time:
        logger.info(f"Generation {generation_counter}: start")
        offsprings = genetic_operators.get_offsprings(scenarios)
        logger.info(f"Generation {generation_counter}: mut/cx done")
        num_evaluated = evaluate_scenarios(containers, offsprings)
        if num_evaluated < len(offsprings):
            logger.error(
                f"Generation {generation_counter}: not all scenarios evaluated"
            )
            failure_counter += 1
            if failure_counter >= 3:
                logger.error(f"Generation {generation_counter}: Restarting containers.")
                for ctn in containers:
                    ctn.start_instance(True)
        else:
            failure_counter = 0
        logger.info(f"Generation {generation_counter}: evaluation done")
        scenarios = genetic_operators.select(scenarios, offsprings)
        logger.info(f"Generation {generation_counter}: selection done")
        logger.info(f"Generation {generation_counter}: end")

        generation_counter += 1
        if FLAGS.dry_run:
            break

    total_hour = (perf_counter() - ga_start_time) / 3600
    logger.info(f"Total time: {total_hour:.2f}h")

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

    csv_files = sorted(
        Path(get_output_dir(), "violations").glob("*.csv"),
        key=lambda x: get_priority(x.name.split(".")[0]),
    )

    for csv_file in csv_files:
        violation_name = csv_file.name[:-4]
        clustered_df = cluster(csv_file)
        num_violation = len(clustered_df)
        num_cluster = len(clustered_df["cluster"].unique())
        logger.info(
            f"{violation_name}: {num_violation} violations in {num_cluster} clusters"
        )


if __name__ == "__main__":
    set_up_gflags()
    try:
        app.run(main)
    except Exception as e:
        logger.exception(e)
        raise e
