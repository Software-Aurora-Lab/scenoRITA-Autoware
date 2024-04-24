import multiprocessing as mp
import random
from logging import Logger
from pathlib import Path
from typing import Optional

from environment.container import Container
from autoware.open_scenario import OpenScenario
from scenario_handling.ScenarioReplayer import replay_scenario
from scenoRITA.components.grading_metrics import GradingResult, grade_scenario
from scenoRITA.representation import ObstacleFitness


def generator_worker(
    _logger: Logger,
    task_queue: "mp.Queue[Optional[OpenScenario]]",
    result_queue: mp.Queue,
    target_dir: Path,
):
    while True:
        scenario = task_queue.get()
        if scenario is None:
            break
        _logger.info(f"{scenario.get_id()}: generate start")

        target_file = Path(target_dir, "input", f"{scenario.get_id()}")
        target_file.parent.mkdir(parents=True, exist_ok=True)
        scenario.export_to_file(target_file)

        _logger.info(f"{scenario.get_id()}: generate end")
        result_queue.put(scenario)


def player_worker(
    container: Container,
    _logger: Logger,
    task_queue: "mp.Queue[Optional[OpenScenario]]",
    result_queue: "mp.Queue[Optional[OpenScenario]]",
    target_dir: Path,
    dry_run: bool,
):
    while True:
        scenario = task_queue.get()
        if scenario is None:
            break
        sce_id = scenario.get_id()
        _logger.info(f"{sce_id}: play start ({container.container_name})")

        target_output_path = Path(target_dir, "records", f"{sce_id}", f"{sce_id}", f"{sce_id}_0.db3")
        target_output_path.parent.mkdir(parents=True, exist_ok=True)
        if dry_run:
            with open(target_output_path, "w") as fp:
                fp.write("dry run")
        else:
            if not container.is_running():
                container.start_instance()
            replay_scenario(scenario, container)

        _logger.info(f"{sce_id}: play end")
        result_queue.put(scenario)


def analysis_worker(
    _logger: Logger,
    task_queue: "mp.Queue[Optional[OpenScenario]]",
    result_queue: "mp.Queue[GradingResult]",
    target_dir: Path,
    dry_run: bool,
):
    while True:
        scenario = task_queue.get()
        if scenario is None:
            break
        sce_id = scenario.get_id()
        _logger.info(f"{sce_id}: analysis start")

        target_input_file = list(Path(target_dir, "records", f"{sce_id}", f"{sce_id}").rglob("*.db3"))[0]
        assert target_input_file.exists()
        if dry_run:
            obs_ids = [obs.id for obs in scenario.obstacles]
            fitnesses = dict()
            for oid in obs_ids:
                fitnesses[oid] = tuple(
                    random.random() for _ in range(len(ObstacleFitness.weights))
                )
            result_queue.put(
                GradingResult(
                    scenario.get_id(),
                    target_input_file,
                    fitnesses,
                    [],
                )
            )
        else:
            grading_result = grade_scenario(
                scenario.get_id(), target_input_file
            )
            if grading_result is None:
                _logger.error(f"{sce_id}: grading failed after 3 retries.")
                fallback_fitness = dict()
                for obs in scenario.obstacles:
                    fallback_fitness[obs.id] = ObstacleFitness.get_fallback_fitness()
                result_queue.put(
                    GradingResult(sce_id, target_input_file, fallback_fitness, [])
                )
            else:
                result_queue.put(grading_result)

        _logger.info(f"{sce_id}: analysis end")
