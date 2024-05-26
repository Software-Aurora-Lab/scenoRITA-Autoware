import time
import subprocess
from pathlib import Path

from autoware.open_scenario import OpenScenario
from config import MAX_RECORD_TIME, MY_SCRIPTS_DIR, AUTOWARE_CMD_PREPARE_TIME, TMP_RECORDS_DIR
from utils import get_output_dir


def replay_scenario(scenario: OpenScenario, container):
    container.kill_process()
    start_replay(scenario, container)
    move_scenario_record_dir(scenario, container)


def move_scenario_record_dir(scenario: OpenScenario, container):
    target_output_path = Path(get_output_dir(), "records")
    target_output_path.mkdir(parents=True, exist_ok=True)
    cmd = f'docker exec {container.container_name} mv {TMP_RECORDS_DIR}/{scenario.get_id()} "{target_output_path}"'
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    p.wait()


def start_replay(scenario: OpenScenario, container):
    script_path = container.scenario_script_update(scenario)
    cmd = f"docker exec --env-file {MY_SCRIPTS_DIR}/{container.env_file} {container.container_name} /bin/bash {script_path}"
    p = subprocess.Popen(cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time_start = time.time()
    while p.poll() is None:
        if MAX_RECORD_TIME + AUTOWARE_CMD_PREPARE_TIME < time.time() - time_start:
            container.stop_recorder()
            break
    time.sleep(0.5)
    container.kill_process()
    p.wait()
