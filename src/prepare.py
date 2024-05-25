import os
import zipfile
from pathlib import Path

import gdown

from config import MY_SCRIPTS_DIR, DIR_ROOT, ADS_ROOT, PRE_SCRIPTS_DIR, PROJECT_ROOT
from tools.file_handling import move_file


def download_maps():
    zip_file = gdown.download(id='1aDC6cnWzmB-6KnvF_JxIcjSahe0EymB2')
    zip_file_path = Path(zip_file)

    output_dir_path = Path(f"{PROJECT_ROOT}/data/")

    with zipfile.ZipFile(zip_file, 'r') as zip_ref:
        zip_ref.extractall(output_dir_path)

    zip_file_path.unlink()


def move_scripts():
    if not os.path.exists(f"{MY_SCRIPTS_DIR}"):
        os.mkdir(f"{MY_SCRIPTS_DIR}")
    file_list = os.listdir(f"{PRE_SCRIPTS_DIR}")
    for file_name in file_list:
        if file_name not in ["dev.env", "requirements.sh"]:
            move_file(f"{PRE_SCRIPTS_DIR}/{file_name}", f"{MY_SCRIPTS_DIR}/{file_name}")


def scripts_preprocessing(file_name):
    with open(f"{PRE_SCRIPTS_DIR}/{file_name}", "r") as f:
        lines = f.read()
    with open(f"{PRE_SCRIPTS_DIR}/{file_name}", "w") as f:
        f.write(lines.replace(f"/home/lori/Desktop", DIR_ROOT))


def move_launch_file():
    os.system(
        f"cp -rf {PROJECT_ROOT}/data/config_files/planning_simulator.launch.xml {ADS_ROOT}/src/launcher/autoware_launch/autoware_launch/launch/planning_simulator.launch.xml")


def init_prepare():
    download_maps()
    scripts_preprocessing("run_scenario.sh")
    scripts_preprocessing(".bashrc")
    scripts_preprocessing("move_bashrc.sh")
    scripts_preprocessing("extract_env.sh")
    scripts_preprocessing("kill_process.sh")
    scripts_preprocessing("setup_env.sh")
    scripts_preprocessing("dev.env")

    move_scripts()
    move_launch_file()


if __name__ == '__main__':
    init_prepare()
