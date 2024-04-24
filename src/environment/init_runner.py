import os
from config import MY_SCRIPTS_DIR, DIR_ROOT, PRE_SCRIPTS_DIR
from tools.file_handling import move_file


class InitRunner:
    def __init__(self):
        self.scripts_preprocessing("run_scenario.sh")
        self.scripts_preprocessing(".bashrc")
        self.scripts_preprocessing("run_bashrc.sh")
        self.move_scripts()

    def move_scripts(self):
        if not os.path.exists(f"{MY_SCRIPTS_DIR}"):
            os.mkdir(f"{MY_SCRIPTS_DIR}")
        file_list = os.listdir(f"{PRE_SCRIPTS_DIR}")
        for file_name in file_list:
            if "dev.env" not in file_name:
                move_file(f"{PRE_SCRIPTS_DIR}/{file_name}", f"{MY_SCRIPTS_DIR}/{file_name}")

    def scripts_preprocessing(self, file_name):
        with open(f"{PRE_SCRIPTS_DIR}/{file_name}", "r") as f:
            lines = f.read()
        with open(f"{PRE_SCRIPTS_DIR}/{file_name}", "w") as f:
            f.write(lines.replace("/home/cloudsky", DIR_ROOT))


if __name__ == '__main__':
    InitRunner()
