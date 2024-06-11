# scenoRITA for Autoware Universe v1.0

This is an implementation of scenoRITA that supports Autoware Universe v1.0.

## Prerequisites

1. Ubuntu 22.04 LTS
2. [Docker CE](https://docs.docker.com/engine/install/ubuntu/)
3. [Python Poetry](https://python-poetry.org/)
4. [Python 3.10.12](https://www.python.org/downloads/release/python-31012/)
5. [ROS2 Humble](https://docs.ros.org/en/humble/index.html)

## File Structure

```
|--DIR_ROOT
    |--autoware
        |--src
        |--install
        |...
    |--autoware_maps
    |--reduced_autoware
    |--scenoRITA-Autoware
        |--data
        |--src
```

## Installation

0. under `DIR_ROOT`, clone the repository, and checkout to `release/v1.0` branch
   ```
   git clone https://github.com/autowarefoundation/autoware.git
   ```

1. under `DIR_ROOT`, clone the repository
   ```
   git clone https://github.com/lethal233/scenoRITA-Autoware.git
   ```

2. On your host machine, run the following settings:
   ```bash
   sudo sysctl -w net.core.rmem_max=2147483647
   
   sudo sysctl -w net.ipv4.ipfrag_time=3
   
   sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728
   ```
3. Pull the image if you haven't done it before
   ```bash
   docker pull ghcr.io/autowarefoundation/autoware-universe:humble-1.0-cuda-amd64
   ```
4. Run the following command to start a `init` docker container:
   ```bash
   rocker --nvidia --x11 --user --name scenoRITA_autoware_0 --network c0 --privileged --volume /path/to/autoware --volume /path/to/autoware_maps --volume /path/to/scenoRITA-Autoware -- ghcr.io/autowarefoundation/autoware-universe:humble-1.0-cuda-amd64
   ```
5. Fill in the docker image id (`DOCKER_IMAGE_ID`) in the `scenoRITA-Autoware/src/config.py` with the docker image id of
   the `scenoRITA_autoware_0` container.

6. Inside the `scenoRITA_autoware_0` container, run the following command to install Autoware (if you haven't done it
   before):
   ```bash
   cd /path/to/autoware
   mkdir src
   vcs import src < autoware.repos
   vcs import src < simulator.repos
   ```
7. Inside the `scenoRITA_autoware_0` container, under `/path/to/autoware`,run the following command to install
   dependencies:
   ```bash
   sudo apt update
   rosdep update
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```
8. Inside the `scenoRITA_autoware_0` container, under `/path/to/autoware`, run the following command to build Autoware (if you haven't done it before):
   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
9. Inside the `scenoRITA_autoware_0` container, under `/path/to/autoware`, run the following command to source the workspace:
   ```bash
   sudo ip link set lo multicast on
   source ~/.bashrc
   source install/setup.bash
   ```
10. Download the maps [here](https://drive.google.com/file/d/1HBBDCtSubJ7pJX6acTaAbWP0COYXCnMs/view?usp=sharing) and de-compress it under `${ADS_MAP_DIR}`, which is defined in `config.py`
11. On your host machine, run the following command to download maps and move scripts:
    ```bash
    python3 /path/to/scenoRITA_Autoware/src/prepare.py
    ```

## Running scenoRITA for Autoware

1. Install project dependencies via command (if you haven't done it before)
   ```
   poetry install
   ```

2. source environments, please note that you need to source the file based on your terminal (bash, zsh, etc.). For example, if you are using bash, you can run the following commands:
   ```bash
   source /path/to/ros/humble/setup.bash
   source /path/to/reduced_autoware/install/setup.bash
   ```
   

3. Run scenoRITA via command
   ```
   poetry run python3 src/main.py
   ```

   > if you want to run scenoRITA on different maps (e.g., san_mateo), you can add `--map=san_mateo` to command. But
   make sure that you add the map to `autoware_maps/{map_name}` directory.
   > If you want to modify any configuration, you can modify `src/config.py` or `src/utils.py` file.

After running the command, the output of scenoRITA will be stored under `out/{execution_id}` and you can find violations
detected under `out/{execution_id}/violations`

## NOTES
See [notes](./NOTES.md)
