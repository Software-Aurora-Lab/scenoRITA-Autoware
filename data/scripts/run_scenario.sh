#!/bin/bash
cd /home/sora/autoware
source install/setup.bash
ros2 launch scenario_test_runner scenario_test_runner.launch.py \
  architecture_type:=awf/universe \
  record:=true \
  port:=5555 \
  scenario:=/home/sora/autoware_map/autoware_scenario_data/scenarios/0a43b704-1789-4369-a65a-ee5ee44bd169.yaml \
  sensor_model:=sample_sensor_kit \
  vehicle_model:=sample_vehicle