#!/bin/bash
ps -ef | grep -E 'emergency_handl|scenario_test_runner|simple_sensor_simulator|openscenario_preprocessor|openscenario_visualization|/home/lori/Desktop|/opt/ros/humble' | grep -v 'grep' | awk '{print $2}' | xargs kill -9
pkill -P $$
pkill -P 1
pkill -P 0

