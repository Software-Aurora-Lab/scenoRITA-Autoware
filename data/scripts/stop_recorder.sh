#!/bin/bash
ps -ef | grep -E 'ros2 bag record' | grep -v 'grep' | awk '{print $2}' | xargs kill -2