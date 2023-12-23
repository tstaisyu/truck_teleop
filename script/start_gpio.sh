#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/taisyu/ros2_ws/install/setup.bash
#ros2 run truck_teleop raspi_on_truck
RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{name}]: {message}' RCUTILS_LOGGING_BUFFERED_STREAM=1 RCUTILS_LOG_MIN_SEVERITY=DEBUG ros2 run truck_teleop raspi_on_truck
