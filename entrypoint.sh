#!/bin/bash

# Source ROS2 installation
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build --symlink-install

# Source the workspace
source /ws/install/setup.bash

# Execute the passed command
exec "$@"