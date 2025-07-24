#!/bin/bash

# Source ROS2
source /opt/ros/humble/setup.bash

# Source this workspace (after building)
if [ -f ~/drone_farm_vision_ws/install/setup.bash ]; then
    source ~/drone_farm_vision_ws/install/setup.bash
    echo "✅ Main workspace sourced successfully"
else
    echo "⚠️  Main workspace not built yet. Run 'colcon build' first."
    # Fallback to separate workspaces during transition
    [ -f ~/drone_farm_vision_ws/sensor_ws/install/setup.bash ] && source ~/drone_farm_vision_ws/sensor_ws/install/setup.bash
    [ -f ~/drone_farm_vision_ws/offboard_ws/install/setup.bash ] && source ~/drone_farm_vision_ws/offboard_ws/install/setup.bash
fi

# Activate Python virtual environment
source ~/drone_farm_vision_ws/dfv_env/bin/activate

# Set environment variables
export GZ_SIM_RESOURCE_PATH=~/.gz/models:$(pwd)/models
export PYTHONPATH="${PYTHONPATH}:$(pwd)/scripts"

# Add PX4 to PATH if available
if [ -d "$HOME/PX4-Autopilot/build/px4_sitl_default/bin" ]; then
    export PATH="${PATH}:$HOME/PX4-Autopilot/build/px4_sitl_default/bin"
fi

echo "✅ Environment configured for drone_farm_vision_ws"
echo "Virtual environment: $(which python)"
echo "ROS2 packages: $(ros2 pkg list | grep -E '(px4)' | wc -l) px4 packages available"
