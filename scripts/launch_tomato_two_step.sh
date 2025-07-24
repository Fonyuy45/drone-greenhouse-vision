#!/bin/bash

# Two-Step Launch Script - Fixes PX4 Bridge Connection

echo "=========================================="
echo "  PX4 TOMATO FARM - TWO STEP LAUNCHER"
echo "=========================================="

if [ ! -d "build/px4_sitl_default" ]; then
    echo "ERROR: Please run this script from PX4-Autopilot directory"
    exit 1
fi

# Clean up any existing processes
echo "🧹 Cleaning up existing processes..."
pkill -f "gz sim" 2>/dev/null
pkill -f "px4" 2>/dev/null
sleep 3

# Set environment variables - UPDATED PATHS
export GZ_SIM_RESOURCE_PATH=$HOME/.gz/models:$HOME/drone_farm_vision_ws/models:$HOME/PX4-Autopilot/Tools/simulation/gz/models
echo "✓ Environment set"
echo "  - Resource path: $GZ_SIM_RESOURCE_PATH"

# Step 1: Launch Gazebo server without GUI first
echo ""
echo "🌱 STEP 1: Starting Gazebo server..."
gz sim -r -s --verbose=1 $HOME/PX4-Autopilot/Tools/simulation/gz/worlds/tomato_farm_px4_complete.sdf &
GAZEBO_SERVER_PID=$!
echo "⏳ Waiting for Gazebo server to initialize (5 seconds)..."
sleep 5

# Step 2: Launch Gazebo GUI to connect to server
echo ""
echo "🖥️ STEP 2: Starting Gazebo GUI..."
gz sim -g &
GAZEBO_GUI_PID=$!
echo "⏳ Waiting for GUI connection (3 seconds)..."
sleep 3

# Step 3: Spawn drone manually
echo ""
echo "🚁 STEP 3: Spawning drone..."
SPAWN_RESULT=$(gz service -s /world/tomato_farm/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 10000 \
  --req "sdf_filename: 'x500_depth', pose: {position: {x: 7.0, y: 3.5, z: 3.0}}, name: 'x500_depth_0'")

if [[ $SPAWN_RESULT == *"true"* ]]; then
    echo "✅ Drone spawned successfully!"
else
    echo "⚠️ Drone spawn may have failed"
fi

# Step 4: Connect PX4 to existing Gazebo
echo ""
echo "🔗 STEP 4: Connecting PX4 to Gazebo..."
# Start PX4 in connection mode (not launching its own Gazebo)
PX4_SYS_AUTOSTART=4002 \
GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH \
./build/px4_sitl_default/bin/px4 -d &
PX4_PID=$!

echo ""
echo "🎉 LAUNCH COMPLETE!"
echo ""
echo "📍 Process Status:"
echo "  - Gazebo Server: PID $GAZEBO_SERVER_PID"
echo "  - Gazebo GUI: PID $GAZEBO_GUI_PID"  
echo "  - PX4 SITL: PID $PX4_PID"
echo ""
echo "🎮 Next Steps:"
echo "  1. Test keyboard control: python ~/drone_farm_vision_ws/scripts/keyboard-mavsdk-test.py"
echo "  2. Test vision detection: python ~/drone_farm_vision_ws/scripts/final_tomato_detection.py"
echo "  3. Camera bridge: ros2 run ros_gz_image image_bridge /camera"
echo ""
echo "⚠️ To stop: kill $GAZEBO_SERVER_PID $GAZEBO_GUI_PID $PX4_PID"

# Keep script running
wait
