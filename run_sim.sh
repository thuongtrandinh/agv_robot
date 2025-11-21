#!/bin/bash

# Script to launch AGV simulation with localization and trajectory tracking
# Author: Auto-generated for ROS2 AGV project
# Date: November 3, 2025

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Array to store all background process PIDs
declare -a PIDS
declare -a TERMINAL_PIDS

# Cleanup function to kill all processes
cleanup() {
    echo ""
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}Cleaning up processes...${NC}"
    echo -e "${RED}========================================${NC}"
    
    # Kill all spawned terminal processes
    echo -e "${YELLOW}Stopping gnome-terminal processes...${NC}"
    for pid in "${TERMINAL_PIDS[@]}"; do
        if kill -0 $pid 2>/dev/null; then
            echo -e "${YELLOW}  Killing terminal PID: $pid${NC}"
            kill -9 $pid 2>/dev/null
            # Kill all children of this terminal
            pkill -9 -P $pid 2>/dev/null
        fi
    done
    
    # Kill Gazebo
    echo -e "${YELLOW}Stopping Gazebo...${NC}"
    killall -9 gzserver gzclient gazebo gz ruby 2>/dev/null
    pkill -9 -f "gz sim" 2>/dev/null
    pkill -9 -f "ros_gz" 2>/dev/null
    
    # Kill all ROS2 launch files
    echo -e "${YELLOW}Stopping ROS2 launch files...${NC}"
    pkill -9 -f "launch_sim.launch.py" 2>/dev/null
    pkill -9 -f "global_localization.launch.py" 2>/dev/null
    pkill -9 -f "trajectory_tracking.launch.py" 2>/dev/null
    
    # Kill specific ROS2 nodes
    echo -e "${YELLOW}Stopping ROS2 nodes...${NC}"
    pkill -9 -f "robot_state_publisher" 2>/dev/null
    pkill -9 -f "ros_gz_bridge" 2>/dev/null
    pkill -9 -f "parameter_bridge" 2>/dev/null
    pkill -9 -f "trajectory_publisher" 2>/dev/null
    pkill -9 -f "fuzzy_trajectory_controller" 2>/dev/null
    pkill -9 -f "trajectory_plotter" 2>/dev/null
    pkill -9 -f "map_server" 2>/dev/null
    pkill -9 -f "amcl" 2>/dev/null
    pkill -9 -f "lifecycle_manager" 2>/dev/null
    pkill -9 -f "controller_manager" 2>/dev/null
    pkill -9 -f "joint_broad" 2>/dev/null
    pkill -9 -f "diff_cont" 2>/dev/null
    
    # Kill matplotlib windows
    echo -e "${YELLOW}Stopping matplotlib windows...${NC}"
    pkill -9 -f "matplotlib" 2>/dev/null
    pkill -9 -f "python3.*plotter" 2>/dev/null
    
    # Clean up shared memory
    echo -e "${YELLOW}Cleaning up shared memory...${NC}"
    rm -rf /dev/shm/gz-* 2>/dev/null
    rm -rf /dev/shm/fastrtps* 2>/dev/null
    
    # Note: Do NOT clean up temporary scripts here - let them persist for terminal windows
    # They will be cleaned up by system /tmp cleaner or on next reboot
    
    sleep 2
    
    # Verify cleanup
    REMAINING=$(pgrep -f "gzserver|gzclient|ros2_ws.*ros2 launch" 2>/dev/null | wc -l)
    if [ $REMAINING -eq 0 ]; then
        echo -e "${GREEN}✓ All processes stopped successfully!${NC}"
    else
        echo -e "${YELLOW}⚠ Warning: $REMAINING processes may still be running${NC}"
        echo -e "${YELLOW}Run ./kill_sim.sh for thorough cleanup${NC}"
    fi
    
    echo -e "${GREEN}Cleanup complete!${NC}"
    echo ""
    
    # Don't exit - return to allow terminal to stay open
    trap - SIGINT SIGTERM EXIT
    return 0
}

# Trap Ctrl+C (SIGINT) and SIGTERM only (not EXIT to avoid double cleanup)
trap cleanup SIGINT SIGTERM

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}AGV Simulation Launch Script${NC}"
echo -e "${GREEN}========================================${NC}"

# Source ROS2 workspace
echo -e "${YELLOW}Sourcing ROS2 workspace...${NC}"
cd ~/ros2_ws
source install/setup.bash

# Default parameters
X_POS=0.0
Y_POS=0.0
WORLD="small_house.world"
MAP_NAME="small_house"
TRAJECTORY_TYPE=2  # 1=Circle, 2=Square, 3=Figure-8
RADIUS=2.0  # Circle radius in meters
TRAJECTORY_SPEED=0.33  # m/s - Trajectory reference speed (OPTIMIZED for Figure-8 smooth flow: 0.33)
MAX_LINEAR_VEL=0.38  # m/s - Maximum robot velocity (TUNED for Figure-8 smooth tracking: 0.38)
ENABLE_TRAJ_PUBLISH="true"  # Enable/disable trajectory publishing (default: true)
VERBOSE_LOGGING="true"  # Enable detailed velocity logging (default: true)

# Parse command line arguments (optional)
while [[ $# -gt 0 ]]; do
    case $1 in
        -x|--x-pos)
            X_POS="$2"
            # Ensure float format (add .0 if integer)
            if [[ "$X_POS" =~ ^-?[0-9]+$ ]]; then
                X_POS="${X_POS}.0"
            fi
            shift 2
            ;;
        -y|--y-pos)
            Y_POS="$2"
            # Ensure float format (add .0 if integer)
            if [[ "$Y_POS" =~ ^-?[0-9]+$ ]]; then
                Y_POS="${Y_POS}.0"
            fi
            shift 2
            ;;
        -w|--world)
            WORLD="$2"
            # Auto-detect map name from world file
            if [[ "$WORLD" == "small_warehouse.world" ]]; then
                MAP_NAME="small_warehouse"
            elif [[ "$WORLD" == "room_20x20.world" ]]; then
                MAP_NAME="room_20x20"
            else
                MAP_NAME="small_house"
            fi
            shift 2
            ;;
        -t|--trajectory)
            TRAJECTORY_TYPE="$2"
            shift 2
            ;;
        -r|--radius)
            RADIUS="$2"
            # Ensure float format (add .0 if integer)
            if [[ "$RADIUS" =~ ^-?[0-9]+$ ]]; then
                RADIUS="${RADIUS}.0"
            fi
            shift 2
            ;;
        -s|--traj-speed)
            TRAJECTORY_SPEED="$2"
            # Ensure float format (add .0 if integer)
            if [[ "$TRAJECTORY_SPEED" =~ ^-?[0-9]+$ ]]; then
                TRAJECTORY_SPEED="${TRAJECTORY_SPEED}.0"
            fi
            shift 2
            ;;
        -v|--max-vel)
            MAX_LINEAR_VEL="$2"
            # Ensure float format (add .0 if integer)
            if [[ "$MAX_LINEAR_VEL" =~ ^-?[0-9]+$ ]]; then
                MAX_LINEAR_VEL="${MAX_LINEAR_VEL}.0"
            fi
            shift 2
            ;;
        --enable-traj)
            ENABLE_TRAJ_PUBLISH="$2"
            shift 2
            ;;
        --verbose)
            VERBOSE_LOGGING="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: ./run_sim.sh [OPTIONS]"
            echo "Options:"
            echo "  -x, --x-pos X          Set robot spawn X position (default: 0.0)"
            echo "  -y, --y-pos Y          Set robot spawn Y position (default: 0.0)"
            echo "  -w, --world WORLD      Set world file (default: small_house.world)"
            echo "  -t, --trajectory TYPE  Set trajectory type: 1=Circle, 2=Square, 3=Figure-8 (default: 2)"
            echo "  -r, --radius R         Set trajectory size parameter in meters (default: 2.0)"
            echo "                         Circle: radius=R, Square: side=2*R, Figure-8: each circle radius=R/2"
            echo "  -s, --traj-speed S     Set trajectory reference speed in m/s (default: 0.3)"
            echo "                         Controls how fast the blue reference path moves"
            echo "  -v, --max-vel V        Set maximum robot velocity in m/s (default: 0.4)"
            echo "  --enable-traj BOOL     Enable/disable trajectory publishing: true|false (default: true)"
            echo "  --verbose BOOL         Enable detailed velocity logging (v, omega, vL, vR): true|false (default: true)"
            echo "  -h, --help             Show this help message"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

echo -e "${YELLOW}Configuration:${NC}"
echo "  Spawn Position: ($X_POS, $Y_POS)"
echo "  World: $WORLD"
echo "  Map: $MAP_NAME"
echo "  Trajectory Type: $TRAJECTORY_TYPE"
echo "  Radius Parameter: $RADIUS m"
echo "  Trajectory Speed: $TRAJECTORY_SPEED m/s (Blue reference path speed)"
echo "  Max Robot Velocity: $MAX_LINEAR_VEL m/s"
echo "  Publish Rate: 20 Hz (FIXED for real-time)"
echo "  Plotter Update: 40 Hz (FIXED for smooth visualization)"
echo "  Enable Trajectory Publish: $ENABLE_TRAJ_PUBLISH"
echo "  Verbose Logging: $VERBOSE_LOGGING"
echo ""

# Export all variables so they are available in gnome-terminal subprocesses
export X_POS
export Y_POS
export WORLD
export MAP_NAME
export TRAJECTORY_TYPE
export RADIUS
export TRAJECTORY_SPEED
export MAX_LINEAR_VEL
export ENABLE_TRAJ_PUBLISH
export VERBOSE_LOGGING

# Create temporary directory for launch scripts
TMP_DIR="/tmp/ros2_sim_$$"
mkdir -p "$TMP_DIR"

# Create Gazebo launch script
cat > "$TMP_DIR/launch_gazebo.sh" << EOFGAZEBO
#!/bin/bash
exec 2>&1  # Redirect stderr to stdout
echo "=== Starting Gazebo Simulation ==="
echo "X_POS=$X_POS, Y_POS=$Y_POS, WORLD=$WORLD"
cd ~/ros2_ws || exit 1
source install/setup.bash || { echo "Failed to source workspace"; exec bash; }
echo "Launching: ros2 launch mobile_robot launch_sim.launch.py x_pos:=$X_POS y_pos:=$Y_POS world:=$WORLD"
ros2 launch mobile_robot launch_sim.launch.py x_pos:=$X_POS y_pos:=$Y_POS world:=$WORLD
EXIT_CODE=\$?
echo ""
echo "=== Gazebo terminated with exit code: \$EXIT_CODE ==="
echo "Terminal will stay open. Press Ctrl+D or type 'exit' to close."
exec bash
EOFGAZEBO

# Create localization launch script
cat > "$TMP_DIR/launch_localization.sh" << EOFLOCAL
#!/bin/bash
exec 2>&1  # Redirect stderr to stdout
echo "=== Starting Global Localization ==="
echo "X_POS=$X_POS, Y_POS=$Y_POS, MAP_NAME=$MAP_NAME"
cd ~/ros2_ws || exit 1
source install/setup.bash || { echo "Failed to source workspace"; exec bash; }
echo "Launching: ros2 launch agv_localization global_localization.launch.py x_pos:=$X_POS y_pos:=$Y_POS map_name:=$MAP_NAME"
ros2 launch agv_localization global_localization.launch.py x_pos:=$X_POS y_pos:=$Y_POS map_name:=$MAP_NAME
EXIT_CODE=\$?
echo ""
echo "=== Localization terminated with exit code: \$EXIT_CODE ==="
echo "Terminal will stay open. Press Ctrl+D or type 'exit' to close."
exec bash
EOFLOCAL

# Create trajectory tracking launch script
cat > "$TMP_DIR/launch_trajectory.sh" << EOFTRAJ
#!/bin/bash
exec 2>&1  # Redirect stderr to stdout
echo "=== Starting Trajectory Tracking ==="
echo "TRAJECTORY_TYPE=$TRAJECTORY_TYPE, X_POS=$X_POS, Y_POS=$Y_POS, RADIUS=$RADIUS"
echo "TRAJECTORY_SPEED=$TRAJECTORY_SPEED m/s, MAX_LINEAR_VEL=$MAX_LINEAR_VEL m/s"
echo "ENABLE_TRAJ_PUBLISH=$ENABLE_TRAJ_PUBLISH, VERBOSE_LOGGING=$VERBOSE_LOGGING"
cd ~/ros2_ws || exit 1
source install/setup.bash || { echo "Failed to source workspace"; exec bash; }
echo "Launching: ros2 launch agv_trajectory_tracking trajectory_tracking.launch.py trajectory_type:=$TRAJECTORY_TYPE center_x:=$X_POS center_y:=$Y_POS radius:=$RADIUS trajectory_speed:=$TRAJECTORY_SPEED max_linear_vel:=$MAX_LINEAR_VEL enable_traj_publish:=$ENABLE_TRAJ_PUBLISH verbose_logging:=$VERBOSE_LOGGING"
ros2 launch agv_trajectory_tracking trajectory_tracking.launch.py trajectory_type:=$TRAJECTORY_TYPE center_x:=$X_POS center_y:=$Y_POS radius:=$RADIUS trajectory_speed:=$TRAJECTORY_SPEED max_linear_vel:=$MAX_LINEAR_VEL enable_traj_publish:=$ENABLE_TRAJ_PUBLISH verbose_logging:=$VERBOSE_LOGGING
EXIT_CODE=\$?
echo ""
echo "=== Trajectory tracking terminated with exit code: \$EXIT_CODE ==="
echo "Terminal will stay open. Press Ctrl+D or type 'exit' to close."
exec bash
EOFTRAJ

# Make scripts executable
chmod +x "$TMP_DIR/launch_gazebo.sh"
chmod +x "$TMP_DIR/launch_localization.sh"
chmod +x "$TMP_DIR/launch_trajectory.sh"

# Launch Gazebo simulation
echo -e "${GREEN}[1/3] Launching Gazebo simulation...${NC}"
gnome-terminal --title="Gazebo Simulation" -- "$TMP_DIR/launch_gazebo.sh" &
TERMINAL_PIDS+=($!)
echo -e "${YELLOW}  Terminal PID: ${TERMINAL_PIDS[-1]}${NC}"

# Wait for Gazebo to initialize
echo -e "${YELLOW}Waiting 10 seconds for Gazebo to initialize...${NC}"
sleep 10

# Launch global localization (AMCL + Map Server)
echo -e "${GREEN}[2/3] Launching global localization (AMCL)...${NC}"
gnome-terminal --title="Global Localization" -- "$TMP_DIR/launch_localization.sh" &
TERMINAL_PIDS+=($!)
echo -e "${YELLOW}  Terminal PID: ${TERMINAL_PIDS[-1]}${NC}"

# Wait for localization to initialize
echo -e "${YELLOW}Waiting 5 seconds for localization to initialize...${NC}"
sleep 5

# Launch trajectory tracking
echo -e "${GREEN}[3/3] Launching trajectory tracking system...${NC}"
gnome-terminal --title="Trajectory Tracking" -- "$TMP_DIR/launch_trajectory.sh" &
TERMINAL_PIDS+=($!)

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}All systems launched successfully!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}Terminal Process IDs:${NC}"
for i in "${!TERMINAL_PIDS[@]}"; do
    echo "  Terminal $((i+1)): ${TERMINAL_PIDS[$i]}"
done
echo ""
echo -e "${YELLOW}3 terminal windows have been opened:${NC}"
echo "  1. Gazebo Simulation"
echo "  2. Global Localization (AMCL)"
echo "  3. Trajectory Tracking"
echo ""
echo -e "${YELLOW}To visualize in RViz:${NC}"
echo "  rviz2"
echo ""
echo -e "${RED}To stop all processes, run:${NC}"
echo -e "${RED}  ./kill_sim.sh${NC}"
echo ""
echo -e "${GREEN}✓ Script finished. Simulation is running in background terminals.${NC}"
echo ""
