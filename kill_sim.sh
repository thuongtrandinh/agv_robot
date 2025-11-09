#!/bin/bash

# Script to kill all simulation-related processes
# Author: Auto-generated for ROS2 AGV project
# Date: November 3, 2025

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${RED}========================================${NC}"
echo -e "${RED}Stopping All Simulation Processes${NC}"
echo -e "${RED}========================================${NC}"
echo ""

# Function to force kill process by name pattern
kill_process() {
    local pattern=$1
    local description=$2
    
    if pgrep -f "$pattern" > /dev/null; then
        echo -e "${YELLOW}Killing $description...${NC}"
        pkill -9 -f "$pattern"
        sleep 0.3
        
        # Double check and force kill if still running
        if pgrep -f "$pattern" > /dev/null; then
            echo -e "${YELLOW}  Force killing $description (2nd attempt)...${NC}"
            local pids=$(pgrep -f "$pattern")
            for pid in $pids; do
                kill -9 $pid 2>/dev/null
                # Kill all children
                pkill -9 -P $pid 2>/dev/null
            done
        fi
    fi
}

# Step 1: Kill gnome-terminal processes running our scripts
echo -e "${YELLOW}[1/8] Killing terminal windows...${NC}"
pkill -9 -f "gnome-terminal.*Gazebo Simulation" 2>/dev/null
pkill -9 -f "gnome-terminal.*Global Localization" 2>/dev/null
pkill -9 -f "gnome-terminal.*Trajectory Tracking" 2>/dev/null

# Step 2: Kill Gazebo (multiple variants)
echo -e "${YELLOW}[2/8] Killing Gazebo processes...${NC}"
killall -9 gzserver 2>/dev/null
killall -9 gzclient 2>/dev/null
killall -9 gazebo 2>/dev/null
killall -9 gz 2>/dev/null
killall -9 ruby 2>/dev/null  # Gazebo sometimes uses ruby
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "gz_sim" 2>/dev/null

# Step 3: Kill ROS2-Gazebo bridges
echo -e "${YELLOW}[3/8] Killing ROS-Gazebo bridges...${NC}"
kill_process "ros_gz_bridge" "ROS-Gazebo Bridge"
kill_process "parameter_bridge" "Parameter Bridge"
kill_process "ros_gz" "ROS-Gazebo"

# Step 4: Kill ROS2 launch processes
echo -e "${YELLOW}[4/8] Killing ROS2 launch files...${NC}"
kill_process "launch_sim.launch.py" "Launch Sim"
kill_process "global_localization.launch.py" "Global Localization"
kill_process "trajectory_tracking.launch.py" "Trajectory Tracking"

# Step 5: Kill robot state and controllers
echo -e "${YELLOW}[5/8] Killing robot controllers...${NC}"
kill_process "robot_state_publisher" "Robot State Publisher"
kill_process "controller_manager" "Controller Manager"
kill_process "joint_broad" "Joint State Broadcaster"
kill_process "diff_cont" "Differential Drive Controller"
kill_process "joint_state" "Joint State"

# Step 6: Kill trajectory tracking nodes
echo -e "${YELLOW}[6/8] Killing trajectory tracking nodes...${NC}"
kill_process "trajectory_publisher" "Trajectory Publisher"
kill_process "fuzzy_trajectory_controller" "Fuzzy Controller"
kill_process "trajectory_plotter" "Trajectory Plotter"

# Step 7: Kill localization nodes
echo -e "${YELLOW}[7/8] Killing localization nodes...${NC}"
kill_process "map_server" "Map Server"
kill_process "amcl" "AMCL"
kill_process "lifecycle_manager" "Lifecycle Manager"
kill_process "scan_repub" "Scan Republisher"

# Step 8: Kill visualization tools
echo -e "${YELLOW}[8/8] Killing visualization tools...${NC}"
killall -9 rviz2 2>/dev/null
pkill -9 -f "rviz2" 2>/dev/null
pkill -9 -f "matplotlib" 2>/dev/null
pkill -9 -f "python3.*plotter" 2>/dev/null

# Nuclear option: Kill all processes from our workspace
echo -e "${YELLOW}Performing deep cleanup...${NC}"
pkill -9 -f "ros2_ws/install" 2>/dev/null
pkill -9 -f "/home/thuong/ros2_ws" 2>/dev/null

# Clean up shared memory
echo -e "${YELLOW}Cleaning up shared memory...${NC}"
rm -rf /dev/shm/gz-* 2>/dev/null
rm -rf /dev/shm/fastrtps* 2>/dev/null
rm -rf /dev/shm/sem.gz* 2>/dev/null

# Optional: Restart ROS2 daemon for clean state
echo -e "${YELLOW}Restarting ROS2 daemon...${NC}"
ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start 2>/dev/null

# Wait for processes to die
sleep 2

# Final verification
echo ""
echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}Final Status Check${NC}"
echo -e "${YELLOW}========================================${NC}"

GAZEBO_RUNNING=$(pgrep -f "gzserver|gzclient|gazebo|gz sim" 2>/dev/null | wc -l)
ROS2_LAUNCH=$(pgrep -f "ros2 launch.*launch_sim|global_localization|trajectory_tracking" 2>/dev/null | wc -l)
ROS2_NODES=$(pgrep -f "robot_state_publisher|fuzzy_trajectory|amcl|map_server" 2>/dev/null | wc -l)
RVIZ_RUNNING=$(pgrep -f "rviz2" 2>/dev/null | wc -l)

echo ""
if [ $GAZEBO_RUNNING -eq 0 ]; then
    echo -e "${GREEN}✓ Gazebo stopped${NC}"
else
    echo -e "${RED}✗ Gazebo still running ($GAZEBO_RUNNING processes)${NC}"
    pgrep -fa "gzserver|gzclient|gazebo" | head -3
fi

if [ $ROS2_LAUNCH -eq 0 ]; then
    echo -e "${GREEN}✓ Launch files stopped${NC}"
else
    echo -e "${RED}✗ Launch files still running ($ROS2_LAUNCH processes)${NC}"
fi

if [ $ROS2_NODES -eq 0 ]; then
    echo -e "${GREEN}✓ ROS2 nodes stopped${NC}"
else
    echo -e "${YELLOW}⚠ Some ROS2 nodes still running ($ROS2_NODES processes)${NC}"
fi

if [ $RVIZ_RUNNING -eq 0 ]; then
    echo -e "${GREEN}✓ RViz2 stopped${NC}"
else
    echo -e "${RED}✗ RViz2 still running${NC}"
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Cleanup Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# If critical processes still running, give additional instructions
TOTAL_REMAINING=$((GAZEBO_RUNNING + ROS2_LAUNCH))
if [ $TOTAL_REMAINING -gt 0 ]; then
    echo -e "${YELLOW}⚠ WARNING: $TOTAL_REMAINING critical processes still running${NC}"
    echo ""
    echo -e "${YELLOW}Additional cleanup commands:${NC}"
    echo -e "  ${RED}killall -9 gzserver gzclient python3 ruby${NC}"
    echo -e "  ${RED}pkill -9 -f ros2${NC}"
    echo -e "  ${RED}pkill -9 -f gz${NC}"
    echo ""
    echo -e "${YELLOW}If nothing works, you may need to:${NC}"
    echo -e "  ${RED}1. Close Gazebo window manually (Alt+F4)${NC}"
    echo -e "  ${RED}2. Log out and log back in${NC}"
    echo -e "  ${RED}3. Reboot system${NC}"
    echo ""
fi
