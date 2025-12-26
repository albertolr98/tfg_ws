#!/bin/bash
#
# Remote Launch Script for ow_peque Robot
#
# This script allows launching the robot from your PC without manual SSH.
# It uses SSH + tmux to run the launch file on the robot in the background.
#
# Usage:
#   ./launch_robot.sh              Launch the robot
#   ./launch_robot.sh --stop       Stop the robot
#   ./launch_robot.sh --status     Check if running
#   ./launch_robot.sh --logs       Show robot logs
#
# Configuration: Edit the variables below to match your setup
#

# ====================== CONFIGURATION ======================
ROBOT_HOST="192.168.50.10"      # Robot hostname or IP address
ROBOT_USER="albertolr98"                   # SSH username on the robot
ROS_WS="/home/albertolr98/tfg_ws"          # ROS workspace path on robot
TMUX_SESSION="ow_robot"           # tmux session name
LAUNCH_FILE="ow_peque_description ow_robot.launch.py"
# ===========================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

ssh_cmd() {
    ssh -o ConnectTimeout=5 "${ROBOT_USER}@${ROBOT_HOST}" "$@"
}

check_connection() {
    echo -e "${YELLOW}Checking connection to ${ROBOT_HOST}...${NC}"
    if ! ssh_cmd "echo 'Connected'" &>/dev/null; then
        echo -e "${RED}Error: Cannot connect to ${ROBOT_HOST}${NC}"
        echo "Make sure:"
        echo "  1. The robot is powered on and connected to the network"
        echo "  2. SSH is configured (ssh-copy-id ${ROBOT_USER}@${ROBOT_HOST})"
        echo "  3. The hostname/IP is correct"
        exit 1
    fi
    echo -e "${GREEN}Connected!${NC}"
}

launch_robot() {
    check_connection
    
    echo -e "${YELLOW}Launching robot...${NC}"
    
    # Check if already running
    if ssh_cmd "tmux has-session -t ${TMUX_SESSION} 2>/dev/null"; then
        echo -e "${YELLOW}Robot is already running in tmux session '${TMUX_SESSION}'${NC}"
        echo "Use --stop to stop it first, or --logs to view logs"
        exit 0
    fi
    
    # Create tmux session and launch
    ssh_cmd "tmux new-session -d -s ${TMUX_SESSION} \
        'source /opt/ros/\$(ls /opt/ros/ | head -1)/setup.bash && \
         source ${ROS_WS}/install/setup.bash && \
         ros2 launch ${LAUNCH_FILE}; \
         echo \"Press any key to close...\"; read'"
    
    echo -e "${GREEN}Robot launched successfully!${NC}"
    echo "Use './launch_robot.sh --logs' to view logs"
    echo "Use './launch_robot.sh --stop' to stop"
}

stop_robot() {
    check_connection
    
    echo -e "${YELLOW}Stopping robot...${NC}"
    
    if ! ssh_cmd "tmux has-session -t ${TMUX_SESSION} 2>/dev/null"; then
        echo -e "${YELLOW}Robot is not running${NC}"
        exit 0
    fi
    
    # Send Ctrl+C to gracefully stop, then kill session
    ssh_cmd "tmux send-keys -t ${TMUX_SESSION} C-c"
    sleep 2
    ssh_cmd "tmux kill-session -t ${TMUX_SESSION} 2>/dev/null || true"
    
    echo -e "${GREEN}Robot stopped${NC}"
}

show_status() {
    check_connection
    
    if ssh_cmd "tmux has-session -t ${TMUX_SESSION} 2>/dev/null"; then
        echo -e "${GREEN}Robot is RUNNING${NC} (tmux session: ${TMUX_SESSION})"
        echo ""
        echo "Active ROS nodes on robot:"
        ssh_cmd "source /opt/ros/\$(ls /opt/ros/ | head -1)/setup.bash && ros2 node list 2>/dev/null" || echo "(Could not list nodes)"
    else
        echo -e "${YELLOW}Robot is NOT running${NC}"
    fi
}

show_logs() {
    check_connection
    
    if ! ssh_cmd "tmux has-session -t ${TMUX_SESSION} 2>/dev/null"; then
        echo -e "${YELLOW}Robot is not running${NC}"
        exit 1
    fi
    
    echo -e "${YELLOW}Attaching to robot logs (Ctrl+B, D to detach)...${NC}"
    ssh -t "${ROBOT_USER}@${ROBOT_HOST}" "tmux attach-session -t ${TMUX_SESSION}"
}

# Main
case "${1:-}" in
    --stop)
        stop_robot
        ;;
    --status)
        show_status
        ;;
    --logs)
        show_logs
        ;;
    --help|-h)
        echo "Usage: $0 [--stop|--status|--logs|--help]"
        echo ""
        echo "Commands:"
        echo "  (none)    Launch the robot"
        echo "  --stop    Stop the robot"
        echo "  --status  Check if running"
        echo "  --logs    Attach to robot logs"
        echo "  --help    Show this help"
        ;;
    *)
        launch_robot
        ;;
esac
