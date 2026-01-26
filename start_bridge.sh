#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SETUP_FILE="$SCRIPT_DIR/install/setup.bash"
SELF_PATH="$SCRIPT_DIR/start_bridge.sh"

# Check if the install/setup.bash exists
if [ ! -f "$SETUP_FILE" ]; then
    echo "Warning: install/setup.bash not found. Make sure you have built the workspace."
    exit 1
fi

source "$SETUP_FILE"

MODE=$1

if [ "$MODE" == "agent" ]; then
    echo "--------------------------------"
    echo "Starting MicroXRCEAgent..."
    echo "--------------------------------"
    MicroXRCEAgent udp4 -p 8888

elif [ "$MODE" == "vrpn" ]; then
    # Wait a bit to ensure agent might be ready (optional, but good practice)
    sleep 2
    echo "--------------------------------"
    echo "Starting VRPN Client..."
    echo "--------------------------------"
    ros2 launch vrpn_mocap client.launch.yaml server:=192.168.31.101 port:=3883

else
    # Default mode: Check for tmux
    if command -v tmux &> /dev/null; then
        echo "Tmux detected. Starting split-screen session..."
        SESSION="px4_bridge_$(date +%s)"
        
        # Create new session but don't attach yet
        tmux new-session -d -s "$SESSION"
        
        # Pane 1 (Left): Run Agent
        # We invoke this script again with 'agent' argument
        tmux send-keys -t "$SESSION" "$SELF_PATH agent" C-m
        
        # Split window horizontally
        tmux split-window -h -t "$SESSION"
        
        # Pane 2 (Right): Run VRPN
        tmux send-keys -t "$SESSION" "$SELF_PATH vrpn" C-m
        
        # Attach to session
        tmux attach -t "$SESSION"
    else
        echo "========================================================"
        echo " SSH / Manual Mode Detected"
        echo "========================================================"
        echo "Since you are in a headless environment (SSH/Trae) and"
        echo "'tmux' is not installed, I cannot open new windows for you."
        echo ""
        echo "Please open TWO terminal tabs in Trae and run:"
        echo ""
        echo "  Terminal 1: $0 agent"
        echo "  Terminal 2: $0 vrpn"
        echo "========================================================"
    fi
fi