#!/usr/bin/env bash

SESSION="ired"
DIR="$HOME/ired_ws/src/ired_bringup/ired_bringup"

# Don't create a second copy if the session already exists
if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux attach-session -t "$SESSION"
    exit 0
fi

# Window 0: shell
tmux new-session -d -s "$SESSION" -n "shell" -c "$DIR"

# Window 1: modbus
tmux new-window -t "$SESSION" -n "modbus" -c "$DIR"
tmux send-keys -t "$SESSION:modbus" \
    "ros2 run ired_bringup modbus_serial" C-m

# Window 2: inverse kinematics
tmux new-window -t "$SESSION" -n "inverse_kinematics" -c "$DIR"
tmux send-keys -t "$SESSION:inverse_kinematics" \
    "ros2 run ired_bringup inverse_kinematics" C-m

# Window 3: forward kinematics
tmux new-window -t "$SESSION" -n "forward_kinematics" -c "$DIR"
tmux send-keys -t "$SESSION:forward_kinematics" \
    "ros2 run ired_bringup forward_kinematics" C-m

# Window 4: teleop
tmux new-window -t "$SESSION" -n "teleop" -c "$DIR"
tmux split-window -h -t "$SESSION:teleop"

# Left pane: teleop (type only, do not run)
tmux send-keys -t "$SESSION:teleop.0" \
    "ros2 run ired_teleop teleop_keyboard"

# Right pane: speed topic (type only, do not run)
tmux send-keys -t "$SESSION:teleop.1" \
    "ros2 topic echo /ired/speed"

# Start on window 0
tmux select-window -t "$SESSION:shell"

# Attach
tmux attach-session -t "$SESSION"
