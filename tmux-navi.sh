#!/usr/bin/env bash

SESSION="ired"

# Don't create a second copy if the session already exists
if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux attach-session -t "$SESSION"
    exit 0
fi

# Window 0: workspace shell
tmux new-session -d -s "$SESSION" -n "shell"
tmux send-keys -t "$SESSION:shell" \
    "cd ~/ired_ws" C-m

# Window 1: bringup
tmux new-window -t "$SESSION" -n "bringup"
tmux send-keys -t "$SESSION:bringup" \
    "cd ~/ired_ws" C-m
tmux send-keys -t "$SESSION:bringup" \
    "ros2 launch ired_bringup bringup.launch.py" C-m

# Window 2: navigation
tmux new-window -t "$SESSION" -n "navigation"
tmux send-keys -t "$SESSION:navigation" \
    "cd ~/ired_ws" C-m
tmux send-keys -t "$SESSION:navigation" \
    "ros2 launch ired_navigation navigation.launch.xml" C-m

# Start on window 0
tmux select-window -t "$SESSION:shell"

# Attach
tmux attach-session -t "$SESSION"
