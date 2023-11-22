#!/bin/bash

tmux new-session -d -s my_session -n "tf"
tmux new-window -t my_session:1 -n "pc_unity"
tmux new-window -t my_session:2 -n "octomap"

tmux send-keys -t my_session:0 "python3 ~/ros2_ws/src/tf_broadcaster_unity.py" Enter
sleep 1
tmux send-keys -t my_session:1 "python3 ~/ros2_ws/src/subs_pc_mqtt_ros2.py" Enter
sleep 1
tmux send-keys -t my_session:2 "ros2 launch octomap_server octomap_mapping.launch.xml" Enter

sleep 3

tmux attach-session -t my_session
