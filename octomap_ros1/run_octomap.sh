#!/bin/bash

tmux new-session -d -s my_session -n "roscore"
tmux new-window -t my_session:1 -n "tf"
tmux new-window -t my_session:2 -n "octomap"
# tmux new-window -t my_session:3 -n "tests"

tmux send-keys -t my_session:0 "roscore" Enter
sleep 1
tmux send-keys -t my_session:1 "python /catkin_ws/src/octomap_mapping/octomap_server/src/tf_broadcaster_unity.py" Enter
sleep 1
tmux send-keys -t my_session:2 "roslaunch octomap_server launch_octomap.launch" Enter

sleep 3

# tmux send-keys -t my_session:1 "source /opt/ros/melodic/setup.bash && source /catkin_ws/devel/setup.bash && chmod +x /catkin_ws/src/octomap_mapping/octomap_server/src/tf_broadcaster.py && chmod +x /catkin_ws/src/octomap_mapping/octomap_server/launch/tf_launch.launch && roslaunch octomap_server tf_launch.launch" Enter

# Attach to the tmux session
tmux attach-session -t my_session
