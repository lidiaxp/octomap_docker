xhost +local:docker
docker run --privileged -it --rm \
           -e ROS_MASTER_URI=http://$(ip route | awk '/^default via/ {print $5}' | xargs -I {} ip addr show {} | awk '/inet / {split($2, a, "/"); print a[1]}'):11311 \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -e NVIDIA_VISIBLE_DEVICES=all \
           -e ROS_DOMAIN_ID=17 \
           --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
           -p 11311:11311 \
           --ipc=host \
           --name my_ros2_container \
           -v /home/lidia/lrs2_ws/src/path_speed_flying/utils/subs_pc_mqtt_ros2.py:/root/ros2_ws/src/subs_pc_mqtt_ros2.py \
           -v /home/lidia/lrs2_ws/src/path_speed_flying/utils/tf_broadcaster_unity.py:/root/ros2_ws/src/tf_broadcaster_unity.py \
           --shm-size=4gb \
           --env="DISPLAY=$DISPLAY" \
           octomap-ros2:latest /bin/bash

#    --net=host \
