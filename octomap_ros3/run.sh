xhost +local:docker
docker run --privileged -it --rm \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -e NVIDIA_VISIBLE_DEVICES=all \
           --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
           --net=host \
           --ipc=host \
            -v /home/lidia/lrs2_ws/src/path_speed_flying/utils/subs_pc_mqtt_ros2.py:/root/ros2_ws/src/subs_pc_mqtt_ros2.py \
            -v /home/lidia/lrs2_ws/src/path_speed_flying/utils/tf_broadcaster_unity.py:/root/ros2_ws/src/tf_broadcaster_unity.py \
           --shm-size=4gb \
           --env="DISPLAY=$DISPLAY" \
           octomap-ros2:latest /bin/bash
           