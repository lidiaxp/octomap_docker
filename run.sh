xhost +
docker run -it --rm \
    -e ROS_MASTER_URI=http://$(ip route | awk '/^default via/ {print $5}' | xargs -I {} ip addr show {} | awk '/inet / {split($2, a, "/"); print a[1]}'):11311 \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e XAUTHORITY=$XAUTH \
    -v $XAUTH:$XAUTH \
    -v ${PWD}/launch_octomap.launch:/catkin_ws/src/octomap_mapping/octomap_server/launch/launch_octomap.launch \
    -v ${PWD}/tf_broadcaster.py:/catkin_ws/src/octomap_mapping/octomap_server/src/tf_broadcaster.py \
    --net=host \
    --privileged \
    octomap \
    /bin/bash
