FROM osrf/ros:humble-desktop-full 

# Install PCL & Eigen & essential libraries
RUN apt-get update && apt-get install -y cmake libatlas-base-dev libeigen3-dev libpcl-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev wget unzip git python3-pip
RUN apt-get install -y ros-humble-tf2 ros-humble-cv-bridge ros-humble-pcl-conversions ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-rviz2 ros-humble-image-transport ros-humble-image-transport-plugins ros-humble-pcl-ros


RUN /bin/bash -c '. /opt/ros/humble/setup.bash;'

WORKDIR /root/ros2_ws

COPY src /root/ros2_ws/src

# RUN source /opt/ros/humble/setup.sh
# RUN source /opt/ros/humble/setup.bash
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# TODO add to entry point and to bashrc
RUN /bin/bash -c "source /root/ros2_ws/install/setup.bash"

# RUN colcon build --packages-select octomap_msgs octomap_ros
# RUN source install/setup.bash
# RUN colcon build 
# RUN source install/setup.bash

RUN sudo apt install -y tmux nano vim
RUN pip3 install paho-mqtt

RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

COPY out.sh /root/out.sh
RUN echo "alias out='bash /root/out.sh'" >> /root/.bashrc

COPY run_octomap.sh /root/run_octomap.sh

# Load ROS environment at each run
COPY ./ros_entrypoint.sh /
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]


# CMD ["bash"]

# --volume="/home/lidia/py_ws/OctoMap-ROS2/src:/root/ros2_ws/src" \
