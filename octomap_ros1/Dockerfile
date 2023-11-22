FROM ros:melodic

RUN apt-get update && apt-get install -y \
    tmux \
    git \
    python-wstool \
    python-catkin-tools \
    ros-melodic-pcl-ros \
    ros-melodic-octomap-mapping \
    ros-melodic-octomap-msgs \
    ros-melodic-octomap-ros \
    ros-melodic-rviz \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri 

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash \
    && mkdir -p /catkin_ws/src \
    && cd /catkin_ws \
    && catkin init  \
    && cd /catkin_ws/src \
    && git clone https://github.com/OctoMap/octomap_mapping.git"

COPY launch_octomap.launch /catkin_ws/src/octomap_mapping/octomap_server/launch

RUN apt install -y \
    ros-melodic-turtlesim \
    xvfb

RUN /bin/bash -c "cd /catkin_ws/src \
    && catkin_create_pkg learning_tf tf roscpp rospy turtlesim"

RUN /bin/bash -c "cd /catkin_ws \
    && source /opt/ros/melodic/setup.bash \
    && catkin build"

COPY run_octomap.sh /root/run_octomap.sh

COPY tf_broadcaster.py /catkin_ws/src/octomap_mapping/octomap_server/src/tf_broadcaster.py
COPY tf_broadcaster_unity.py /catkin_ws/src/octomap_mapping/octomap_server/src/tf_broadcaster_unity.py
COPY tf_launch.launch /catkin_ws/src/octomap_mapping/octomap_server/launch/tf_launch.launch
COPY subs_pc_mqtt.py /catkin_ws/src/octomap_mapping/octomap_server/src/subs_pc_mqtt.py
COPY check_collision.py /catkin_ws/src/octomap_mapping/octomap_server/src/check_collision.py

COPY out.sh /root/out.sh
RUN echo "alias out='bash /root/out.sh'" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/melodic/setup.bash && source /catkin_ws/devel/setup.bash && bash /root/run_octomap.sh && /bin/bash"]
