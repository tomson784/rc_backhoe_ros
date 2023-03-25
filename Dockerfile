ARG ROS_DISTRO="noetic"

FROM ros:$ROS_DISTRO

SHELL ["/bin/bash", "-c"]

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update -y && apt-get install git -y && \
    # # install ros-packages
    # apt-get install -y ros-$ROS_DISTRO-tf2 ros-$ROS_DISTRO-tf2-geometry-msgs && \
    # apt-get install -y ros-$ROS_DISTRO-geometry2 && \
    apt-get install -y ros-$ROS_DISTRO-imu-tools ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-joint-state-publisher && \
    apt-get install -y ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-hector-gazebo-plugins && \
    apt-get install -y ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-gazebo-ros-control && \
    apt-get install -y ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers && \
    # apt-get -y install ros-$ROS_DISTRO-rosbridge-server && \
    # apt-get -y install ros-$ROS_DISTRO-roswww && \
    # apt-get install -y libarmadillo-dev qtbase5-dev && \
    apt-get install -y ros-$ROS_DISTRO-rviz && \
    # apt-get install -y ros-$ROS_DISTRO-costmap-2d && \
    # apt-get install -y ros-$ROS_DISTRO-grid-map && \
    # # 
    mkdir -p /catkin_ws/src && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd /catkin_ws && catkin_make && \
    rm -rf /var/lib/apt/lists/*

RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc && \
    echo "alias cm='cd /catkin_ws/ && catkin_make'" >> ~/.bashrc && \
    echo "alias cm_test='cd /catkin_ws/ && catkin_make run_tests'"  >> ~/.bashrc && \
    echo "alias rosenv='env | grep ROS'" >> ~/.bashrc && \
    git config --global credential.helper cachcde && \
    git config --global credential.helper 'cache --timeout=600' && \
    cd /catkin_ws

WORKDIR /catkin_ws/src