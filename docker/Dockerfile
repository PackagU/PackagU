# PackagU 개발용 이미지 (amd64 데스크톱: Gazebo/RViz 포함)
# 빌드:  docker build -f docker/Dockerfile -t packagu/ros2-humble-slam:humble docker/
FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

LABEL org.opencontainers.image.source="https://github.com/PackagU/PackagU"
LABEL org.opencontainers.image.description="PackagU ROS2 Humble desktop image for SLAM development"
LABEL org.opencontainers.image.licenses="MIT"

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    git \
    iputils-ping \
    less \
    mesa-utils \
    nano \
    net-tools \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-serial \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    ros-humble-robot-state-publisher \
    ros-humble-rplidar-ros \
    ros-humble-rviz2 \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    ros-humble-tf2-tools \
    ros-humble-xacro \
    vim \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true && rosdep update

RUN mkdir -p /ros2_ws/src /ros2_ws/maps
WORKDIR /ros2_ws

ENV ROS_DOMAIN_ID=0
ENV QT_X11_NO_MITSHM=1

COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc \
    && echo 'if [ -f "/ros2_ws/install/setup.bash" ]; then source /ros2_ws/install/setup.bash; fi' >> /etc/bash.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
