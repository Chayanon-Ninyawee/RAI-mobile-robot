FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV BASH_ENV=/etc/bash.bashrc

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    iproute2 \
    iputils-ping \
    net-tools \
    tmux \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-rviz2 \
    ros-jazzy-laser-filters \
    ros-jazzy-tf-transformations \
    ros-jazzy-xacro \
    ros-jazzy-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/ired_ws

COPY src/ /opt/ired_ws/src/

RUN apt-get update \
    && rosdep update \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro jazzy \
        -r -y \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c \
    "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

RUN printf '%s\n' \
    'source /opt/ros/jazzy/setup.bash' \
    'if [ -f /opt/ired_ws/install/setup.bash ]; then source /opt/ired_ws/install/setup.bash; fi' \
    "alias rviz-navi='rviz2 -d /opt/ired_ws/src/ired_rviz/navi.rviz'" \
    "alias rviz-slam='rviz2 -d /opt/ired_ws/src/ired_rviz/slam.rviz'" \
    >> /etc/bash.bashrc

WORKDIR /opt/ired_ws

CMD ["bash"]
