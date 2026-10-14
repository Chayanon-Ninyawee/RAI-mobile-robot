FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    build-essential \
    cmake \
    git \
    tmux \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    python3-crcmod \
    iproute2 \
    iputils-ping \
    nano \
    vim \
    ros-dev-tools \
    ros-jazzy-ament-cmake \
    ros-jazzy-tf-transformations \
    ros-jazzy-xacro \
    ros-jazzy-laser-filters \
    ros-jazzy-cartographer \
    ros-jazzy-cartographer-ros \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN printf '\nsource /opt/ros/jazzy/setup.bash\n' >>/etc/bash.bashrc

WORKDIR /workspace

CMD ["bash"]
