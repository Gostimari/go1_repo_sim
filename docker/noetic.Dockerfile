ARG ARCH=
#2
ARG CORES=6
FROM ${ARCH}ros:noetic-ros-base

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install packages
RUN apt-get update \
    && apt-get install -y \
    # Basic utilities
    build-essential \
    apt-utils \
    curl \
    git \
    wget \
    vim \
    nano \
    libtbb-dev \
    libgoogle-glog-dev


# Install some python packages
RUN apt-get -y install \
    python3 \
    python3-pip \
    python3-serial \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-rosdep \
    python3-catkin-tools

RUN pip3 install "numpy>=1.20" pyquaternion

#Install ROS Packages
RUN apt-get install -y ros-${ROS_DISTRO}-pcl-ros \ 
    ros-${ROS_DISTRO}-geometry \
    ros-${ROS_DISTRO}-grid-map-core \
    ros-${ROS_DISTRO}-grid-map-ros \
    ros-${ROS_DISTRO}-grid-map-filters \
    ros-${ROS_DISTRO}-grid-map-rviz-plugin \
    ros-${ROS_DISTRO}-grid-map-visualization \
    ros-${ROS_DISTRO}-imu-filter-madgwick \
    ros-${ROS_DISTRO}-imu-transformer \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-navigation \
    ros-${ROS_DISTRO}-mapviz \
    ros-${ROS_DISTRO}-mapviz-plugins \
    ros-${ROS_DISTRO}-tile-map \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-dynamic-reconfigure \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-visualization-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-octomap-ros \
    ros-${ROS_DISTRO}-octomap-msgs \
    ros-${ROS_DISTRO}-nodelet \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-gazebo-ros \
    yad

# Clean-up
WORKDIR /
RUN apt-get clean

#Configure catkin workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
#WORKDIR $CATKIN_WS

# Clean-up
WORKDIR /
RUN apt-get clean

#RUN echo "source /usr/local/bin/catkin_entrypoint.sh" >> /root/.bashrc
COPY noetic-launch.sh /noetic-launch.sh
RUN chmod +x /noetic-launch.sh

COPY app_launcher_noetic.sh /app_launcher_noetic.sh
RUN chmod +x /app_launcher_noetic.sh

#ENTRYPOINT ["/noetic-launch.sh"]
CMD ["bash"]
