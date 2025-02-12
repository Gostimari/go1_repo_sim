ARG ARCH=
#2
ARG CORES=6
FROM ${ARCH}ros:melodic-ros-core

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

SHELL ["/bin/bash","-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Install packages
RUN apt-get update \
    && apt-get install -y \
    build-essential \
    apt-utils \
    cmake \
    curl \
    cmake \
    vim \
    git \
    wget \
    nano \
    libboost-all-dev

# Install some python packages
RUN apt-get -y install \
    python \
    python-catkin-pkg \
    python-pip \
    python-serial \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    python-rosdep \
    python-catkin-tools

#Install ROS Packages
RUN apt-get install -y ros-${ROS_DISTRO}-navigation \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-tf-conversions \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-interactive-markers \
    ros-${ROS_DISTRO}-nav-core \
    ros-${ROS_DISTRO}-laser-geometry \
    yad

# Clean-up
WORKDIR /
RUN apt-get clean

#Configure catkin workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS

# Clean-up
WORKDIR /
RUN apt-get clean

#RUN echo "source /usr/local/bin/catkin_entrypoint.sh" >> /root/.bashrc
COPY melodic-launch.sh /melodic-launch.sh
RUN chmod +x /melodic-launch.sh

COPY app_launcher_melodic.sh /app_launcher_melodic.sh
RUN chmod +x /app_launcher_melodic.sh

#ENTRYPOINT ["/melodic-launch.sh"]
CMD ["bash"]
