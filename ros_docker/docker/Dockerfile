# Handle ros distro
ARG ROS_DISTRO=noetic
ARG ROS_IMAGE=${ROS_DISTRO}-desktop-full

FROM osrf/ros:${ROS_IMAGE} as ros-ws

ARG ROS_DISTRO
ARG USER=root
ARG HOME=/home/${USER}

ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES video,compute,utility

# create and configure a new user
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} ${USER}
RUN adduser --uid ${UID} --gid ${GID} ${USER} && yes | passwd ${USER}
RUN usermod -a -G dialout ${USER}

WORKDIR ${HOME}
RUN chown -R ${USER}:${USER} ${HOME}

# Install ubuntu libraries
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    git \
    python3 \
    ros-noetic-lms1xx \
    ros-noetic-interactive-marker-twist-server \
    ros-noetic-robot-localization \
    ros-noetic-twist-mux \
    ros-noetic-joy \
    ros-noetic-teleop-twist-joy

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

RUN chown -R ${USER}:${USER} ${HOME}

WORKDIR ${HOME}/catkin_ws/src
RUN chown -R ${USER}:${USER} ${HOME}/catkin_ws
RUN chown -R ${USER}:${USER} ${HOME}/catkin_ws/src

USER ${USER}

WORKDIR ${HOME}/catkin_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make"
RUN echo "source ${HOME}/catkin_ws/devel/setup.bash" >> ~/.bashrc

CMD [ "bash" ]