#-------------------------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License. See https://go.microsoft.com/fwlink/?linkid=2090316 for license information.
#-------------------------------------------------------------------------------------------------------------
FROM osrf/ros:noetic-desktop-full


# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive
# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# This Dockerfile adds a non-root user with sudo access. Use the "remoteUser"
# property in devcontainer.json to use it. On Linux, the container user's GID/UIDs
# will be updated to match your local UID/GID (when using the dockerFile property).
# See https://aka.ms/vscode-remote/containers/non-root-user for details.
ARG USERNAME=robot
ARG USER_UID=1000
ARG USER_GID=$USER_UID

## Install python stuff
RUN apt-get update \
  && apt-get install -y \
    psmisc \
    python3.6 \
    python3-pip \
    python-is-python3 \
    mpich \
    python3-tk \
    python3-catkin-pkg \
    python3-rospy \
  && rm -rf /var/lib/apt/lists/*

## conda docker
ARG CONDA_INSTALL_PATH=/opt/conda
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV PATH ${CONDA_INSTALL_PATH}/bin:$PATH

RUN apt-get update --fix-missing && \
    apt-get install -y wget bzip2 ca-certificates curl git && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-4.5.11-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p ${CONDA_INSTALL_PATH} && \
    rm ~/miniconda.sh && \
    ${CONDA_INSTALL_PATH}/bin/conda clean -tipsy && \
    ln -s ${CONDA_INSTALL_PATH}/etc/profile.d/conda.sh /etc/profile.d/conda.sh

RUN ${CONDA_INSTALL_PATH}/bin/conda create -n robot-rl python=3.6

COPY . /home/${USERNAME}/catkin_ws
WORKDIR /home/${USERNAME}/catkin_ws
RUN pip3 install -r requirements.txt --upgrade

# Add keys for ros
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt install curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Configure apt and install packages
RUN apt-get update \
    && apt-get -y install --no-install-recommends apt-utils dialog 2>&1 \
    #
    # Verify git, process tools, lsb-release (common in install instructions for CLIs) installed
    && apt-get -y install git iproute2 procps lsb-release nano libopenmpi-dev swig wget
    #

## Rosdep install
RUN rm /etc/ros/rosdep/sources.list.d/20-default.list
RUN rosdep init \
    && rosdep update
WORKDIR /home/${USERNAME}/catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; \
    rosdep install --from-paths /home/${USERNAME}/catkin_ws/src --ignore-src -y -i -r;'

RUN apt-get -y install

## Install spinningup
WORKDIR /home/${USERNAME}/catkin_ws/src/spinningup
RUN /bin/bash -c "source activate robot-rl && pip install --upgrade pip && pip install -e ."
WORKDIR /home/${USERNAME}/catkin_ws

## User settings
# Create a non-root user to use if preferred - see https://aka.ms/vscode-remote/containers/non-root-user.
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME:$USERNAME" | chpasswd \
    && usermod --shell /bin/bash $USERNAME \
    && usermod -aG sudo $USERNAME \
    # [Optional] Add sudo support for the non-root user
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME
    #
# Create alternate global install location that both uses have rights to access
RUN mkdir -p /usr/local/share/pip-global \
    && chown ${USERNAME}:root /usr/local/share/pip-global
    #
# Clean up
RUN apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=dialog

COPY .bashrc_docker /root/.bashrc
RUN echo "export ROBOT_RL_SIM_ROOT=/home/${USERNAME}/catkin_ws/" >> /root/.bashrc
RUN echo "source $ROBOT_RL_SIM_ROOT/devel/setup.bash" >> /root/.bashrc
COPY .bashrc_docker /home/$USERNAME/.bashrc
RUN echo "export ROBOT_RL_SIM_ROOT=/home/${USERNAME}/catkin_ws/" >> /home/$USERNAME/.bashrc
RUN echo "source $ROBOT_RL_SIM_ROOT/devel/setup.bash" >>  /home/$USERNAME/.bashrc

RUN echo "Image build"
# In case you do not run interactive container.
#CMD source /root/.bashrc
