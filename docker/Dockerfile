# Handle ros distro
ARG ROS_DISTRO=noetic

FROM ros:${ROS_DISTRO} as ros-ws

ARG ROS_DISTRO
ARG USER=root
ARG HOME=/home/${USER}

ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES video,compute,utility
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"

# Tell docker we want to use bash instead of sh in general
SHELL ["/bin/bash", "-c"]

# create and configure a new user
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y sudo
RUN useradd -m ${USER} && echo "${USER}:${USER}passwd" | chpasswd && adduser ${USER} sudo

### Add a few essential tools
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y git openssh-client python3 net-tools build-essential cmake git
RUN apt install -y \
    bash-completion \
    silversearcher-ag \
    apt-transport-https \
    less \
    alsa-utils \
    ros-${ROS_DISTRO}-vrpn-client-ros \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-rosparam-shortcuts \
    libcgal-dev

# Install catkin tools
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y wget
RUN sh -c \
    'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
    > /etc/apt/sources.list.d/ros-latest.list'

RUN wget http://packages.ros.org/ros.key -O - | apt-key add -
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y python3-catkin-tools

# Add GitHub's host key
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y python3-pip
RUN pip install matplotlib
RUN ln -s /usr/bin/python3 /usr/bin/python
RUN mkdir -p /root/.ssh
RUN --mount=type=secret,id=ssh_key ln -s /run/secrets/ssh_key /root/.ssh/id_ed25519
RUN ssh-keyscan -H github.com >> /root/.ssh/known_hosts

FROM ros-ws as libraries

# Setup the project using automatic script
# Use a secret for the SSH key
COPY ./scripts/setupIiwaRosEnv.sh /run/setupIiwaRosEnv.sh
RUN chmod +x /run/setupIiwaRosEnv.sh
RUN --mount=type=secret,id=ssh_key /run/setupIiwaRosEnv.sh

COPY ./scripts/setupControlLasaEnv.sh /run/setupControlLasaEnv.sh
RUN chmod +x /run/setupControlLasaEnv.sh
RUN --mount=type=secret,id=ssh_key /run/setupControlLasaEnv.sh

# Remove soft link to SSH key
RUN unlink /root/.ssh/id_ed25519

# Install moveit tools
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    python3-vcstool \
    ros-noetic-rqt-joint-trajectory-controller \
    ros-noetic-moveit-commander \
    ros-noetic-teleop-twist-keyboard

# Install trac ik
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    ros-${ROS_DISTRO}-trac-ik \
    ros-${ROS_DISTRO}-trac-ik-kinematics-plugin \
    ros-${ROS_DISTRO}-diagnostic-updater

FROM libraries as cmake-options

# Handle SIMD option
RUN if [ "${USE_SIMD}" = "ON" ] ; \
    then export CMAKE_CXX_FLAGS="-march=native -faligned-new" ; fi

# Add cmake option to bash rc if needed
RUN if [ "${USE_SIMD}" = "ON" ] ; \
    then echo "export ENABLE_SIMD=ON" >> /home/${USER}/.bashrc ; fi

FROM cmake-options as simulation-tools

# Install gazebo
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros-control

FROM simulation-tools as finalisation

# Give bashrc back to user
WORKDIR /home/${USER}

RUN echo export ROS_MASTER_URI=http://localhost:11311 >> ~/.bashrc
RUN echo export ROS_IP=127.0.0.1 >> ~/.bashrc
RUN chown -R ${USER}:${HOST_GID} .bashrc

# Create catkin workspace
RUN mkdir -p ${HOME}/catkin_ws/src
RUN chown -R ${USER}:${USER} ${HOME}/catkin_ws

USER ${USER}
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && cd ${HOME}/catkin_ws && catkin build"
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
WORKDIR /home/${USER}/catkin_ws
