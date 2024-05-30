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
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"

# Tell docker we want to use bash instead of sh in general
SHELL ["/bin/bash", "-c"]

# create and configure a new user
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y sudo
RUN useradd -m ${USER} && echo "${USER}:${USER}passwd" | chpasswd && adduser ${USER} sudo

### Add a few essential tools and catkin tools
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    git \
    openssh-client \
    python3 \
    net-tools \
    build-essential \
    cmake \
    wget

RUN sh -c \
    'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
    > /etc/apt/sources.list.d/ros-latest.list'

RUN wget http://packages.ros.org/ros.key -O - | apt-key add -
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y python3-catkin-tools

# Create catkin workspace
RUN mkdir -p ${HOME}/catkin_ws/src
RUN chown -R ${USER}:${USER} ${HOME}/catkin_ws

RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    bash-completion \
    silversearcher-ag \
    apt-transport-https \
    less \
    alsa-utils \
    ros-${ROS_DISTRO}-vrpn-client-ros \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-moveit \V
    ros-${ROS_DISTRO}-rosparam-shortcuts \
    pybind11-dev \
    libcgal-dev \
    python3-pip \
    python-dev \
    python3-tk 

RUN  pip install --upgrade pip     

RUN pip install --upgrade pip setuptools
RUN pip install matplotlib
RUN pip install transforms3d
RUN pip install statsmodels
RUN pip install opencv-python
RUN apt remove python3-yaml -y
RUN pip install open3d==0.18.0
RUN pip install --upgrade numpy pandas
RUN pip install opencv-contrib-python
RUN pip install pybind11


FROM ros-ws as libraries

# Setup the project using automatic script
COPY ./scripts/setupControlLasaEnv.sh /run/setupControlLasaEnv.sh
RUN bash /run/setupControlLasaEnv.sh

COPY ./src/polygon_coverage_planning/install/prepare-jenkins-slave.sh /run/prepare-jenkins-slave.sh
RUN bash /run/prepare-jenkins-slave.sh


# Install moveit tools
RUN apt update --fix-missing && apt upgrade -y && apt clean
RUN apt install -y \
    python3-vcstool \
    ros-${ROS_DISTRO}-rqt-joint-trajectory-controller \
    ros-${ROS_DISTRO}-moveit-commander \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-robot-state-publisher

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
    gazebo11 \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros-control

FROM simulation-tools as finalisation

# Give bashrc back to user
WORKDIR /home/${USER}
RUN chown -R ${USER}:${HOST_GID} .bashrc

USER ${USER}
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && cd ${HOME}/catkin_ws && catkin build"
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${HOME}/catkin_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /home/${USER}/catkin_ws

CMD [ "bash" ]
