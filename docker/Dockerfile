################################################################################
# There are three places where your actions are needed in this Dockerfile.
# These are marked with the label `ACTIONS_NEEDED`.
# After you are done with those:
#
# Pull the robetarme noetic base image with
# `docker login registry.gitlab.com`
# `docker pull registry.gitlab.com/certh-iti-robotics-lab/robetarme/robetarme/ros-noetic:base`
#
# Then build with
# `docker compose build`,
#
# and run with
# `docker compose up -d`.
#
# Then gain access to the container with
# `docker attach ros_noetic_xy_development_container`,
# ** where the string after "attach" is the one specified in the `docker-compose.yml` file under
# services/ros_noetic_xy_development/container_name.
#
# Then simply launch your ROS noetic packages.
################################################################################

FROM registry.gitlab.com/certh-iti-robotics-lab/robetarme/robetarme/ros-noetic:base
ARG USER=robetarme_user


# ACTIONS_NEEDED
#-------------------------------------------------------------------------------
# Install your dependencies here
#-------------------------------------------------------------------------------
#RUN apt-get -y update                                                       && \
    #DEBIAN_FRONTEND=noninteractive                                          && \
    #apt install -y --no-install-recommends PACKAGE_1 PACKAGE_2 ...          && \
    #rm -rf /var/lib/apt/lists/*
#-------------------------------------------------------------------------------


# ------------------------------------------------------------------------------
# Place ROS packages into /home/${USER}/ros1_ws workspace and catkin build
# ------------------------------------------------------------------------------
# ACTIONS_NEEDED
# COPY your ROS packages from the host into the container's
# /home/${USER}/ros1_ws/src/. THIS INCLUDES CUSTOM INTERFACES.
# These packages must reside on the host alongside this Dockerfile,
# e.g. if this Dockerfile is at /path/Dockerfile then you should
# `cd /path/`
# `git clone https://github.com/ros/ros_tutorials.git`
# where you should replace the `ros_tutorials` repository with yours.
# This applies to any other directory you want access to in the container.
# The following two lines are indicative and should be replaced by you.
# CAUTION: leave trailing slashes as-is
COPY ros1_talker/ /home/${USER}/catkin_ws/src/ros1_talker/
COPY custom_interfaces_ros1_pkg/ /home/${USER}/catkin_ws/src/custom_interfaces_ros1_pkg/
#-------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# catkin build ROS noetic packages
RUN cd /home/${USER}/catkin_ws                                              && \
    source /opt/ros/noetic/setup.bash                                       && \
    catkin build                                                            && \
    source /home/${USER}/catkin_ws/devel/setup.bash
# ------------------------------------------------------------------------------

#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
# Do not modify below this line
#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

# ------------------------------------------------------------------------------
# Bind mount/shared directory between container and host
# https://docs.docker.com/storage/bind-mounts/
RUN mkdir /home/${USER}/shared
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# Make ${USER} a homeowner
RUN chown -R robetarme_user /home/${USER}
# ------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# ACTIONS_NEEDED
# Your attention is required at file
# docker-compose.yml (:services/volumes).
#-------------------------------------------------------------------------------
# `fix-perms.sh` fixes permissions for volumes when the host user uid does not
# coincide with the container user uid
#-------------------------------------------------------------------------------
COPY fix-perms.sh /fix-perms.sh
COPY entrypointd.sh /entrypoint.sh
RUN chmod +x /fix-perms.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
#-------------------------------------------------------------------------------

WORKDIR /home/${USER}
CMD ["/bin/bash"]
