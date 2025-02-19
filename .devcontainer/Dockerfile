# Change ROS version in build.sh, not here!
ARG BASE_IMAGE=ros
ARG BASE_TAG=humble-ros-base
# ARG LOCAL_LIBFRANKA_VERSION=0.13.2

FROM ${BASE_IMAGE}:${BASE_TAG}

# Prevent bash to ask for user input which may break the building process
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

# Install sudo, and some essential packages
RUN apt update && apt install --no-install-recommends -qqy \
    bash-completion \
    build-essential \
    git \
    sudo \
    wget \
    gdb \
    nano \
    iputils-ping \
    curl \
    openssh-client \
    python3-colcon-argcomplete \
    python3-colcon-common-extensions \
    libeigen3-dev \
    libpoco-dev \
    libyaml-cpp-dev \
    liburdfdom-dev \
    libfmt-dev \
    lsb-release 

# Install some ROS 2 dependencies to create a cache layer
RUN sudo apt update && sudo apt install -y --no-install-recommends \
    ros-humble-ros-gz \
    ros-humble-sdformat-urdf \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros2controlcli \
    ros-humble-controller-interface \
    ros-humble-hardware-interface-testing \
    ros-humble-ament-cmake-clang-format \
    ros-humble-ament-cmake-clang-tidy \
    ros-humble-controller-manager \
    ros-humble-ros2-control-test-assets \
    libignition-gazebo6-dev \
    libignition-plugin-dev \
    ros-humble-hardware-interface \
    ros-humble-control-msgs \
    ros-humble-backward-ros \
    ros-humble-generate-parameter-library \
    ros-humble-realtime-tools \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-broadcaster \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-kinematics \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-ros-visualization \
    ros-humble-joint-trajectory-controller \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-rviz2 \
    ros-humble-xacro 

# Install packages specific for ROS 1
# RUN --mount=type=cache,sharing=locked,target=/var/cache/apt --mount=type=cache,sharing=locked,target=/var/lib/apt \
# RUN apt update && apt install --no-install-recommends -qqy \
#     ros-${ROS_DISTRO}-catkin \
#     python3-catkin-tools \
#     python3-osrf-pycommon \
#     ros-${ROS_DISTRO}-gazebo-ros \
# 	ros-${ROS_DISTRO}-rqt \
# 	ros-${ROS_DISTRO}-rqt-common-plugins \
# 	ros-${ROS_DISTRO}-controller-interface \
# 	ros-${ROS_DISTRO}-controller-manager \
# 	ros-${ROS_DISTRO}-ros-control \
# 	ros-${ROS_DISTRO}-ros-controllers \
#     ros-${ROS_DISTRO}-tf2-geometry-msgs \
#     ros-${ROS_DISTRO}-rviz-visual-tools \
# 	ros-${ROS_DISTRO}-plotjuggler-ros \
#     ros-${ROS_DISTRO}-gazebo-ros-pkgs \
#     ros-${ROS_DISTRO}-boost-sml

# Install Pinocchio and its dependencies
# RUN mkdir -p /etc/apt/keyrings \
#     && curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc \
#     && echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | tee /etc/apt/sources.list.d/robotpkg.list \
#     && apt update && apt install --no-install-recommends -qqy robotpkg-pinocchio
# # Ensure libraries are found at runtime
# ENV LD_LIBRARY_PATH="/usr/local/lib:/usr/lib:/opt/openrobots/lib:${LD_LIBRARY_PATH}"
# # Run ldconfig to update the linker cache
# RUN ldconfig
# RUN apt update && apt install --no-install-recommends -qqy \
#     ros-humble-pinocchio

# install libfranka
# RUN git clone --recurse-submodules https://github.com/frankaemika/libfranka.git  /libfranka \
#     && cd libfranka \
#     && git checkout 0.15.0 \
#     && git submodule update \
#     && mkdir build \
#     && cd build \
#     && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake -DBUILD_TESTS=OFF .. \
#     && make \
#     && cpack -G DEB \
#     && sudo dpkg -i libfranka*.deb

# default USER
ARG MYUID=1000
ARG MYGID=1000
ARG USER=docker
ARG PWDR=/
ENV WORKDIR=${PWDR}

RUN OLD_USER=$(awk -v uid="$MYUID" -F ':' '$3==uid {print $1}' /etc/passwd) \
    && if [ -z "${OLD_USER}" ]; then \
        # There is no existing UID equal to MYUID.
        # Create the same user as the host itself. (By default Docker creates the container as root, which is not recommended.)
        addgroup --gid ${MYGID} ${USER} \
        && adduser --gecos "ROS User" --disabled-password --uid ${MYUID} --gid ${MYGID} ${USER} \
        && usermod -a -G dialout ${USER} \
        && echo ${USER}" ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/99_aptget \
        && chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget ; \
    else \
        usermod -l ${USER} ${OLD_USER} \
        && sudo usermod -d /home/${USER} -m ${USER} \
        && echo ${USER}" ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/99_aptget \
        && chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget ; \
    fi
   
# - add real-time capabilities - #    
# Ensure user exists and is in the 'realtime' group
RUN groupadd realtime \
    && if id "$USER" &>/dev/null; then \
      usermod -a -G realtime "$USER"; \
    else \
      groupadd -g $MYGID $USER && \
      useradd -m -u $MYUID -g $MYGID -G realtime -s /bin/bash $USER; \
    fi \
    && echo "$USER ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Insert real-time limits at the end of the file
RUN sed -i '/# End of file/i \
@realtime soft rtprio 99\n\
@realtime soft priority 99\n\
@realtime soft memlock 102400\n\
@realtime hard rtprio 99\n\
@realtime hard priority 99\n\
@realtime hard memlock 102400\n' /etc/security/limits.conf

# Choose to run as user
ENV USER=${USER}
USER ${USER}

# Change HOME environment variable
ENV HOME=/home/${USER}

# Set up .bashrc
# COPY ./.devcontainer/update_bashrc.sh /sbin/update_bashrc.sh
# RUN sudo chmod +x /sbin/update_bashrc.sh ; sudo chown ${USER} /sbin/update_bashrc.sh \
#     && echo 'echo "source '${PWDR}'/install/setup.bash" >> ~/.bashrc' >> /sbin/update_bashrc.sh \
#     && echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc \
#     && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USER/.bashrc \
#     && cat /sbin/update_bashrc.sh \
#     && sync ; /bin/bash -c /sbin/update_bashrc.sh ; sudo rm /sbin/update_bashrc.sh
RUN echo "" >> $HOME/.bashrc \
    && echo "source /opt/ros/humble/setup.bash" >> $HOME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> $HOME/.bashrc \
    && echo "source ${PWDR}/install/setup.bash" >> $HOME/.bashrc

# Install the missing ROS 2 dependencies for franka_ros2
COPY ./src/franka_ros2/ /tmp_ws/
RUN sudo chown -R $USER:$USER /tmp_ws \
    && vcs import /tmp_ws < /tmp_ws/franka.repos --recursive --skip-existing \
    && cd /tmp_ws \
    && sudo apt update \
    && rosdep update \
    && rosdep install --from-paths /tmp_ws --ignore-src --rosdistro humble -y \
    && rm -rf /home/$USER/.ros

# Change entrypoint to source ~/.bashrc and start in ~
COPY ./.devcontainer/entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh ; sudo chown ${USER} /entrypoint.sh \
    && echo "cd "${PWDR} >> /entrypoint.sh \
    && echo 'exec bash -i -c $@' >> /entrypoint.sh \
    && cat /entrypoint.sh

# Clean image
RUN sudo apt clean && sudo rm -rf /var/lib/apt/lists/*

ENV COLCON_DEFAULTS_FILE=${PWDR}/.colcon
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
