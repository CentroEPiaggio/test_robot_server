# Change ROS version in build.sh, not here!
ARG BASE_IMAGE=osrf/ros
ARG BASE_TAG=noetic-desktop
# ARG LOCAL_LIBFRANKA_VERSION=0.13.2

FROM ${BASE_IMAGE}:${BASE_TAG}

# Prevent bash to ask for user input which may break the building process
ENV DEBIAN_FRONTEND=noninteractive

# Install sudo, and some essential packages
RUN apt-get update && apt-get install --no-install-recommends -qqy \
    bash-completion \
    build-essential \
    git \
    sudo \
    wget \
    gdb \
    nano \
    iputils-ping

# Install packages specific for ROS 1
# RUN --mount=type=cache,sharing=locked,target=/var/cache/apt --mount=type=cache,sharing=locked,target=/var/lib/apt \
RUN apt-get update && apt-get install --no-install-recommends -qqy \
    ros-${ROS_DISTRO}-catkin \
    python3-catkin-tools \
    python3-osrf-pycommon \
    ros-${ROS_DISTRO}-gazebo-ros \
	ros-${ROS_DISTRO}-rqt \
	ros-${ROS_DISTRO}-rqt-common-plugins \
	ros-${ROS_DISTRO}-controller-interface \
	ros-${ROS_DISTRO}-controller-manager \
	ros-${ROS_DISTRO}-ros-control \
	ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-rviz-visual-tools \
	ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-boost-sml

# depends of libfranka
RUN apt-get update && apt-get install --no-install-recommends -qqy \
    libpoco-dev \
    libeigen3-dev \
    libfmt-dev \
    lsb-release \
    curl

# install pinocchio
RUN mkdir -p /etc/apt/keyrings \
    && curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc \
    && echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | tee /etc/apt/sources.list.d/robotpkg.list \
    && apt-get update && apt-get install --no-install-recommends -qqy robotpkg-pinocchio

# install libfranka
RUN git clone --recurse-submodules https://github.com/frankaemika/libfranka.git  libfranka\
    && cd libfranka \
    && git checkout 0.13.2 \
    && git submodule update \
    && mkdir build \
    && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake -DBUILD_TESTS=OFF .. \
    && make \
    && cpack -G DEB \
    && sudo dpkg -i libfranka*.deb

# Install packages specific for Franka and qb devices
RUN apt-get update && apt-get install --no-install-recommends -qqy \
    # ros-${ROS_DISTRO}-libfranka \
	# ros-${ROS_DISTRO}-franka-ros \
    ros-${ROS_DISTRO}-qb-move

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

# # Ensure real-time limits are set in /etc/security/limits.conf
# RUN grep -qxF '@realtime soft rtprio 99' /etc/security/limits.conf || echo "@realtime soft rtprio 99" >> /etc/security/limits.conf && \
#     grep -qxF '@realtime soft priority 99' /etc/security/limits.conf || echo "@realtime soft priority 99" >> /etc/security/limits.conf && \
#     grep -qxF '@realtime soft memlock 102400' /etc/security/limits.conf || echo "@realtime soft memlock 102400" >> /etc/security/limits.conf && \
#     grep -qxF '@realtime hard rtprio 99' /etc/security/limits.conf || echo "@realtime hard rtprio 99" >> /etc/security/limits.conf && \
#     grep -qxF '@realtime hard priority 99' /etc/security/limits.conf || echo "@realtime hard priority 99" >> /etc/security/limits.conf && \
#     grep -qxF '@realtime hard memlock 102400' /etc/security/limits.conf || echo "@realtime hard memlock 102400" >> /etc/security/limits.conf

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

# Set up environment
COPY ./.devcontainer/update_bashrc.sh /sbin/update_bashrc.sh

RUN sudo chmod +x /sbin/update_bashrc.sh ; sudo chown ${USER} /sbin/update_bashrc.sh \
    && echo 'echo "source '${PWDR}'/devel/setup.bash" >> ~/.bashrc' >> /sbin/update_bashrc.sh \
    && cat /sbin/update_bashrc.sh \
    && sync ; /bin/bash -c /sbin/update_bashrc.sh ; sudo rm /sbin/update_bashrc.sh

# Change entrypoint to source ~/.bashrc and start in ~
COPY ./.devcontainer/entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh ; sudo chown ${USER} /entrypoint.sh \
    && echo "cd "${PWDR} >> /entrypoint.sh \
    && echo 'exec bash -i -c $@' >> /entrypoint.sh \
    && cat /entrypoint.sh

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
