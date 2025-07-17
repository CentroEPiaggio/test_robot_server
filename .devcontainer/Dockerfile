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
# Ensure libraries are found at runtime
# ENV LD_LIBRARY_PATH="/usr/local/lib:/usr/lib:/opt/openrobots/lib:${LD_LIBRARY_PATH}"  # LD_LIBRARY_PATH not in the base image
ENV LD_LIBRARY_PATH="/usr/local/lib:/usr/lib:/opt/openrobots/lib"

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
    libeigen3-dev \
    libpoco-dev \
    libyaml-cpp-dev \
    liburdfdom-dev \
    libfmt-dev \
    lsb-release \
    xterm \
    python3-pip

# install ignition gazebo
# RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
#     && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
#     && apt-get update \
#     && apt-get install  -y --no-install-recommends ignition-fortress

# Install some ROS 2 dependencies to create a cache layer
RUN apt update && apt install -y --no-install-recommends \
    python3-colcon-argcomplete \
    python3-colcon-common-extensions \
    ros-humble-ros-gz \
    # ros-humble-ign-gazebo \
    ros-humble-sdformat-urdf \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros2controlcli \
    ros-humble-controller-interface \
    ros-humble-hardware-interface-testing \
    ros-humble-ament-cmake-clang-format \
    ros-humble-ament-cmake-clang-tidy \
    ros-humble-controller-manager \
    ros-humble-ros2-control-test-assets \
    # libignition-gazebo6-dev \
    # libignition-plugin-dev \
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
    ros-humble-xacro \
    ros-humble-ign-ros2-control \
    ros-humble-rqt-common-plugins

# Install Pinocchio and its dependencies
RUN mkdir -p /etc/apt/keyrings \
    && curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc \
    && echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | tee /etc/apt/sources.list.d/robotpkg.list \
    && apt update && apt install --no-install-recommends -qqy robotpkg-pinocchio

# Run ldconfig to update the linker cache
RUN ldconfig
RUN apt update && apt install --no-install-recommends -qqy \
    ros-humble-pinocchio

# install libfranka
RUN git clone --recurse-submodules https://github.com/frankaemika/libfranka.git  /libfranka \
    && cd libfranka \
    # && git checkout 0.15.0 \
    # && git submodule update \
    && mkdir build \
    && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake -DBUILD_TESTS=OFF .. \
    && make \
    && cpack -G DEB \
    && sudo dpkg -i libfranka*.deb

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
@realtime soft memlock 1048576\n\
@realtime hard rtprio 99\n\
@realtime hard priority 99\n\
@realtime hard memlock 1048576\n' /etc/security/limits.conf

# Choose to run as user
ENV USER=${USER}
USER ${USER}

# Change HOME environment variable
ENV HOME=/home/${USER}

RUN echo "" >> $HOME/.bashrc \
    && echo "export PS1='[\[\033[1;34m\]DOCKER\[\033[0m\]]${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> $HOME/.bashrc \
    && echo "source /opt/ros/humble/setup.bash" >> $HOME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> $HOME/.bashrc \
    && echo "[ -f \"${PWDR}/install/setup.bash\" ] && source \"${PWDR}/install/setup.bash\"" >> $HOME/.bashrc \
    && echo "export RCUTILS_COLORIZED_OUTPUT=1" >> $HOME/.bashrc \
    && echo "export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})'" >> $HOME/.bashrc

# Change entrypoint to source ~/.bashrc and start in ~
COPY ./entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh ; sudo chown ${USER} /entrypoint.sh \
    && echo "cd "${PWDR} >> /entrypoint.sh \
    && echo 'exec bash -i -c $@' >> /entrypoint.sh
    # && cat /entrypoint.sh

# Clean image
RUN sudo apt clean && sudo rm -rf /var/lib/apt/lists/*

ENV COLCON_DEFAULTS_FILE=${PWDR}/.colcon
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
