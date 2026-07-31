FROM arm64v8/ros:noetic-ros-base

RUN apt-get update && apt-get install -y curl gnupg2 && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    gdb \
    apt-utils \
    python3-rosdep \
    python3-pip \
    python3-vcstool \
    python3-pymodbus \
    build-essential \
    ros-noetic-catkin \
    python3-catkin-tools \
    ros-noetic-ros-controllers \
    nano \
    ros-noetic-soem \
    libvlccore-dev \
    libvlc-dev \
    ros-noetic-joy \
    i2c-tools \
    git

RUN pip3 install \
    transforms3d \
    pyserial \
    smbus2 \
    regex \
    matplotlib \
    pynput \
    numpy

# Make the prompt a little nicer
RUN echo "PS1='${debian_chroot:+($debian_chroot)}\u@:\w\$ '" >> /etc/bash.bashrc 

WORKDIR /dingo_ws
COPY /dingo_ws/src /dingo_ws/src
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc

RUN /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash &&\
catkin_make --directory /dingo_ws -DCMAKE_BUILD_TYPE=Debug'

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
RUN echo "source /dingo_ws/devel/setup.bash" >> /etc/bash.bashrc

COPY ros_entrypoint.sh /dingo_ws/ros_entrypoint.sh
RUN chmod +x /dingo_ws/ros_entrypoint.sh

ENTRYPOINT ["./ros_entrypoint.sh"]
CMD [ "bash" ]



