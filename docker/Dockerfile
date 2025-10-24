FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y python3-pip \
    git \
    python3-colcon-common-extensions \
    nano \
    # mesa-common-dev \
    xauth \
    libgl1 \
    # libgl1-mesa-dev \
    # libegl1-mesa-dev \
    # libgles2-mesa-dev \
    pkg-config

ENV USERNAME=developer
# we use uid of 1000, seems to be default in ubuntu but change if different
RUN useradd -U --uid 1000 -ms /bin/bash $USERNAME \
    && echo "$USERNAME:$USERNAME" | chpasswd \
    && adduser $USERNAME sudo \
    && adduser $USERNAME dialout \
    && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

# Commands below run as the developer user
USER $USERNAME

WORKDIR /home/$USERNAME
# building pybullet freezes the raspberrypi so manually limit parallelization
RUN git clone --depth 1 --branch 3.25 https://github.com/bulletphysics/bullet3/
WORKDIR /home/$USERNAME/bullet3
RUN sed -i -e 's%make -j $(command nproc 2>/dev/null || echo 12)%make -j 2%g' build_cmake_pybullet_double.sh
RUN cat build_cmake_pybullet_double.sh
RUN bash build_cmake_pybullet_double.sh
ENV PYTHONPATH=/home/$USERNAME/bullet3/build_cmake/examples/pybullet

WORKDIR /tmp
COPY docker/requirements.txt ./requirements.txt
RUN pip install -r requirements.txt
ENV PATH=$PATH:/home/$USERNAME/.local/bin

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME
RUN mkdir -p /home/$USERNAME/ros_ws/src

# COPY ros_arm_controller /home/$USERNAME/ros_ws/src
# COPY ros_cobot_controller /home/$USERNAME/ros_ws/src
# COPY ros_sim /home/$USERNAME/ros_ws/src
# COPY scripts /home/$USERNAME/scripts

RUN sudo apt install -y python3-rosdep python3-vcstool
RUN rosdep update
RUN cd /home/$USERNAME/ros_ws/src && git clone --branch humble https://github.com/ros-planning/moveit2_tutorials
RUN cd /home/$USERNAME/ros_ws/src && vcs import < moveit2_tutorials/moveit2_tutorials.repos
RUN cd /home/$USERNAME/ros_ws/src && sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
ENV MAKEFLAGS="-j1"
SHELL ["/bin/bash", "-c"]
RUN cd /home/$USERNAME/ros_ws/ && source /opt/ros/humble/setup.bash && colcon build --parallel-workers 6 --cmake-args -DCMAKE_BUILD_TYPE=Release
# move it 2 claims that only cyclonedds is supported?
RUN sudo apt install -y ros-humble-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN sudo apt install -y ros-humble-rqt*