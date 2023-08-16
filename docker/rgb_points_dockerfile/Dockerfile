####################
# In Ubuntu 22.04 (Jammy), using two templates (DOCKWATER & Athackst template from ros2.dockerfile.jinja)
####################

FROM ros:humble-ros-base-jammy

####################
# Setup timezone & locale
####################
ENV TZ=Etc/UTC
RUN echo $TZ > /etc/timezone && \
  ln -fs /usr/share/zoneinfo/$TZ /etc/localtime

RUN sudo apt update && sudo apt install locales \
  && sudo locale-gen en_US en_US.UTF-8 \
  && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV ROSDIST=humble
ARG GZDIST=garden
ENV GZ_VERSION garden
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV DEBIAN_FRONTEND=
  
# Tools necessary and useful during development
ENV DEBIAN_FRONTEND=noninteractive
RUN apt update && apt install --no-install-recommends -y \
        bash-completion \
        build-essential \
        atop \
	ca-certificates \
        cmake \
        cppcheck \
	curl \
        expect \
        gdb \
        git \
        gnutls-bin \
	iputils-ping \
        libbluetooth-dev \
        libccd-dev \
        libcwiid-dev \
	libeigen3-dev \
        libfcl-dev \
        libgflags-dev \
        libgles2-mesa-dev \
        libgoogle-glog-dev \
        libspnav-dev \
        libusb-dev \
	lsb-release \
	net-tools \
	pkg-config \
        protobuf-compiler \
        python3-argcomplete \
        python3-sklearn \
        python3-matplotlib \
        python3-dbg \
        python3-empy \
        python3-setuptools \
        python3-scipy \
        python3-pip \
        python3-venv \
	ruby \
        ros-dev-tools \
        rsync \
        software-properties-common \
	sudo \
        vim \
	wget \
	xvfb \
 && apt clean -qq

# Set up repo to install Ignition, install Gazebo & ROS2 Desktop
RUN /bin/sh -c 'wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null'

RUN apt update && apt install -y --no-install-recommends \
     gz-${GZDIST} \
     ros-${ROSDIST}-desktop \
  && rm -rf /var/lib/apt/lists/* \
  && apt clean -qq

# Install some 'standard' ROS packages and utilities.
RUN apt update && apt install -y --no-install-recommends \
     python3-colcon-common-extensions \
     python3-vcstool \
     python3-sdformat13 \
     ros-${ROSDIST}-actuator-msgs \
     ros-${ROSDIST}-ament-cmake-pycodestyle \
     ros-${ROSDIST}-image-transport \
     ros-${ROSDIST}-image-transport-plugins \
     ros-${ROSDIST}-joy-teleop \
     ros-${ROSDIST}-joy-linux \
     ros-${ROSDIST}-mavros-msgs \
     ros-${ROSDIST}-radar-msgs \
     ros-${ROSDIST}-ros-gzgarden \
     ros-${ROSDIST}-rviz2 \
     ros-${ROSDIST}-xacro \
     ros-${ROSDIST}-navigation2 \ 
     ros-${ROSDIST}-nav2-bringup \ 
     ros-${ROSDIST}-slam-toolbox \
     ros-${ROSDIST}-pointcloud-to-laserscan \
     ros-${ROSDIST}-sensor-msgs-py \
     ros-${ROSDIST}-joint-state-publisher \
     ros-${ROSDIST}-joint-state-publisher-gui \
     ros-${ROSDIST}-robot-localization \
  && sudo rm -rf /var/lib/apt/lists/* \
  && sudo apt clean -qq    

####################
#    rgb_points    #
####################

WORKDIR $ROS_2_WS/src/temp
RUN git clone https://github.com/isabelleviraldo/rgb_points.git .
WORKDIR $ROS_2_WS/src
RUN rsync -av -I temp/ rgb_points/
RUN rm -rf temp

WORKDIR $ROS_2_WS
#resolve dependencies & build ws
RUN rosdep update
RUN rosdep install -i --from-path src --rosdistro humble -y -r
RUN colcon build --symlink-install
##source new workspace
# RUN ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash"]
RUN ["/bin/bash", "-c", "source install/setup.bash"]

#RUN echo "in each window, do $ source /opt/ros2_ws/install/setup.bash" >> readme.txt
RUN curl -o rgb_points_README.txt -L https://raw.githubusercontent.com/isabelleviraldo/rgb_points/main/docker/README_docker.txt
RUN curl -o rgb_points_MENU.txt -L https://raw.githubusercontent.com/isabelleviraldo/rgb_points/main/docker/MENU_docker.txt
RUN curl -o rgb_points_AHC_info.txt -L https://raw.githubusercontent.com/isabelleviraldo/rgb_points/main/docker/AHC_info_docker.txt
