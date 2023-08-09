##############################################
# Created from template ros2.dockerfile.jinja
##############################################
#Tyler, based on Athackst template
###########################################
# Iron with rgb_points
###########################################
FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

RUN dpkg --configure -a

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    sudo \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Iron
RUN sudo add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y --no-install-recommends \
    ros-iron-ros-base \
    python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=iron
ENV AMENT_PREFIX_PATH=/opt/ros/iron
ENV COLCON_PREFIX_PATH=/opt/ros/iron
ENV LD_LIBRARY_PATH=/opt/ros/iron/lib
ENV PATH=/opt/ros/iron/bin:$PATH
ENV PYTHONPATH=/opt/ros/iron/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV DEBIAN_FRONTEND=

###########################################
#  Develop image 
###########################################
FROM base AS dev

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  openssh-client \
  python3-argcomplete \
  python3-pip \
  ros-dev-tools \
  nano \
  && rm -rf /var/lib/apt/lists/*

RUN rosdep init || echo "rosdep already initialized"

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Set up autocompletion for user
RUN apt-get update && apt-get install -y git-core bash-completion \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
  && rm -rf /var/lib/apt/lists/* 

ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

#install python packages
RUN pip3 install -U \
    argcomplete \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures 

######################## Tyler's changes ##########################
#I did apt-get on my machine instead of pip3, not sure if there would be any difference.
# except rsync, these are inputs in my rgb_points package
RUN apt-get update && apt-get install -y \
    python3-sklearn \
    python3-matplotlib \
    python3-scipy \
    ros-iron-sensor-msgs-py \
    rsync 

RUN pip3 install -U \
    transforms3d \
    opencv-python

### BRING IN rgb_points PACKAGE 
#define ENV variable
ENV ROS_2_WS /opt/ros2_ws  
#define working directory, which also creates it, using variable $ROS_2_WS defined above
WORKDIR $ROS_2_WS      

### following the "create a workspace flow" mixed with "new package flow" from Tyler's ROS2 cheat sheet     
#prep for cloning source
#RUN source /opt/ros/iron/setup.bash ##this was erroring on docker build
RUN ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash &&  echo ${AMENT_PREFIX_PATH}"]
RUN mkdir -p $ROS_2_WS/src
#how docker likes to change working directory. RUN cd might also work...
WORKDIR $ROS_2_WS/src
RUN ros2 pkg create --build-type ament_python  rgb_points  --dependencies rclpy
#make dir for cloning package. I had trouble taking it straight into src/rgb_points
WORKDIR $ROS_2_WS/src/temp

RUN git clone https://github.com/isabelleviraldo/rgb_points.git .

WORKDIR $ROS_2_WS/src
# copy code from temp to src/rgb_points, overwriting any files that already exist
RUN rsync -av -I temp/ rgb_points/
# delete temp now that it's copied over.
RUN rm -rf temp

WORKDIR $ROS_2_WS
#resolve dependencies
RUN rosdep update
RUN rosdep install -i --from-path src --rosdistro iron -y -r
#build workspace
RUN colcon build --symlink-install
##this was the way for athackst to build, but idk what most of this does and mine seems to work. 
# RUN colcon \
#     build \
#     --cmake-args \
#       -DSECURITY=ON --no-warn-unused-cli \
#     --symlink-install
##source new workspace
# RUN ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash"]
RUN ["/bin/bash", "-c", "source install/setup.bash"]

### setup entrypoint script for sourcing
## this is causing the whole container to exit, not going to use it for now...
# WORKDIR /usr/local/bin
# RUN curl -o entry.sh -L https://raw.githubusercontent.com/hazelrah2/2023interns/main/entry2.sh
# RUN chmod +x entry.sh
# ENTRYPOINT ["entry.sh"]

### include a bagfile so we can test rgb_points works
RUN mkdir -p $ROS_2_WS/bagfiles
#copy from . on local machine to docker location
# COPY two_hand_wave.bag  $ROS_2_WS/bagfiles
WORKDIR $ROS_2_WS/src
# RUN echo "in each window, do $ source /opt/ros2_ws/install/setup.bash" >> readme.txt

RUN curl -o README.txt -L https://raw.githubusercontent.com/isabelleviraldo/rgb_points/main/docker/README_docker.txt
RUN curl -o MENU.txt -L https://raw.githubusercontent.com/isabelleviraldo/rgb_points/main/docker/MENU_docker.txt
RUN curl -o AHC_info.txt -L https://raw.githubusercontent.com/isabelleviraldo/rgb_points/main/docker/AHC_info_docker.txt

