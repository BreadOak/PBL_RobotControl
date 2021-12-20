FROM nvidia/cudagl:11.4.0-base-ubuntu18.04

RUN apt-get update && apt-get install locales sudo

ARG DEBIAN_FRONTEND=noninteractive

RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en  
ENV LC_ALL en_US.UTF-8  

RUN apt-get update &&  apt-get install -y -qq --no-install-recommends \
    libgl1 \
    libxext6 \ 
    libx11-6 \
   && rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
RUN sudo apt-get update && apt-get install -y --no-install-recommends apt-utils
RUN sudo apt-get install -y wget
RUN sudo apt-get install -y sudo curl
RUN sudo apt-get  update && sudo apt-get  install -y curl gnupg2 lsb-release
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Minsk
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
ENV DEBIAN_FRONTEND=noninteractive 
RUN sudo apt-get update && DEBIAN_FRONTEND=noninteractive && sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  wget
# install some pip packages needed for testing
RUN python3 -m pip install -U \
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
  pytest-rerunfailures \
  pytest \
  pytest-cov \
  pytest-runner \
  setuptools
# install Fast-RTPS dependencies
RUN sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
RUN sudo apt install --no-install-recommends -y \
  libcunit1-dev

ARG UNAME="junyoung"
ARG UID=1000
ARG GID=1000
ARG HOME=/home/${UNAME}
RUN useradd -rm -d ${HOME} -s /bin/bash -g root -G sudo,audio,video,plugdev -u ${UID} ${UNAME}
RUN mkdir -p /etc/sudoers.d && \
    echo "${UNAME} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${UNAME} && \
    chmod 0440 /etc/sudoers.d/${UNAME}

USER ${UNAME}
WORKDIR $HOME

RUN sudo rosdep init
RUN rosdep update

RUN sudo apt-get install -y ros-dashing-desktop ros-dashing-ros-base ros-dashing-rmw-cyclonedds* 

RUN sudo apt-get update && sudo apt-get install -y \
    libusb-dev \
    libspnav-dev \
    libbluetooth-dev \
    libcwiid-dev \
    qt5-default \
    libncurses5-dev \
    x11-apps \
    tmux \
    curl \
    wget \
    vim \
    sudo \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    unzip \
    locales \
    ntp \
    whois \
    tree

# init catkin workspace
RUN mkdir -p $HOME/colcon_ws/src
RUN /bin/bash -c "source /opt/ros/dashing/setup.bash; cd ~/colcon_ws/;colcon build --symlink-install"

RUN echo "source /opt/ros/dashing/setup.bash" >> $HOME/.bashrc
RUN echo "source ~/colcon_ws/install/local_setup.bash" >> $HOME/.bashrc
RUN echo "" >> $HOME/.bashrc
RUN echo "export ROS_DOMAIN_ID=0" >> $HOME/.bashrc
RUN echo "" >> $HOME/.bashrc
RUN echo "# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> $HOME/.bashrc
RUN echo "# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> $HOME/.bashrc
RUN echo "# export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})'" >> $HOME/.bashrc
RUN echo "# export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{time}]: {message}'" >> $HOME/.bashrc
RUN echo "# export RCUTILS_COLORIZED_OUTPUT=1" >> $HOME/.bashrc
RUN echo "# export RCUTILS_LOGGING_USE_STDOUT=0" >> $HOME/.bashrc
RUN echo "# export RCUTILS_LOGGING_BUFFERED_STREAM=0" >> $HOME/.bashrc
RUN echo "" >> $HOME/.bashrc
RUN echo "" >> $HOME/.bashrc
RUN echo "alias cs='cd ~/colcon_ws/src'" >> $HOME/.bashrc
RUN echo "alias cm='cd ~/colcon_ws/ && colcon build --symlink-install'" >> $HOME/.bashrc
RUN echo "alias eb='vi ~/.bashrc'" >> $HOME/.bashrc
RUN echo "alias sb='source ~/.bashrc'" >> $HOME/.bashrc
RUN echo "alias rt='ros2 topic list'" >> $HOME/.bashrc
RUN echo "alias re='ros2 topic echo'" >> $HOME/.bashrc
RUN echo "alias rn='ros2 node list'" >> $HOME/.bashrc
RUN echo "alias af='ament_flake8'" >> $HOME/.bashrc
RUN echo "alias ac='ament_cpplint'" >> $HOME/.bashrc
RUN echo "alias ros2_init='. /opt/ros/dashing/setup.bash'" >> $HOME/.bashrc
RUN echo "" >> $HOME/.bashrc
RUN echo "alias killgazebo='killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient'" >> $HOME/.bashrc

EXPOSE 80
EXPOSE 443