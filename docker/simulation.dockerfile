FROM nvidia/cudagl:11.3.0-devel-ubuntu20.04

# Please contact with me if you have problems
LABEL maintainer="Zipeng Dai <daizipeng@bit.edu.cn>"
ENV DEBIAN_FRONTEND=noninteractive
ARG PYTHON_VERSION=3.9
# TODO：网络不好的话可以走代理
ENV http_proxy=http://127.0.0.1:8889
ENV https_proxy=http://127.0.0.1:8889

# Setup basic packages
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata software-properties-common && \
    add-apt-repository ppa:deadsnakes/ppa
RUN apt-get update && apt-get install -y --no-install-recommends \
    python${PYTHON_VERSION} \
    python${PYTHON_VERSION}-dev \
    $( [ ${PYTHON_VERSION%%.*} -ge 3 ] && echo "python${PYTHON_VERSION%%.*}-distutils" ) \
    python3-pip \
    build-essential \
    git \
    git-lfs \
    curl \
    vim \
    tmux \
    gnupg2 \
    lsb-release \
    ca-certificates \
    libjpeg-dev \
    libpng-dev \
    libglfw3-dev \
    libglm-dev \
    libx11-dev \
    libomp-dev \
    libegl1-mesa-dev \
    pkg-config \
    wget \
    gedit \
    zip \
    unzip \
    libcgal-dev \
    cpufrequtils \
    libompl-dev
# RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python${PYTHON_VERSION} 1

# Install cmake
RUN wget https://github.com/Kitware/CMake/releases/download/v3.16.0/cmake-3.16.0-Linux-x86_64.sh
RUN mkdir /opt/cmake
RUN sh /cmake-3.16.0-Linux-x86_64.sh --prefix=/opt/cmake --skip-license
RUN ln -s /opt/cmake/bin/cmake /usr/local/bin/cmake
RUN cmake --version

# -----------------------------------------------------
# ROS and relevant infra
ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -
RUN apt update
RUN apt-get install -y ros-noetic-desktop-full
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bash_profile

# -----------------------------------------------------
# PX4 and relevant infra
WORKDIR /workspace
RUN git clone https://github.com/PX4/PX4-Autopilot.git -b v1.15.2 --recursive
COPY docker/requirements.txt /workspace/PX4-Autopilot/requirements.txt
COPY docker/ubuntu.sh /workspace/PX4-Autopilot/ubuntu.sh 
RUN cd PX4-Autopilot/ && bash ubuntu.sh --no-nuttx && make clean
RUN pip3 install --upgrade numpy
RUN cd PX4-Autopilot/ && DONT_RUN=1 make px4_sitl_default gazebo-classic
WORKDIR /workspace
# RUN wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage && chmod +x ./QGroundControl.AppImage 
RUN apt-get install -y ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-vision-msgs ros-${ROS_DISTRO}-octomap* && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash install_geographiclib_datasets.sh
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && mkdir -p /workspace/catkin_ws/src && cd /workspace/catkin_ws/ && catkin_make"
RUN echo "source /workspace/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash /workspace/PX4-Autopilot/ /workspace/PX4-Autopilot/build/px4_sitl_default" >> ~/.bashrc
RUN echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/workspace/PX4-Autopilot/" >> ~/.bashrc
RUN echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/workspace/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic" >> ~/.bashrc

# -----------------------------------------------------
# # Setup habitat-sim
# WORKDIR /workspace
# RUN git clone https://github.com/Fanxing-LI/habitat-sim
# RUN cd habitat-sim && pip3 install -r requirements.txt
# RUN cd habitat-sim && pip3 install pillow==8.3.2 && python3 setup.py install --headless

# # Install challenge specific habitat-lab
# WORKDIR /workspace
# RUN git clone --branch stable https://github.com/facebookresearch/habitat-lab.git
# RUN cd habitat-lab && pip3 install -e habitat-lab/

# # Our project
# RUN pip3 install stable-baselines3==2.2.1 torchvision scikit-learn tensorboard geographiclib plotly
# RUN pip3 install --upgrade netifaces matplotlib 
# RUN find /usr/lib/python3/dist-packages/Cryptodome/ -type f -name "*.cpython-38-x86_64-linux-gnu.so" \
#     -exec bash -c 'cp "$1" "${1/cpython-38/cpython-39}"' _ {} \;
WORKDIR /workspace/ROS/src

# -----------------------------------------------------
RUN rm -rf /var/lib/apt/lists/* && apt-get clean
ENV GLOG_minloglevel=2
ENV MAGNUM_LOG="quiet"
# TODO：如果走了代理、但是想镜像本地化到其它机器，记得清空代理（或者容器内unset）
# ENV http_proxy=
# ENV https_proxy=
# ENV no_proxy=
CMD ["/bin/bash"]
WORKDIR /workspace/ROS