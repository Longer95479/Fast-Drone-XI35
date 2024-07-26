FROM nvidia/cuda:11.4.3-cudnn8-devel-ubuntu20.04

LABEL maintainer="bkbk_b <1320832053@qq.com>"

SHELL ["bash", "-c"]

USER root

ARG ROS_VERSION=noetic
ARG USE_PROC=12
ENV OPENCV_VERSION=4.5.4
ENV FDRONE_WS=/root/Fast-Drone-XI35

# Install some essential tools
RUN apt-get -y update && \
    TZ=Asia/Beijing apt-get -y install tzdata && \
    apt-get -y install wget curl \
    lsb-release \
    git \
    vim \
    pkg-config \
    openssh-server openssh-client \
    net-tools \
    build-essential \
    htop \
    gdb \
    zip unzip \
    libdw-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libglib2.0-dev \
    libyaml-cpp-dev

#ROS
RUN   sh -c 'echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' && \
      apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
      apt-get update && \
      apt-get install -y  --no-install-recommends \
      ros-${ROS_VERSION}-ros-base \
      ros-${ROS_VERSION}-nav-msgs \
      ros-${ROS_VERSION}-sensor-msgs \
      ros-${ROS_VERSION}-cv-bridge \
      ros-${ROS_VERSION}-rviz \
      ros-${ROS_VERSION}-image-transport-plugins \
      ros-${ROS_VERSION}-pcl-ros \
      ros-${ROS_VERSION}-message-filters \
      ros-${ROS_VERSION}-tf \
      ros-${ROS_VERSION}-catkin \
      ros-${ROS_VERSION}-ddynamic-reconfigure && \
      echo "source /opt/ros/${ROS_VERSION}/setup.bash" >> /root/.bashrc

#unzip the 3rd_party
COPY  ./Docker/3rd_party.zip /root
RUN   unzip /root/3rd_party.zip

#Install OpenCV4 with CUDA
RUN   git clone https://github.com/opencv/opencv_contrib.git -b ${OPENCV_VERSION} && \
      cp /root/boostdesc_extfiles/*.i ./opencv_contrib/modules/xfeatures2d/src/
RUN   apt update && \
      apt install libgtk2.0-dev -y && \
      wget https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip -O opencv.zip && \
      unzip opencv.zip && \
      rm opencv.zip && \
      cd opencv-${OPENCV_VERSION} && \
      mkdir build && cd build && \
      cmake .. \
            -D CMAKE_BUILD_TYPE=RELEASE \
            -D CMAKE_INSTALL_PREFIX=/usr/local/opencv-${OPENCV_VERSION}  \
            -D WITH_CUDA=ON \
            -D WITH_CUDNN=ON \
            -D WITH_CUBLAS=ON \
            -D WITH_TBB=ON \
            -D OPENCV_DNN_CUDA=ON \
            -D OPENCV_ENABLE_NONFREE=ON \
            -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
            -D BUILD_EXAMPLES=OFF \
            -D BUILD_opencv_java=OFF \
            -D BUILD_opencv_python=OFF \
            -D BUILD_TESTS=OFF \
            -D BUILD_PERF_TESTS=OFF \
            -D BUILD_opencv_apps=OFF \
            -D BUILD_LIST=calib3d,features2d,highgui,dnn,imgproc,imgcodecs,\
cudev,cudaoptflow,cudaimgproc,cudalegacy,cudaarithm,cudacodec,cudastereo,\
cudafeatures2d,xfeatures2d,tracking,stereo,\
aruco,videoio,ccalib && \
      make -j $(USE_PROC) && \
      make install 

#Build cv_bridge for opencv4.5.4
RUN   mkdir -p /root/cv_bridge${OPENCV_VERSION}_ws/src && \
      mv /root/cv_bridge${OPENCV_VERSION} /root/cv_bridge${OPENCV_VERSION}_ws/src && \
      /bin/bash -c 'source /opt/ros/${ROS_VERSION}/setup.bash && \ 
      cd /root/cv_bridge${OPENCV_VERSION}_ws && catkin_make && echo "source /root/cv_bridge${OPENCV_VERSION}_ws/devel/setup.bash" >> /root/.bashrc'

#Install realsense
RUN   apt-get install -y software-properties-common && \
      apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \ 
      apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key  F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
      add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u && \
      apt-get update -y && \
      apt-get install -y librealsense2-utils librealsense2-dev librealsense2-dbg

# Install MAVROS
RUN   apt-get install -y ros-${ROS_VERSION}-mavros && \
      cd /opt/ros/${ROS_VERSION}/lib/mavros && \
      ./install_geographiclib_datasets.sh

#Build glog
RUN   apt-get install -y liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev && \
      cd /root/glog && \
      chmod +x autogen.sh configure && ./autogen.sh ./configure && \
      make && make install

#Build ceres
RUN   cd /root/ceres-solver-2.0.0rc1 && mkdir build && cd build && \
      cmake -DCMAKE_BUILD_TYPE=Release -DCUDA=OFF .. && make -j${USE_PROC} && make install && \
      rm -rf ../../ceres-solver-2.0.0rc1

#Install LCM
RUN   cd /root && git clone https://github.com/lcm-proj/lcm && \
      cd lcm && \
      git checkout tags/v1.4.0 && \
      mkdir build && cd build && \
      cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF .. && \
      make -j${USE_PROC} && make install

#Build Fast-Drone-XI35
COPY . ${FDRONE_WS}
WORKDIR ${FDRONE_WS}
RUN catkin_make && echo "source ${FDRONE_WS}/devel/setup.bash" >> /root/.bashrc

ENTRYPOINT [ "cd ${FDRONE_WS}" ]

# Clean up all temp files
RUN apt clean && rm -rf /tmp/* /var/tmp/* /var/lib/apt/lists/* /var/cache/apt/*
