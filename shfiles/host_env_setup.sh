# 用来配置宿主机环境的脚本
#!/bin/bash
set -e
set -o pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 配置参数
ROS_VERSION=noetic
CUDA_ARCH_BIN=8.7
USE_PROC=$(nproc)
ENABLE_NEON="ON"
OPENCV_VERSION=4.5.4
FDRONE_WS=${SCRIPT_DIR}/../
LIB_DIR=$HOME/lib

echo ">>> 开始安装基础工具"
sudo apt-get -y update
sudo apt-get -y install tzdata \
    wget curl lsb-release git vim pkg-config \
    openssh-server openssh-client net-tools build-essential htop gdb \
    zip unzip libdw-dev libatlas-base-dev libeigen3-dev \
    libglib2.0-dev libyaml-cpp-dev libfastrtps-dev libfastcdr-dev

echo ">>> 安装 ROS ${ROS_VERSION}"
sudo sh -c "echo \"deb http://mirrors.ustc.edu.cn/ros/ubuntu/ \$(lsb_release -cs) main\" > /etc/apt/sources.list.d/ros-latest.list"
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
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
    ros-${ROS_VERSION}-roslint \
    ros-${ROS_VERSION}-ddynamic-reconfigure
echo "source /opt/ros/${ROS_VERSION}/setup.bash" >> $HOME/.bashrc

echo ">>> 解压 third_party"
mkdir -p ${LIB_DIR} && cd ${LIB_DIR}
unzip ${FDRONE_WS}/Docker/3rd_party.zip -d ${LIB_DIR}

echo ">>> 安装 OpenCV ${OPENCV_VERSION} (with CUDA)"
cd ${LIB_DIR}
git clone https://github.com/opencv/opencv_contrib.git -b ${OPENCV_VERSION}
cp ${LIB_DIR}/boostdesc_extfiles/*.i ./opencv_contrib/modules/xfeatures2d/src/
sudo apt-get install -y libgtk2.0-dev
wget https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip -O opencv.zip
unzip opencv.zip && rm opencv.zip
cd ${LIB_DIR}/opencv-${OPENCV_VERSION} && mkdir -p build && cd build
cmake .. \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local/opencv-${OPENCV_VERSION} \
    -D WITH_CUDA=ON \
    -D WITH_CUDNN=ON \
    -D WITH_CUBLAS=ON \
    -D CUDNN_VERSION='8.6' \
    -D CUDNN_INCLUDE_DIR=/usr/include/ \
    -D CUDA_ARCH_BIN=${CUDA_ARCH_BIN} \
    -D CUDA_ARCH_PTX="" \
    -D CUDA_FAST_MATH=ON \
    -D WITH_TBB=ON \
    -D BUILD_opencv_python2=OFF \
    -D BUILD_opencv_python3=ON \
    -D OPENCV_DNN_CUDA=ON \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_opencv_java=OFF \
    -D BUILD_opencv_python=OFF \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D BUILD_opencv_apps=OFF \
    -D ENABLE_NEON=${ENABLE_NEON} \
    -D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \
    -D WITH_EIGEN=ON \
    -D WITH_IPP=OFF \
    -D WITH_OPENCL=OFF \
    -D PYTHON3_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.8.so \
    -D BUILD_LIST=calib3d,features2d,highgui,dnn,imgproc,imgcodecs,cudev,cudaoptflow,cudaimgproc,cudalegacy,cudaarithm,cudacodec,cudastereo,cudafeatures2d,xfeatures2d,tracking,stereo,aruco,videoio,ccalib
make -j${USE_PROC}
sudo make install
rm -rf ${LIB_DIR}/opencv_contrib
rm -rf ${LIB_DIR}/boostdesc_extfiles
rm -rf ${LIB_DIR}/opencv-${OPENCV_VERSION}

echo ">>> 构建 cv_bridge for OpenCV${OPENCV_VERSION}"
mkdir -p ${LIB_DIR}/cv_bridge${OPENCV_VERSION}_ws/src
cp -r ${LIB_DIR}/cv_bridge${OPENCV_VERSION} ${LIB_DIR}/cv_bridge${OPENCV_VERSION}_ws/src
source /opt/ros/${ROS_VERSION}/setup.bash
cd ${LIB_DIR}/cv_bridge${OPENCV_VERSION}_ws
catkin_make
echo "source ${LIB_DIR}/cv_bridge${OPENCV_VERSION}_ws/devel/setup.bash" >> $HOME/.bashrc
rm -rf ${LIB_DIR}/cv_bridge${OPENCV_VERSION}

echo ">>> 安装 librealsense"
sudo apt-get install -y software-properties-common
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get update -y
sudo apt install librealsense2-dev=2.55.1-0~realsense.3336 \
                 librealsense2=2.55.1-0~realsense.3336 \
                 librealsense2-utils=2.55.1-0~realsense.3336 \
                 librealsense2-dbg=2.55.1-0~realsense.3336 \
                 librealsense2-gl=2.55.1-0~realsense.3336 -y
sudo apt-mark hold librealsense2 librealsense2-dev librealsense2-utils librealsense2-dbg

echo ">>> 安装 MAVROS"
sudo apt-get install -y ros-${ROS_VERSION}-mavros
cd /opt/ros/${ROS_VERSION}/lib/mavros
sudo ./install_geographiclib_datasets.sh

echo ">>> 构建 glog"
sudo apt-get install -y liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev
cd ${LIB_DIR}/glog
chmod +x autogen.sh configure
./autogen.sh
./configure
make -j${USE_PROC}
sudo make install
rm -rf ${LIB_DIR}/glog

echo ">>> 构建 ceres"
cd ${LIB_DIR}/ceres-solver-2.0.0rc1
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCUDA=OFF ..
make -j${USE_PROC}
sudo make install
rm -rf ${LIB_DIR}/ceres-solver-2.0.0rc1

echo ">>> 安装 LCM"
cd ${LIB_DIR}
git clone https://github.com/lcm-proj/lcm
cd lcm
git checkout tags/v1.4.0
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF ..
make -j${USE_PROC}
sudo make install
rm -rf ${LIB_DIR}/lcm

echo ">>> 安装 onnxruntime"
sudo apt install python3-pip -y
pip3 install onnxruntime -i https://pypi.tuna.tsinghua.edu.cn/simple

echo ">>> 安装 gscam 依赖"
sudo apt install -y gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev
sudo apt install -y ros-${ROS_VERSION}-camera-calibration-parsers ros-${ROS_VERSION}-camera-info-manager

echo ">>> 构建 OpenCV 3.4.16"
cp ${FDRONE_WS}/Docker/opencv-3.4.16.zip ${LIB_DIR}/
cp ${FDRONE_WS}/Docker/opencv_contrib-3.4.16.zip ${LIB_DIR}/
cd  ${LIB_DIR}/
unzip opencv-3.4.16.zip
unzip opencv_contrib-3.4.16.zip
rm -rf ${LIB_DIR}/opencv-3.4.16.zip
rm -rf ${LIB_DIR}/opencv_contrib-3.4.16.zip
cd opencv-3.4.16
mkdir -p build && cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local/opencv-3.4.16 \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.4.16/modules \
    -D WITH_CUDA=ON \
    -D CUDA_ARCH_BIN=8.7 \
    -D CUDNN_VERSION='8.6' \
    -D CUDNN_INCLUDE_DIR=/usr/include/ \
    -D CUDA_ARCH_PTX="" \
    -D ENABLE_FAST_MATH=ON \
    -D CUDA_FAST_MATH=ON \
    -D WITH_CUBLAS=ON \
    -D WITH_LIBV4L=ON \
    -D WITH_GSTREAMER=ON \
    -D WITH_GSTREAMER_0_10=OFF \
    -D WITH_QT=ON \
    -D WITH_OPENGL=ON \
    -D CUDA_NVCC_FLAGS="--expt-relaxed-constexpr" \
    -D WITH_TBB=ON ..
make -j${USE_PROC}
sudo make install
rm -rf ${LIB_DIR}/opencv-3.4.16
rm -rf ${LIB_DIR}/opencv_contrib-3.4.16

echo ">>> 安装其他依赖"
pip3 install scikit-image

echo ">>> 脚本执行完毕"
