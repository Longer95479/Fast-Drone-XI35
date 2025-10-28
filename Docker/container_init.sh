ROS_VERSION=noetic
OPENCV_VERSION=4.5.4
FDRONE_WS=/root/Fast-Drone-XI35
LOG_DIR=${FDRONE_WS}/log
LOG_FILE=${LOG_DIR}/catkin_build.log

# modify mavros
sed -i 's|/dev/ttyACM0:57600|/dev/ttyTHS0:921600|1' /opt/ros/noetic/share/mavros/launch/px4.launch

# LCM Config
ifconfig wlan1 multicast
route add -net 224.0.0.0 netmask 240.0.0.0 dev wlan1

# start service
service ssh start
/daemon/nvargus-daemon &

# add ROS log dir env variable
if ! grep -q "export ROS_LOG_DIR=" /root/.bashrc; then
    echo 'export ROS_LOG_DIR=/Fast-Drone-XI35/log' >> /root/.bashrc
    echo "Added ROS_LOG_DIR to /root/.bashrc"
fi

# Sourcing ROS environment
source /opt/ros/${ROS_VERSION}/setup.bash
source /root/cv_bridge${OPENCV_VERSION}_ws/devel/setup.bash

# Build Fast-Drone-XI35 workspace
cd ${FDRONE_WS}
mkdir -p ${LOG_DIR}
catkin_make 2>&1 | tee ${LOG_FILE}
if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo -e " catkin_make succeeded " | tee -a ${LOG_FILE}
    if ! grep -q "${FDRONE_WS}/devel/setup.bash" /root/.bashrc; then
        echo "source ${FDRONE_WS}/devel/setup.bash" >> /root/.bashrc
    fi
else
    echo " catkin_make failed " | tee -a ${LOG_FILE}
fi

# login as root
su - root
