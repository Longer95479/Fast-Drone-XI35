SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export ROS_LOG_DIR=${SCRIPT_DIR}/../log
sudo chmod 777 /dev/ttyTHS0
sleep 1;
roslaunch realsense2_camera rs_camera.launch & sleep 4;
roslaunch mavros px4.launch & sleep 4;
wait;
