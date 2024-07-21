sudo chmod 777 /dev/ttyTHS0 & sleep 2;
source jetson_csi_cam_ros_ws/devel/setup.bash & sleep 1;
roslaunch jetson_csi_cam jetson_csi_cam.launch & sleep 2;
roslaunch mavros px4.launch & sleep 10;
roslaunch vins xi35_mono_downward.launch & sleep 4;
wait;
