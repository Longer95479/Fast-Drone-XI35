sudo chmod 777 /dev/ttyTHS0
sleep 1;
roslaunch realsense2_camera rs_camera.launch & sleep 3;
roslaunch mavros px4.launch & sleep 4;
roslaunch imu_filter imu_filter.launch & sleep 1;
rosservice call /mavros/set_message_interval 147 10.0 & sleep 1;
find ../ -type f -name 'fast_drone_250.yaml' -exec sed -i 's/use_external_front_end: 1/use_external_front_end: 0/1' {} \;
roslaunch vins fast_drone_250.launch & sleep 4;
wait;
