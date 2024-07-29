#modify mavros
sed -i 's|/dev/ttyACM0:57600|/dev/ttyTHS0:921600|1' /opt/ros/noetic/share/mavros/launch/px4.launch

#start service
service ssh start
/daemon/nvargus-daemon &

#login as root
su - root
