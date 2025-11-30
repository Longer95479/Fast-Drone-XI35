# 功能
VINS-Fusion后端输出的里程计的格式是nav_msgs::Odometry，目前使用到的数据有：

位置p的xyz，姿态四元数 xyzw，速度v的xyz，角速度w的xyz

动捕输出的三个话题（uav0是动捕系统软件中定义的刚体名称）

| topic | 类型 | 成员 |
| --- | --- | --- |
| /vrpn_client_node/uav0/pose | geometry_msgs/PoseStamped | position->x,y,z<br/>orientation->x,y,z,w |
| /vrpn_client_node/uav0/twist | geometry_msgs/TwistStamped | linnear->x,y,z<br/>angel->x,y,x |
| /vrpn_client_node/uav0/accel | geometry_msgs/TwistStamped | linnear->x,y,z<br/>angel->x,y,x |


所以需要将动捕输出的3个Topic整合为一个动捕里程计话题进行集中输出，本ROS包通过固定Pose的数据和时间，将Twist和Accel数据进行线性插值计算出和Pose时间戳对齐的数据，最后打包整合发布在/motion_capture/motion_capture_odom

# 使用
1. 使用动捕系统软件创建刚体，并且打开VRPN输出流（打开速度、加速度输出）
2. 运行脚本

```plain
chmod +x ./shfile/motion_capture_receive_publish.sh
./shfile/motion_capture_receive_publish.sh
```

3. 即可订阅动捕转换的里程计数据，Topic：/motion_capture/motion_capture_odom，类型：nav_msgs/Odometry



# 参数配置
在ROS的config文件夹下可以配置

```yaml
动捕的三个话题的topic
motion_capture_pose_topic: "/vrpn_client_node/uav0/pose"
motion_capture_twist_topic: "/vrpn_client_node/uav0/twist"
motion_capture_accel_topic: "/vrpn_client_node/uav0/accel"
```

