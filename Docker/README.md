# Docker 使用

## nvidia jetson平台

## x86 平台

⚠️ 前置要求

1. 宿主机必须正确安装好Nvidia驱动，使用nvidia-smi测试输出，CUDA Version字段必须大于等于11.8

2. 宿主机必须正确安装好Nvidia Docker Toolkit
```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

3. 确认电脑的CUDA_ARCH_BIN，对Dockerfile.pc的ARG CUDA_ARCH_BIN=7.5（默认）进行修改

```bash
git clone https://github.com/NVIDIA-AI-IOT/deepstream_tlt_apps.git
cd deepstream_tlt_apps/TRT-OSS/x86
nvcc deviceQuery.cpp -o deviceQuery
./deviceQuery
### 输出
Detected 1 CUDA Capable device(s)

Device 0: "NVIDIA GeForce GTX 1660 SUPER"
  CUDA Driver Version / Runtime Version          12.2 / 11.8
  CUDA Capability Major/Minor version number:    7.5

CUDA Capability Major/Minor version number这个字段的数字就是CUDA_ARCH_BIN
###
```

4. 由于需要从docker Hub pull一个base镜像，需要提前解决docker的代理问题，否则会出错

参考：

```bash
vim /etc/docker/daemon.json
###
# 添加以下内容
{
 "registry-mirrors": ["https://docker.1ms.run", "https://docker.1panel.live/"]
}
###
sudo systemctl daemon-reload
sudo systemctl restart docker
```


1. 下载TensorRT到Docker文件夹下

下载地址（需要登录）：
https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/secure/8.6.1/tars/TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-11.8.tar.gz

2. 构建Docker镜像

```bash
cd Docker/Dockerfile
make pc
```

中间会提示

- ⚠️ Please download TensorRT and put it in the parent directory (../) url:https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/secure/8.6.1/tars/TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-11.8.tar.gz
Press any key to continue...
安装第一步下载好tensorRT文件到指定位置，此处可按任意键跳过

- Do you want to use a proxy? [y/n]
是否使用代理进行接下来的构建，由于github、国外软件源的存在，不使用代理会有几率存在网络问题，推荐使用代理，回车默认y

- Use default proxy (http://127.0.0.1:7897) (http://127.0.0.1:7897)?
默认代理地址指向本机代理软件的地址和端口，如果想要更改代理地址，输入n后进行修改。 ⚠️ 注意修改代理地址时，一般HTTP_PROXY和HTTPS_PROXY都是http开头的代理地址

构建完成后

```bash
docker image ls
```

输出

```bash
REPOSITORY      TAG IMAGE ID        CREATED         SIZE
fastdronexi35   pc  a615436b212d    19 hours ago    24.4GB
```

3. 实例化Docker容器

其中$TARGET_DIR需要替换为宿主机的代码目录，或者其他的想要映射到容器内的目录

- -v $TARGET_DIR:/root/Fast-Drone-XI35 作用是将宿主机的$TARGET_DIR目录映射到容器内的/root/Fast-Drone-XI35，实现宿主机和容器共同使用同一份文件。两个系统均可对文件进行修改。


```bash
xhost +local:root # 允许本地 root 显示 X 窗口

docker run -it \
--name fd_runtime_pc \
--gpus all \
--net=host \
--privileged \
-e DISPLAY=$DISPLAY \
-e QT_X11_NO_MITSHM=1 \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v $TARGET_DIR:/root/Fast-Drone-XI35 \
fastdronexi35:pc \
bash
```

4. 测试容器

- 进入容器终端

```bash
docker exec -it fd_runtime_pc bash
```

- 建议将代码source setup.bash放入.bashrc环境变量中,同时配置ROS_LOG_DIR的环境变量，根据自己实际的容器中代码路径下述命令

```bash
echo 'export ROS_LOG_DIR=/root/Fast-Drone-XI35/log' >> /root/.bashrc
echo "source /root/Fast-Drone-XI35/devel/setup.bash" >> /root/.bashrc
```

- 运行VINS-Fusion测试

```
roslaunch imu_filter imu_filter.launch
roslaunch superpoint superpoint_frontend.launch
roslaunch vins fast_drone_250.launch
roslaunch vins rviz.launch
rosbag play your_dataset.bag
```

⚠️ 注意第一次运行前端superpoint_frontend.launch会卡住，是正常现象，内部在进行跨平台的.onnx文件构建，等待3~4分钟即可正常运行，下一次也可正常启动。
