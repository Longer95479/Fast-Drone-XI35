mkdir -p ${HOME}/Docker_Data
docker run -itd --privileged=true --network host \
        --mount type=bind,source=${HOME}/Docker_Data,target=/root/data \
        --mount type=bind,source=/dev,target=/dev \
        --mount source=Fast-Drone-XI35,target=/root/Fast-Drone-XI35 \
        --runtime=nvidia --gpus all \
        --name fd_runtime \
        fastdronexi35:orin /bin/bash
