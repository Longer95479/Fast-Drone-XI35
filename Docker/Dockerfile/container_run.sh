# check if the container already exists
if docker ps -a --format '{{.Names}}' | grep -qw fd_runtime; then
    echo "Container 'fd_runtime' already exists. Exiting..."
    exit 0
fi

# create data directory if it doesn't exist
mkdir -p ${HOME}/Docker_Data

# run the container
docker run -itd --privileged=true --network host \
        --mount type=bind,source=${HOME}/Docker_Data,target=/root/data \
        --mount type=bind,source=/dev,target=/dev \
        --mount source=Fast-Drone-XI35,target=/root/Fast-Drone-XI35 \
        --runtime=nvidia --gpus all \
        --name fd_runtime \
        fastdronexi35:orin /bin/bash

# LOG_FILE=/root/Fast-Drone-XI35/log/catkin_build.log

# monitor the build log
docker exec -i fd_runtime bash <<EOF
LOG_FILE=/root/Fast-Drone-XI35/log/catkin_build.log

while [ ! -f "\$LOG_FILE" ]; do
    sleep 1
done

tail -n 0 -f "\$LOG_FILE" | while read line; do
    echo "\$line"
    if echo "\$line" | grep -q "catkin_make succeeded"; then
        echo -e '\033[1;32m Build succeeded !!! \033[0m'
        exit 0
    fi
    if echo "\$line" | grep -q "catkin_make failed"; then
        echo -e '\033[1;31m Build failed !!! \033[0m'
        exit 1
    fi
done
EOF
