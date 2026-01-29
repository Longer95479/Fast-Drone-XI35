SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
TARGET_DIR=$(realpath "${SCRIPT_DIR}/../../../Fast-Drone-XI35")
echo "$TARGET_DIR"
# check if the container already exists
if docker ps -a --format '{{.Names}}' | grep -qw fd_runtime_pc; then
    echo "Container 'fd_runtime_pc' already exists. Exiting..."
    exit 0
fi

# run the container
docker run -it --name fd_runtime_pc --gpus all --net=host --privileged -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $TARGET_DIR:/root/Fast-Drone-XI35 fastdronexi35:pc bash

