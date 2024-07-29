base_image_name="local/fastdronexi35:orin_base_35.3.1"
fd_image_name="fastdronexi35:orin"
container_id=$(docker ps -aq --filter "name=fd_runtime")
volume_name="Fast-Drone-XI35"

#check base image
base_image_exists=$(docker images -q "$base_image_name")
if [ -n "$base_image_exists" ]; then
    echo "Base image $base_image_name exists, Operation continues.."
    read -p "The following action will remove the image $fd_image_name and the volume $volume_name,  Do you want to contiune? (y/n): " user_input
    if [ "$user_input" = "y" ] || [ "$user_input" = "Y" ]; then
        #check fd_runtime and remove it
        if [ -n "$container_id" ]; then
        echo "Stopping and removing container fd_runtime..."
        docker stop "$container_id"
        docker rm "$container_id"
        echo "Container fd_runtime has been stopped and removed."
        fi
        #rm fd image
        docker rmi $fd_image_name
        #rm the fd volumes
        docker volume rm $volume_name
        #rebuild the image
        make jetson
    else
        echo "Operation terminated!."
        exit 1
    fi

else
    echo "Base image $base_image_name does not exist. Terminating script."
    exit 1
fi
