#!/bin/bash
# doc for shfiles/motion_capture_receive_publish.sh at Fast-Drone-XI35/src/realflight_modules/motion_capture/ReadMe.md
# modify the IP address to your motion capture system's broadcast IP !!!
motion_capture_broadcast_ip=192.168.31.45

roslaunch vrpn_client_ros sample.launch server:=${motion_capture_broadcast_ip} & VRPN_PID=$!

sleep 5

roslaunch motion_capture motion_capture.launch & MC_PID=$!

cleanup() {
    echo "Ctrl+C detected. Shutting down only the nodes started by this script..."

    if ps -p $VRPN_PID > /dev/null; then
        echo "Killing VRPN client (PID $VRPN_PID)"
        kill $VRPN_PID
    fi

    if ps -p $MC_PID > /dev/null; then
        echo "Killing motion_capture_node (PID $MC_PID)"
        kill $MC_PID
    fi

    exit 0
}


trap cleanup SIGINT

wait