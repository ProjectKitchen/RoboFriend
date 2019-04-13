#!/usr/bin/env bash

kill_nodes() {
    echo ""
    echo "Kill Face and Ojectdetection Node"
    trap - SIGINT SIGTERM
}

trap kill_nodes SIGINT SIGTERM

ip_address=$1
ROS_MASTER_URI="ROS_MASTER_URI=http://$ip_address:11311"

export $ROS_MASTER_URI

roscore &
python3 FaceDetectionNode/FaceDetectionNode.py -ip $ip_address &
python3 ObjectDetectionNode/ObjectDetectionNode.py -ip $ip_address &

sleep infinity
