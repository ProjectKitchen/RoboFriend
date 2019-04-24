#!/usr/bin/env bash

kill_nodes() {
    echo ""
    echo "Kill Face and Ojectdetection Node"
    trap - SIGINT SIGTERM
}


if [ -z "$1" ]
then
    echo "[EROR] No IP address entered!"
    exit 0
else
    ip_address=$1
fi

trap kill_nodes SIGINT SIGTERM


ROS_MASTER_URI="ROS_MASTER_URI=http://$ip_address:11311"

export $ROS_MASTER_URI

roscore &
python3 FaceDetectionNode/FaceDetectionNode.py -ip $ip_address &
python3 ObjectDetectionNode/ObjectDetectionNode.py -ip $ip_address &

sleep infinity
