#!/bin/bash

ping -q -c1 192.168.2.10 > /dev/null
ping_ret=$?

if [ -a /dev/rplidar ]; then
  echo "Launching Rplidar node..."
  roslaunch pandora_slam_2d pandora_rplidar.launch
elif [ -a /dev/hokuyo_04 ]; then
  echo "Launching hokuyo_node for URG-04LX"
  roslaunch pandora_slam_2d pandora_hokuyo_04lx.launch
elif [[ $ping_ret -eq 0 ]]; then
  echo "Launching urg_node for UST-20LX"
  roslaunch pandora_slam_2d pandora_hokuyo_20lx.launch
else
  echo -e "\n\e[2m\e[1m\e[31m[Fatal Error] Failed to detect laser device!!!\e[0m\n"
fi

