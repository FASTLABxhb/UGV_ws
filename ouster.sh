#!/bin/bash
source /home/fastipc/czk_ws/UGV_ws/devel/setup.bash

roslaunch ouster_ros ouster.launch sensor_hostname:=169.254.119.238 metadata:='/home/fastipc/czk_ws/UGV_ws/src/ouster_example/tests/os-122114000532-32U0.json'

