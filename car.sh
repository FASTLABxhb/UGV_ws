modprobe gs_usb
ip link set can0 up type can bitrate 500000

source /home/fastipc/czk_ws/bunker/devel/setup.bash
roslaunch bunker_bringup bunker_robot_base.launch
