# ros_network_gap

## Overview

Node to send a single ROS topic across a network using UDP.

## Usage

To test, run
`roslaunch ros_network_gap ros_network_gap.launch`
and 
`rosrun roscpp_tutorials talker`
on a broadcaster machine A. Adjust the adress of the listener machine B in `ros_network_gap/config/example.yaml` and run 
`roslaunch ros_network_gap ros_network_gap.launch`
on B as well to receive the output of the talker running on A.

If you have net-tools installed, run
`rosrun ros_network_gap check_ports.sh`
on B to get a list of open ports.