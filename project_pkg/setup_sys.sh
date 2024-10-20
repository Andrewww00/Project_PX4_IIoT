#!/bin/bash

gnome-terminal --title="PX4 SITL" -- bash -c "cd ~/PX4-Autopilot && make px4_sitl_default gazebo-classic_iris__baylands; exec bash"

gnome-terminal --title="QGC" -- bash -c "./QGroundControl.AppImage; exec bash"

gnome-terminal --title="MAVROS" -- bash -c "ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14580"; exec bash"

