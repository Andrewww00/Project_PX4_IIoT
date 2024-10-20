# IIoT project
University of Messina A.Y. 2023/2024 | Industrial IoT

This project aims to control a simulated drone and make it react to external factors such as the presence of obstacles along its path.
First, the drone is sent on a mission with GPS coordinates. When it encounters an obstacle, it switches to offboard mode. After successfully avoiding the obstacle, the drone continues towards the mission point.

In this repo, you'll find a guide to setting up the environment required to reproduce the proposed solution.

To achieve the project's goal, the following tools are used: 
* VM with Ubuntu 22.04
* PX4-Autopilot
* QGroundControl
* ROS 2 Humble
* MAVROS

### Install PX4-Autopilot

To install the toolchain:
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
Restart the computer on completion.

### Install ROS2 Humble
To install ROS2 Humble: [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Install MAVROS
To install MAVROS run:
```
sudo apt-get install ros-humble-mavros ros-humble-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```

### Install Gazebo Classic
To install Gazebo Classic run this:
```
sudo apt remove gz-garden
sudo apt install aptitude
sudo aptitude install gazebo libgazebo11 libgazebo-dev
```

### Install QGroundControl
To install QGC:
First run this commands:
```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```
Then download the AppImage from [here](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)

Once downloaded, move the file to home directory and run:
```
chmod +x ./QGroundControl.AppImage
```

### Create Folder and clone repository:
Run the following command to create a workspace folder and a src folder.
```
mkdir -p ~/(folder_name)/src
cd ~/(folder_name)/src
```
Now, clone the repository inside the src:
```
git clone https://github.com/Andrewww00/Project_PX4_IIoT.git
```
Install colcon to build the workspace:
```
sudo apt install python3-colcon-common-extensions
sudo apt install python3-argcomplete
```
Before building the workspace, add this lines to your .bashrc file. By using this you avoid sourceing the setup.bash of both ros2 distro and workspace everytime.
```
source /opt/ros/humble/setup.bash
source ~/(folder_name)/install/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```
First, move to (folder_name) then build the workspace:
```
colcon build
```

Once done, you can use the setup_sys.sh file to setup the env:
```
cd
chmod +x ~/(folder_name)/src/Project_PX4_IIoT/project_pkg/setup_sys.sh
./(folder_name)/src/Project_PX4_IIoT/project_pkg/setup_sys.sh
```
Then run the ros2 node to start the mission:
```
ros2 run project_pkg mission
```

# Flask version
To run the flask version of the project, first navigate to the server folder and run the server script:
```
cd ~/(folder_name)/src/Project_PX4_IIoT/project_pkg/server
python3 flask_server.py
```
Then, in another terminal, run the flask version:
```
ros2 run project_pkg flask_mission
```


Credits:
* Longo Andrea
* Musmeci Edoardo
