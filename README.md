
# IIoT project 
This project aims to control a simulated drone and make it react to external factors such as the presence of obstacles along its path.
First, the drone is sent on a mission with GPS coordinates. When it encounters an obstacle, it switches to obstacle avoidance mode. After successfully avoiding the obstacle, the drone continues towards the mission point.

## Setup

### Install PX4-Autopilot

To install the toolchain:
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```
Run the ubuntu.sh with no arguments (in a bash shell) to install everything
```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
Restart the computer on completion.

### Install ROS2 Humble
To install ROS2 Humble: [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Install Dependencies
Install Python dependencies with this code
```
pip3 install --user -U empy pyros-genmsg setuptools
```
And:
```
pip3 install kconfiglib
pip install --user jsonschema
pip install --user jinja2
```

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

### Create Folder and clone repository:
Run the following command to create a workspace folder and a src folder.
```
mkdir -p ~/(folder_name)/src
cd ~/(folder_name)/src
```
Now, clone the repository inside the src:
```
git clone "https://github.com/Andrewww00/ros2_ws.git"
```
Once done, run the .sh file for setup the env:
```
./setup_sys.sh
```
Then run the ros2 node to start the mission:
```
ros2 run project_pkg mission
```
