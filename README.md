# Spatial Futures Lab Bebop Autonomy


Software for the Bebop Autonomy and integration with Grasshopper.

## Setup Instructions

### Contents
1. [Install Prerequisites](#1-install-prerequisites)
2. [Clone repository](#2-clone-or-fork-repositories)
3. [Compilation/Running](#3-compilation-running)

### 1. Install Prerequisites
1. __Install [Ubuntu 16.04 64-bit](http://www.ubuntu.com)__
   _Tutorials_

   The following tutorials could be followed to install Ubuntu from USB.
   
   * https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-windows#0
   * https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop#0
   
2. __Install(http://wiki.ros.org/kinetic/Installation/Ubuntu) ros-kinetic-desktop-full__
3. __Install Git__

	```sudo apt-get install git```
4. __Install Required Packages__

	```sudo apt-get install build-essential python-rosdep python-catkin-tools python-pip``` 
	
	```sudo apt-get install ros-kinetic-vrpn-client-ros```
	
	```sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras```
	```wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh```
	```chmod +x install_geographiclib_datasets.sh```
	```sudo ./install_geographiclib_datasets.sh```
	
	```sudo apt-get install ros-kinetic-rosbridge-server```


### 2. Clone or Fork Repositories
1. __Create catkin_workspace__

	```mkdir -p ~/bebop_ws/src && cd~/bebop_ws``` 

2. __Initialize workspace__

	```catkin init```

3. __Clone bebop_autonomy__ 

	```git clone https://github.com/AutonomyLab/bebop_autonomy.git src/bebop_autonomy```

4. __Clone drone_mocap__

	```git clone https://github.com/OsloMet1811/Digital-Twin.git src/drone_mocap```

5. __Clone vrpn_client_ros_kinetic__

	```git clone https://github.com/ros-drivers/vrpn_client_ros.git src/vrpn_client_ros```

6. __Clone SFL_Bebop workspace__

	```git clone https://github.com/SpatialFuturesLab/MR-UAV.git```

7. __Clone GTSAM into bebop_ws and build__

	```git clone https://bitbucket.org/gtborg/gtsam.git```
	
	```sudo pip install Cython backports_abc numpy```
	
	```cd gtsam```
	
	```mkdir build && cd build```
	
	```cmake -DGTSAM_INSTALL_CYTHON_TOOLBOX=ON -DGTSAM_PYTHON_VERSION=2.7 ..```
 


### 3. Compilation & Running
1. __Update rosdep and install dependencies__

	```sudo rosdep update```
	
	```sudo rosdep install --rosdistro kinetic --from-paths src -i```

2. __Build the workspace__

	```catkin build```
