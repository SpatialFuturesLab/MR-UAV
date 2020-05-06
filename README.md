# Spatial Futures Lab Bebop Autonomy


Software for the Bebop Autonomy and integration with Grasshopper.

## Setup Instructions

### Contents
1. [Install Prerequisites](#1-install-prerequisites)
2. [Clone repository](#2-clone-or-fork-repositories)
3. [Compilation/Running](#3-compilation-running)
4. [Communication](#3-communication)

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

6. __Clone SFL_Bebop workspace and MRAC_IAAC workspace__
	The MRAC_IAAC workspace is added in the mrac_iaac folder.

	```git clone --recursive https://github.com/SpatialFuturesLab/MR-UAV.git```


### 3. Compilation & Running
1. __Update rosdep and install dependencies__

	```sudo rosdep update```
	
	```sudo rosdep install --rosdistro kinetic --from-paths src -i```

2. __Build the workspace__

	```catkin build```
3. __Update bashrc__

	```alias bebop = source ~/devel/setup.sh```

	Add the above line of code with the path of the setup.sh in bashrc. Everytime a new terminal is opened, type bebop.
4. __Launch Drone__

	```bebop```

	```roslaunch bebop_driver bebop_node.launch```

### 4. Communication
 * https://wiki.teltonika-networks.com/view/Setting_up_a_Static_IP_address_on_a_Ubuntu_16.04_PC
 * https://www.dummies.com/programming/networking/configuring-network-connections-windows-10/
 
## Operating Procedures

### Contents
1. [Capture Points from Rigid Bodies](#1-capture-points)
2. [Autonomous Flight](#2-autonomous-flight)
3. [Manual Flight](#3-manual-flight)
4. [Marker Snapshot](#3-marker-snapshot)

### 1. Capture Points from Rigid Bodies
1. __Define Rigid Body (Windows)__
   * Create a rigid body in Motive using a minimum of 3 markers
   * Label it using the naming convention GeomToolX where X is an upper case letter (A-Z)
   * To capture points from the same rigid body, create a rigid body in Motive. Use the naming convention SnapToolX where X is a number (0-9) to define different pivot points on the rigid body. 
   
2. __Run Python Script (Ubuntu)__
   * Run Python script(Ubuntu) capture_rigid_body.py
   * With the terminal active, press a key X on the keyboard (A - Z) to capture the rigid body GeomToolX corresponding to the pressed key
   * The position and orientation of this rigid body can be obtained by subscribing to the rostopic /rigid_body/GeomToolX
   * Marker Points from a single rigid body could be captured by pressing keys (0-9) on the keyboard. Press spacebar on the keyboard to send all the captured Marker points to Grasshopper 
   
   ```bebop```
   
   ```rostopic echo /rigid_body/GeomToolX```
   ```rostopic echo /rigid_body/SnapToolX```
   
### 2. Autonomous Flight
1. __Launch Drone__

	```bebop```
	
	```roslaunch bebop_tools joy_teleop.launch```
2. __Run Python Script__

	```bebop```
	
	```python bebop_position_controller.py``` 

3. __Toggle Autonomous Mode__
	* Press Buttons RB + LB on Joystick to toggle to/back from autonomous mode 
	* In autonomous mode, drone can navigate to the positions given to it
	
4. __Define Waypoints__ 
	* Publish to rostopic bebop/setpoints to add waypoints 
	* If drone does not receive any new waypoints, it hovers at previously defined waypoint
	
	
### 3. Manual Flight
1. __Launch Drone__

	```bebop```
	
	```roslaunch bebop_tools joy_teleop.launch```

2. __Joystick Control__

	```bebop```
	
	```roslaunch bebop_tools joy_teleop.launch```
	
3. __Update Joystick Sensitivity__

	* Update script in bebop_autonomy/bebop_tools/config/xbox360.yaml to change sensitivity of drone commands from joystick
	* Change scale in lines 13, 18, 23, 28 to 0.5
