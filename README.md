# AGV-OTA (ROS Noetic)

This repository includes the AGV ROS Noetic packages."main" branch is latest version of AGV-OTA.

![Image of AGV](https://github.com/inomuh/agv/blob/main/images/agv_gazebo.png)

- agv_description: It is the sub-package containing urdf and mesh files of the AGV.
- agv_simulation: It is a sub-package containing the package and launch files required for the simulation of the AGV.
- agv_navigation: It is a sub-package containing the navigation launch and config files.
- (zozibush) agv_launch: It is a sub-package for launching multirobot
- (zozibush) agv_follow_lane : It's a sub-package for follow lane
- (zozibush) agv_follow_aruco : It's a sub-package for follow aruco
- (zozibush) agv_control : It's a sub-package for control agv to drive

## Launch Command

### Warning

Before using launch commands, you must unzip ~/agv/agv_description/meshes/OTAv07_meshes/OTA-v0.7.tar.xz file..

-------------------------------------------------------------------------------------------------------------
Gazebo Launching:

    roslaunch agv_launch main_multi.launch
ArUco Spawn(id: 0~7):

    roslaunch agv_launch spawn_aruco.launch marker_id:=0 x:=0 y:=0 z:=0 P:=0 R:=0 Y:=0
follow_lane:

    roslaunch agv_follow_lane follow_lane.launch
follow_aruco:

    roslaunch agv_follow_aruco follow_aruco.launch
driving agv(with publishing 'robot_name/control' topic):

    roslaunch agv_control agv_control.launch

-----------------------------------------------------------------------------------------------------------------------
### Requirements:
-------------
- In order for the "joint_state_publisher" to work, "joint_state_publisher_gui" package must be downloaded to your computer.

        $ sudo apt update && sudo apt install ros-noetic-joint-state-publisher-gui
        
- In order for the "joint_state_controller" to work, "joint_state_controller_gui" package must be downloaded to your computer.

        $ sudo apt install ros-noetic-ros-controllers
        
- In order for the SLAM to work, "slam_gmapping" package must be downloaded to your workspace.
        
        $ cd ~/catkin_ws/src && git clone https://github.com/ros-perception/slam_gmapping.git -b melodic-devel
        
- In order for the sensors to work properly, "gazebo_ros_pkgs" files must be downloaded to your computer.

        $ sudo apt-get install ros-noetic-gazebo-ros-pkgs
        
- In order for the navigation tools to work properly, "ros-navigation" files must be downloaded to your computer.

        $ sudo apt-get install ros-noetic-ros-navigation
        
- In order for the multirobot applications to work properly, "robot_state_publisher" package must be downloaded to your workspace.
        
        $ cd ~/catkin_ws/src && git clone https://github.com/rhaschke/robot_state_publisher -b noetic-devel
- submodule update
        
        $ git submodule update --init --recursive
