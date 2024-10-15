# pcd_pub_ros

[![ROS Version](https://img.shields.io/badge/ROS-Noetic-green)](https://wiki.ros.org/noetic)

[![TDG-Attribution-NonCommercial-NoDistrib](https://img.shields.io/badge/license-TDG_Attribution_NC_NoDistrib-red)](LICENSE)

[![GitHub release](https://img.shields.io/github/release/zhz03/pcd_pub_ros.svg)](https://github.com/https://img.shields.io/github/release/zhz03/pcd_pub_ros/releases)

[![Installation Instructions](https://img.shields.io/badge/Installation-ROS-blue)](https://docs.ros.org/en/)[![Tutorials](https://img.shields.io/badge/Tutorials-ROS-orange)](https://docs.ros.org/en/)[![FAQs](https://img.shields.io/badge/FAQs-ROS-lightgrey)](https://docs.ros.org/en/)

## Overview
- Visualizes point clouds and markers in RViz
- Streams raw points from PCD files to ROS topics for visualization
![](pcd_viz_result.png)

## Installation
1. Prerequisites: ROS Noetic, C++ 14, Catkin
2. Clone and build in your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/zhz03/pcd_pub_ros
```

Then navigate to the workspace and build the package:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## How to Use

This ROS package is intended to be run using a pcd data file. Using a file explorer, navigate to the pcd file(s) you want to visualize. 

Navigate to the launch folder within the pcd_pub_ros package. The launch folder's file path will be of the following form. 
```bash
~/catkin_ws/src/pcd_pub_ros/launch
```
For any argument (<arg>) with name "pcd_file1" or "pcd_file2", replace the default filepath (default=) with the filepath of the pcd data file(s) you want to visualize. 

Open a terminal window, then change the current working directory to the ROS workspace containing the pcd_pub_ros directory. 

If you have not done so within this terminal session, run the following command to enable ROS commmand use: 
```bash
source devel/setup.bash
```

Run a launch file from the pcd_pub_ros package using the ROS launch command template below, replacing LAUNCH_FILE with the .launch file associated with the desired launch configuration: 
```bash
roslaunch pcd_pub_ros <launch_file>
```

If launch is successful, you will get output displaying information on your pcd data, which we will use later. 

Open another terminal window, and open RViz using: 
```bash 
rviz 
``` 

- Add a new display by clicking the "Add" button in the bottom left corner and selecting the "PointCloud2" option from the window prompt. 

- Click the arrow to the left of the new "PointCloud2" display to view more information on the display. 

- Change the "Topic" field within RViz's PointCloud2 menu to the topic name of the pcd file. This topic name should be an option in the selector's dropdown list--enter manually if needed. 

- Change the "Fixed Frame" field within RViz's global options to match that of the frame ID. 

- Note that if the frame ID begins with a slash (/), you will likely need to begin with two slashes, since one will be automatically removed. 

- Click the "Reset" button in the bottom left corner of the RViz window to play the pcd data from its start.
