# pcd_pub_ros
This package is to vizualize a pcd file using PointCloud2. 

[![ROS Version](https://img.shields.io/badge/ROS-Noetic-green)](https://wiki.ros.org/noetic)

[![TDG-Attribution-NonCommercial-NoDistrib](https://img.shields.io/badge/license-TDG_Attribution_NC_NoDistrib-red)](LICENSE)

[![GitHub release](https://img.shields.io/github/release/zhz03/pcd_pub_ros.svg)](https://github.com/https://img.shields.io/github/release/zhz03/pcd_pub_ros/releases)

[![Installation Instructions](https://img.shields.io/badge/Installation-ROS-blue)](https://docs.ros.org/en/)[![Tutorials](https://img.shields.io/badge/Tutorials-ROS-orange)](https://docs.ros.org/en/)[![FAQs](https://img.shields.io/badge/FAQs-ROS-lightgrey)](https://docs.ros.org/en/)

## Overview

This ROS package provides functionality for vizualization of up to 2 pcd files with point cloud. 

![](pcd_viz_result.png)

## Features
- Allows the user to visualize point clouds and markers from collected data on RViz
- Streams raw points and data in pcd files to ROS topic for visualization

## Installation
### Prerequisites
- ROS (Robot Operating System) Noetic installed on Ubuntu 20.0.4
- C++ 14
- Catkin

```bash
sudo apt install catkin
```

### Installation Steps
If your catkin workspace is not set up, refer to [the ROS workspace setup tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) to set up the catkin workspace first. If it is already setup, proceed to next step.

To build from source, clone the repository into your ROS workspace and compile:

```bash
cd ~/catkin_ws/src
git clone https://github.com/zhz03/pcd_pub_ros
```

Then navigate to the workspace and build the package:
```bash
cd ~/catkin_ws
catkin_make
```

Source the ROS setup script:
```bash
source devel/setup.bash
```

## How to Use

This ROS package is intended to be run using a pcd data file. Using a file explorer, navigate to the pcd file(s) you want to visualize. 

In a separate file explorer window, navigate to the launch folder within the pcd_pub_ros package. The launch folder's file path will be of the following form. 
```bash
~/catkin_ws/src/pcd_pub_ros/launch
``` 

Open the launch file that you plan to use in a text editor. The cloned package includes three pre-configured launch files. 

Within the text editor, for any argument (<arg>) with name "pcd_file1" or "pcd_file2", replace the default filepath (default=) with the filepath of the pcd data file(s) you want to visualize. 

Save the launch file and close the text editor. 

Open a terminal window, then change the current working directory to the ROS workspace containing the pcd_pub_ros directory. 

If you have not done so within this terminal session, run the following command to enable ROS commmand use: 
```bash
source devel/setup.bash
```

Run a launch file from the pcd_pub_ros package using the ROS launch command template below, replacing LAUNCH_FILE with the .launch file associated with the desired launch configuration: 
```bash
roslaunch pcd_pub_ros [LAUNCH_FILE]
```

Note that the previously mentioned ROS launch command should be run from the top ROS workspace directory. 

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

- You will see the pcd file PointCloud2 data running in RViz. 

- The pcd file can be run as many times as needed while using the same RViz setup. 

### Example Usage 

Suppose you wanted to visualize a pcd file with the following directory via PointCloud2: 
```bash
~/Downloads/1678758798032182.pcd
```

The directory of the ROS workspace containing the pcd_pub_ros package is: 
```bash
~/pcd_ros_ws
```

Open a terminal window, then run the following to enable ROS commands in the ROS workspace: 
```bash
cd ~/pcd_ros_ws 
source devel/setup.bash
```

Suppose the configuration you want is implemented by the launch file "launch_1pcd.launch". Then, we would run the following: 
```bash
roslaunch pcd_pub_ros launch_1pcd.launch
```

In the case of this example pcd file, the associated rostopic name is "/hd_map", with frame id "/map". 
![](pcd_file_info.png)

Now, you can open a second terminal and run the following to open RViz: 
```bash
rviz
```

- Add a display by clicking the "Add" button in the bottom left corner of the RViz window, then selecting "PointCloud2". 

- Enter our PointCloud2 topic "/hd_map" in the "Topic" field that appears within the PointCloud2 display. 

- Under the Global Options "Fixed Frame" field, enter the name of our frame ID, "/map". 
- Note that in this example, you may need to type "//map", which will correct to "/map". 

- We can now hit the reset button in our RViz window.  The visualization should look similar to the following: 

![](pcd_viz_result.png) 

### launch_2pcd.launch 

The launch configuration "launch_2pcd.launch" can be used to visualize two pcd files simultaneously using two different frame IDs. 

- Before using "launch_2pc.launch", open the file "launch_2pc.launch" using a text editor.  
- Enter the complete path name of the two pcd files you want to visualize as the default values for the arguments (<arg>) with name "pcd_file1" and "pcd_file2". 

Your edited launch file should look similar to the following example, whose input pcd files are "1678758798032182.pcd" and "1678758798332120.pcd", with the following file path names: 
```bash 
/media/roszzl/data/1678758798032182.pcd
/media/roszzl/data/1678758798332120.pcd
```

![](pcd_launch.png)

Be sure to save the launch file, then open a terminal window and navigate to the pcd workspace. 

Run the following series of commands to build and setup the updated ROS package. 
```bash 
catkin_make 
source devel/setup.bash
```

Now, run the launch file with the following command: 
```bash
roslaunch pcd_pub_ros launch_2pcd.launch
```

Open another terminal window to launch RViz. 
```bash 
rviz
``` 

- Add a new display by clicking the "Add" button in the bottom left corner and selecting the "PointCloud2" option from the window prompt. 

- Change the PointCloud's "Topic" field to "/velodyne_1"--this is the default name of the topic associated with the first pcd file published by the launch configuration. 

- Now, add a second new PointCloud2 display. Change the associated "Topic" to "/ouster_points_1", the default name of the topic associated with the second pcd file. 

- Note: Each of these options should be available in the dropdown menu for the "Topic" field. 

- Note that if you change the default topic in the launch file, you will have different topics for visualization. 

- Now, you can visualize each pcd file, by changing the "Fixed Frame" field within RViz's global options. 

- To view the first pcd file, change the fixed frame to "/velodyne1", the default name of the frame ID for the pcd file 1; change the fixed frame to "/os_sensor1" to view the second pcd file. 

- As before, use the "Reset" button in the bottom left corner of the RViz window to play pcd data from its start.

- The resultant visualization should look similar to the following: 

![](pcd_viz_result_2.png) 
