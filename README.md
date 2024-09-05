# learning_ros
This repository accompanies the text "A Systematic Approach to Learning Robot Programming with ROS".
This version has been updated for ROS Noetic. Accompanying videos can be found at:
https://www.youtube.com/playlist?app=desktop&list=PLy-BfVK12-hyJFHbFhEr4PTEqF_HO5mi0

Code examples reside in folders corresponding to chapters.

This entire repository should be cloned to: ~/ros_ws/src (assuming your ros workspace is named "ros_ws" and resides within your home directory).
To do so, navigate to ~/ros_ws/src from a terminal and enter:
```
`git clone https://github.com/rojas70/learning_ros_noetic.git`
```
and also clone the external packages used with:
```
`git clone https://github.com/rojas70/learning_ros_external_pkgs_noetic.git`
```

Then, from a terminal, navigate to ~/ros_ws and compile the code with the command:
`catkin_make`

If you are installing ROS for the first time, see the instructions here:
[installation scripts](//github.com/rojas70/learning_ros_setup_scripts)

The scripts located at this site automate installation of ROS (consistent with the version and packages used with the learning-ROS code examples).  These scripts also install a variety of useful tools.


