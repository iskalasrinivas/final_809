# group6_rwa5

(a) Build Instructions

1. copy group6_rwa5.zip to "catkin_ws/src"
2. Run following command in the terminal:

 cd ~/catkin_ws/src
 unzip group6_rwa5.zip
 cd ..
 catkin_make --only-pkg-with-deps group6_rwa5


(b) Run Instructions

1. Open terminal
2. Type following commands in the terminal

 cd ~/catkin_ws
 source devel/setup.bash

3. Run following commands in new terminals:
 
 roslaunch group6_rwa5 group6_rwa5.launch
 roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1
 roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm2
 rosrun group6_rwa5 main_node

