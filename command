- let's copy the folder models of the aruco_ros package in .root/gazebo/models

roslaunch erl_assignment3 simulation.launch
roslaunch erl_assignment3 gazebo4.launch
roslaunch planning gmapping.launch
roslaunch planning move_base.launch

catkin_make --only-pkg-with-deps erl2 my_rosplan_interface

catkin_make -DCATKIN_WHITELIST_PACKAGES="" 

A6 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

requirement

planning pkg
motion plan
aruco
aruco_msg
