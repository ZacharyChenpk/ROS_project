Usage:

1. ensure that "export TURTLEBOT3_MODEL=burger" added to your .bashrc 
2. compile this package
	cd catkin_ws/src
	git clone XXXXX
	cd ..
	catkin_make
	source devel/setup.bash
2. run these command in different terminals
	roslaunch proj_api simple.launch
	roslaunch proj_api turtlebot3_dqn_stage_1.launch
	roslaunch proj_api result_graph.launch