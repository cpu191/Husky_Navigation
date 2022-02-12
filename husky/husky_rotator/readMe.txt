by Tim Nguyen email:giathinh1907@gmail.com

Part 1 - Simulation robot control

1.Requirement:
	- Ubuntu 18.04
	- ROS Melodic
	- Husky robot ROS package (checkout https://github.com/husky/husky)

2.Description: 
	The node will broadcast messages to '/husky_velocity_controller/cmd_vel' topic to turn the robot Clockwise for 10 seconds and Counter Clockwise for 10 seconds (repetitive).The code is written in C++ and Python

3.Instruction:
	Copy the husky_rotator package into the catkin workspace source (usually at "~/catkin_ws/src/husky")  

	In terminal,navigate to your catkin workspace,and run the following command:
	 - catkin_make   #the command should run successfully
	 - roscore
	 - roslaunch husky_gazebo husky_playpen.launch #If you have a different path for the package, change the path accordingly

	*To run the C++ node that rotating the husky robot, in terminal:
	 - rosrun husky_rotator husky_rotator_node
	
	*To run the python node, navigate to the "~/catkin_ws/src/husky/husky_rotator/src/husky_rotator_py" or change the path according to your setup, in terminal:
	 - chmod +x husky_rotator_py.py 
	 - rosrun husky_rotator husky_rotator_py.py 



 


