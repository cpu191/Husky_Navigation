by Tim Nguyen email:giathinh1907@gmail.com

Part 2 - Autonomous Navigation

1.Requirement:
	- Ubuntu 18.04
	- ROS Melodic
	- Husky robot ROS package (checkout https://github.com/husky/husky)
	- ROS navigation packages (install gmapping and navigation package with : sudo apt-get install ros-melodic-openslam-gmapping ros-melodic-navigation)

2.Description: 
	The node will retrieve the navigation goal from the navigation_io.csv file in "~/catkin_ws/src/husky/husky_autonomous/src", then communicate with the move_base node to process the navigation, after the navigation process finished, the final pose of the robot will be written back in the navigation_io.csv file. The program will automatically terminate when the result is saved

3.Instruction:
	Copy the husky_autonomous package into the catkin workspace source (usually at "~/catkin_ws/src/husky")  

	In terminal,navigate to your catkin workspace,and run the following commands:
	 - catkin_make   #the command should run successfully
	 - roscore
	 - roslaunch husky_gazebo husky_playpen.launch #If you have a different path for the package, change the path accordingly
	 - roslaunch husky_navigation move_base_mapless_demo.launch 
	
	To change the navigation goal, open the navigation_io.csv file and change the first 6 element in the second line, according to the collum names (xGoal,yGoal,zGoal,rollGoal,pitchGoal,yawGoal), elements are separated by commas ",".	
	
	*To run the C++ node that navigating the husky robot, in terminal:
	 - rosrun husky_autonomous husky_autonomous_node
	
	*To run the python node, navigate to the "~/catkin_ws/src/husky/husky_autonomous/src/husky_autonomous_py" or change the path according to your setup, in terminal:
	 - chmod +x husky_navigator_py.py 
	 - rosrun husky_navigator husky_navigator_py.py 

