/**
 **  Auto navigation node for Husky,goal(x,y,z,R,P,Y) can be set with navigation_io.csv file 
 **/
#include <sstream>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <utility>
#include <stdexcept> 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib_msgs/GoalStatus.h>
bool navFinished=0;    // Switch for navigation process

// Callback function to check if the navigation process has finished
void navCallback(const move_base_msgs::MoveBaseActionResult& msg)
{
  actionlib_msgs::GoalStatus nav_stat = msg.status;
  if (nav_stat.status == 3)
    {
      ROS_INFO("Goal Reached ");
      navFinished = 1 ;
    }  
}

//Function to convert Quaternion to Roll Pitch Yaw
geometry_msgs::Vector3 quat2rpy(const geometry_msgs::Quaternion msg)
{
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf2::Quaternion quat;
    tf2::fromMsg(msg, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;    
    return rpy;
}

//Write the navigation result to CSV file
void write_odom_csv(std::string fileName,geometry_msgs::PoseStamped goalPose,nav_msgs::Odometry finalPose){

  //Convert the quaternions to RPYs
  geometry_msgs::Vector3 goalRPY = quat2rpy(goalPose.pose.orientation);
  geometry_msgs::Vector3 finalRPY = quat2rpy(finalPose.pose.pose.orientation);

  //Print the data into the file
  std::ofstream fname;
  fname.open (fileName);
  fname << "xGoal,yGoal,zGoal,rollGoal,pitchGoal,yawGoal,xFinal,yFinal,zFinal,rollFinal,pitchFinal,yawFinal\n";    //Collums name

  fname << goalPose.pose.position.x << "," << goalPose.pose.position.y << "," << goalPose.pose.position.z << ",";
  fname << goalRPY.x << "," << goalRPY.y << "," << goalRPY.z << ",";
  fname << finalPose.pose.pose.position.x << "," << finalPose.pose.pose.position.y << "," << finalPose.pose.pose.position.z << ",";
  fname << finalRPY.x << "," << finalRPY.y << "," << finalRPY.z;
  fname.close();
}

//Function to read CSV file
std::vector<std::vector<std::string>> read_csv(std::string fname){

  std::vector<std::vector<std::string>> content;
  std::vector<std::string> row;
  std::string line, word;

  std::fstream file (fname,std::ios::in);
  if(file.is_open())
    {
      while(getline(file, line))
	{
	  row.clear();
	  std::stringstream str(line);
	  while(getline(str, word, ','))
	    row.push_back(word);
	  content.push_back(row);
	}
    }
  else
    ROS_INFO("Could not open the file\n");

  return content;  
}

int main(int argc, char* argv[])
{
  //Init the ros
  ros::init(argc,argv,"husky_auto_nav_node");
	
  //Ros handle
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  ros::Subscriber sub = nh.subscribe("/move_base/result",10,navCallback);
  //Create goal pose
  geometry_msgs::PoseStamped goalPose;

  //Read pose from CSV
  std::vector<std::vector<std::string>> data = read_csv("navigation_io.csv");

  //Set Position
  goalPose.pose.position.x = std::stod(data[1][0]);
  goalPose.pose.position.y = std::stod(data[1][1]);
  goalPose.pose.position.z = std::stod(data[1][2]);

  //Record RPY
  double roll= std::stod(data[1][3]);
  double pitch= std::stod(data[1][4]);
  double yaw= std::stod(data[1][5]);
  
  //Convert Roll,Pitch,Yaw to Quaternion
  tf2::Quaternion quat_tf;
  quat_tf.setRPY(roll,pitch,yaw);
  quat_tf.normalize();
  geometry_msgs::Quaternion quat_msg;
  quat_msg = tf2::toMsg(quat_tf);
  
  //Set Orientation
  goalPose.pose.orientation = quat_msg;
  
  //set frame id
  goalPose.header.frame_id ="odom";
  
  while (ros::ok())
    {
      while (pub.getNumSubscribers() < 1) {}    // wait for a connection to publisher
      pub.publish(goalPose);    //publish the goal pose
      ROS_INFO_STREAM("publishing Pose: "<<goalPose.pose.position.x<<" , "<<goalPose.pose.position.y<<" , "<<goalPose.pose.position.z);
      ROS_INFO_STREAM("Orientation: "<<roll<<" , "<<pitch<<" , "<<yaw<<" \n Navigating...  ");

      while(!navFinished)    //wait for navigation to complete, run all the callbacks while waiting
       	{
	  ros::spinOnce();
      	}
      
      ROS_INFO("Navigation Finished - Writing result");
      
      //Get one msg from odom to update robot position
      nav_msgs::Odometry::ConstPtr shared_finalPose = ros::topic::waitForMessage<nav_msgs::Odometry>("/odometry/filtered",nh);
      
      if (shared_finalPose != NULL)
	{
	  nav_msgs::Odometry finalPose= *shared_finalPose;   //pointer to the odom msg
	  write_odom_csv("navigation_io.csv",goalPose,finalPose); // write the navigation result
	  ROS_INFO("Navigation result saved - Exiting. ");
	}
      
      break;
    }
  
  
}
