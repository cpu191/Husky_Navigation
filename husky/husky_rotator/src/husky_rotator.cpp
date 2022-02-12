#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <chrono>

/**
 **  Node to rotate Husky robot CW 10s and then anti-CW 10s
 **/

int main(int argc, char* argv[])
{
  //init the ros
  ros::init(argc,argv,"husky_rotator_node");
	
  //Ros handle
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);
  geometry_msgs::Twist msg;
  
  //set the run duration 
  std::chrono::milliseconds duration = std::chrono::milliseconds(10000);

  while (ros::ok())
    {
      //assign the robot velocities for CW rotation with some linear movement 
      msg.linear.x = -0.5 ;// Due to the obstacle in the playpen map, the robot will run in revese for the sake of demonstration
      msg.linear.y = 0 ;
      msg.linear.z = 0 ;
      msg.angular.x = 0;
      msg.angular.y = 0;
      msg.angular.z = 0.3;

      //assign the the end
      std::chrono::time_point<std::chrono::system_clock> end;
      end = std::chrono::system_clock::now() + duration;
      
      // run til the end
      while(std::chrono::system_clock::now() < end) 
	{
	  pub.publish(msg);
	  ROS_INFO_STREAM("Velocity :"<<" linear="<<msg.linear.x<<" CW angular="<<msg.angular.z);
	}

      // rotating robot CCW, keeping other params same
      msg.angular.z = -0.3;

      //change the the end 
      end = std::chrono::system_clock::now() + duration;
      
      // run til the end
      while(std::chrono::system_clock::now() < end) 
	{
	  pub.publish(msg);
	  ROS_INFO_STREAM("Velocity :"<<" linear="<<msg.linear.x<<" CCW angular="<<msg.angular.z);
	}

      ros::spinOnce();
    }
}

