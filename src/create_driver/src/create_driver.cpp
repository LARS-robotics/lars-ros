#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "create_driver/CreateDriver.h"
#include <string>
#include <unistd.h>

#define rho	133.334

CreateDriver *driver;
std::string hostname;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	double lwv = msg->linear.x - rho * msg->angular.z;
	double rwv = msg->linear.x + rho * msg->angular.z;
  	driver->directDrive(rwv, lwv);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "create_driver");

	ros::NodeHandle n;
	ros::NodeHandle private_n("~");
	private_n.param<std::string>("hostname", hostname, "192.168.2.20");
	
	ros::Rate r(100);
	
	driver = new CreateDriver(hostname.c_str());
	driver->doOpen();
	sleep(3);
	driver->doStart();
	driver->setRobotState(SAFE);
	
	ros::Subscriber sub = n.subscribe("cmd_vel", 10, Callback);
	
	while (ros::ok()) { 
		ros::spinOnce();
		r.sleep();
	}
		
	driver->doStop();
	driver->doClose();
	return 0;
}
