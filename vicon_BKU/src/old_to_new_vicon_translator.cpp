#include <ros/ros.h>




#include <math.h>

#include "ros/console.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TransformStamped.h"


#define PI	3.1415926535
#define rho	133.334
#define alphaOverBar 900
#define alphaUnderBar 0



double k = 0.5; // linear velocity gain
double ktheta = 5.0; // angular velocity gain
bool power = true;
//double xd = 0; // x desired
//double yd = 0; // y desired

std::string robot_name;

std::string robot1_name;
std::string robot2_name;
std::string robot3_name;
std::string robot4_name;
std::string robot5_name;
std::string robot6_name;

int id = 0;

double x[6] = {0.0}; 
double y[6] = {0.0};
double theta[6] = {0.0};

double trMx[6] = {0.0};
double trMy[6] = {0.0};
double u_ca_x[6] = {0.0};
double u_ca_y[6] = {0.0};
double u_fc_x[6] = {0.0};

double u_fc_y[6] = {0.0};
double detM = 0.0;
geometry_msgs::TransformStamped pumsg[6];


void Callbackzero(const nav_msgs::Odometry::ConstPtr& msg)
{

   geometry_msgs::TransformStamped pmsg;
   pmsg.transform.translation.x = msg->pose.pose.position.x*0.001;
   pmsg.transform.translation.y = msg->pose.pose.position.y*0.001;
   pmsg.transform.translation.z = msg->pose.pose.position.z*0.001;

   pmsg.transform.rotation.x = msg->pose.pose.orientation.x;
   pmsg.transform.rotation.y = msg->pose.pose.orientation.y;
   pmsg.transform.rotation.z = msg->pose.pose.orientation.z;
   pmsg.transform.rotation.w = msg->pose.pose.orientation.w;
   pumsg[0] = pmsg;

}

void Callbackone(const nav_msgs::Odometry::ConstPtr& msg)
{

   geometry_msgs::TransformStamped pmsg;
   pmsg.transform.translation.x = msg->pose.pose.position.x*0.001;
   pmsg.transform.translation.y = msg->pose.pose.position.y*0.001;
   pmsg.transform.translation.z = msg->pose.pose.position.z*0.001;

   pmsg.transform.rotation.x = msg->pose.pose.orientation.x;
   pmsg.transform.rotation.y = msg->pose.pose.orientation.y;
   pmsg.transform.rotation.z = msg->pose.pose.orientation.z;
   pmsg.transform.rotation.w = msg->pose.pose.orientation.w;
   pumsg[1] = pmsg;
}

void Callbacktwo(const nav_msgs::Odometry::ConstPtr& msg)
{

   geometry_msgs::TransformStamped pmsg;
   pmsg.transform.translation.x = msg->pose.pose.position.x*0.001;
   pmsg.transform.translation.y = msg->pose.pose.position.y*0.001;
   pmsg.transform.translation.z = msg->pose.pose.position.z*0.001;

   pmsg.transform.rotation.x = msg->pose.pose.orientation.x;
   pmsg.transform.rotation.y = msg->pose.pose.orientation.y;
   pmsg.transform.rotation.z = msg->pose.pose.orientation.z;
   pmsg.transform.rotation.w = msg->pose.pose.orientation.w;
   pumsg[2] = pmsg;
}

void Callbackthree(const nav_msgs::Odometry::ConstPtr& msg)
{

   geometry_msgs::TransformStamped pmsg;
   pmsg.transform.translation.x = msg->pose.pose.position.x*0.001;
   pmsg.transform.translation.y = msg->pose.pose.position.y*0.001;
   pmsg.transform.translation.z = msg->pose.pose.position.z*0.001;

   pmsg.transform.rotation.x = msg->pose.pose.orientation.x;
   pmsg.transform.rotation.y = msg->pose.pose.orientation.y;
   pmsg.transform.rotation.z = msg->pose.pose.orientation.z;
   pmsg.transform.rotation.w = msg->pose.pose.orientation.w;
   pumsg[3] = pmsg;
}

void Callbackfour(const nav_msgs::Odometry::ConstPtr& msg)
{

  geometry_msgs::TransformStamped pmsg;
  pmsg.transform.translation.x = msg->pose.pose.position.x*0.001;
  pmsg.transform.translation.y = msg->pose.pose.position.y*0.001;
  pmsg.transform.translation.z = msg->pose.pose.position.z*0.001;

  pmsg.transform.rotation.x = msg->pose.pose.orientation.x;
  pmsg.transform.rotation.y = msg->pose.pose.orientation.y;
  pmsg.transform.rotation.z = msg->pose.pose.orientation.z;
  pmsg.transform.rotation.w = msg->pose.pose.orientation.w;
  pumsg[4] = pmsg;
}

void Callbackfive(const nav_msgs::Odometry::ConstPtr& msg)
{

  geometry_msgs::TransformStamped pmsg;
  pmsg.transform.translation.x = msg->pose.pose.position.x*0.001;
  pmsg.transform.translation.y = msg->pose.pose.position.y*0.001;
  pmsg.transform.translation.z = msg->pose.pose.position.z*0.001;

  pmsg.transform.rotation.x = msg->pose.pose.orientation.x;
  pmsg.transform.rotation.y = msg->pose.pose.orientation.y;
  pmsg.transform.rotation.z = msg->pose.pose.orientation.z;
  pmsg.transform.rotation.w = msg->pose.pose.orientation.w;
  pumsg[5] = pmsg;
}
 


int main(int argc, char **argv)
{
	ros::init(argc, argv, "translate");


	ros::NodeHandle n;
	ros::NodeHandle private_n("~");
	
	private_n.param<std::string>("robotname", robot_name, "gandhi");
	
	private_n.param<std::string>("robot1name", robot1_name, "gandhi");
	private_n.param<std::string>("robot2name", robot2_name, "suleyman");
	private_n.param<std::string>("robot3name", robot3_name, "alexander");
	private_n.param<std::string>("robot4name", robot4_name, "hiawatha");
	private_n.param<std::string>("robot5name", robot5_name, "darius");
	private_n.param<std::string>("robot6name", robot6_name, "caesar");

 	
 	ros::Subscriber subzero = n.subscribe(  robot1_name + "/odom", 10, Callbackzero);
	ros::Subscriber subone =  n.subscribe(  robot2_name + "/odom", 10, Callbackone);
	ros::Subscriber subtwo =  n.subscribe(  robot3_name + "/odom", 10, Callbacktwo);
	ros::Subscriber subthree =n.subscribe(  robot4_name + "/odom", 10, Callbackthree);
	ros::Subscriber subfour =  n.subscribe(  robot5_name + "/odom", 10, Callbackfour);
	ros::Subscriber subfive =n.subscribe(  robot6_name + "/odom", 10, Callbackfive);
	 
	
	ros::Publisher pubzero = n.advertise<geometry_msgs::TransformStamped>(robot1_name + "/tf", 10);
	ros::Publisher pubone = n.advertise<geometry_msgs::TransformStamped>(robot2_name + "/tf", 10);
	ros::Publisher pubtwo = n.advertise<geometry_msgs::TransformStamped>(robot3_name + "/tf", 10);
	ros::Publisher pubthree = n.advertise<geometry_msgs::TransformStamped>(robot4_name + "/tf", 10);
	ros::Publisher pubfour = n.advertise<geometry_msgs::TransformStamped>(robot5_name + "/tf", 10);
	ros::Publisher pubfive = n.advertise<geometry_msgs::TransformStamped>(robot6_name + "/tf", 10);
	 
	if ( robot_name.compare("gandhi") == 0 ) { 
		id = 0;
	} else if ( robot_name.compare("suleyman") == 0) {
		id = 1;
	} else if ( robot_name.compare("alexander") == 0) {
		id = 2;
	} else if ( robot_name.compare("hiawatha") == 0) {
		id = 3;
	} else if ( robot_name.compare("darius") == 0) {
		id = 4;
	} else if ( robot_name.compare("caesar") == 0) {
		id = 5;
	}
	
	//ros::Publisher vel_pub = n.advertise<create_driver::WheelVelocities>(robot_name + "_vel", 10);

	
	ros::Rate loop_rate(100); // 100 Hz

	
	ros::spinOnce(); 
	
	while (ros::ok())
	{   
		
		
		// One of the agents moves in a circle
		/*		
		if(id==3){
			v=-300;
			w=1/4.0;
		}
		*/	

		pubzero.publish(pumsg[0]);
		pubone.publish(pumsg[1]);
		pubtwo.publish(pumsg[2]);
		pubthree.publish(pumsg[3]);
		pubfour.publish(pumsg[4]);
		pubfive.publish(pumsg[5]);

		/*
		create_driver::WheelVelocities msg;
		msg.rightWheelVelocity = (int)rwv;
		msg.leftWheelVelocity = (int)lwv;
		vel_pub.publish(msg);
		*/
		// Update Diagnostics information

		loop_rate.sleep();
		ros::spinOnce(); 

	}
	
}
