#include <ros/ros.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "connectivity_controller_old/Control_oldMsgs.h"
#include "connectivity_controller_old/Connectivity_oldMsgs.h"
#include "dynamic_reconfigure/server.h"
#include <connectivity_controller_old/Control_oldVariablesConfig.h>
#include <math.h>
#include <algorithm>
#include "ros/console.h"
#include "stdio.h"
//#include <LinearMath/btQuaternion.h>
//#include <LinearMath/btMatrix3x3.h>
#include "tf/transform_datatypes.h" 
#include "nav_msgs/Odometry.h"
#include "Eigen/Core"

#define PI	3.1415926535
#define rho	133.334
#define alphaOverBar 900
#define alphaUnderBar 0

using namespace Eigen;

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

void reconfigureCallback(connectivity_controller_old::Control_oldVariablesConfig &config, uint32_t level) {
	k = config.k;
	ktheta = config.ktheta;
	power = config.power;
	//xd = config.xd;
	//yd = config.yd;
}

void Callbackzero(const nav_msgs::Odometry::ConstPtr& msg)
{
	double pitch, roll;
	x[0] = msg->pose.pose.position.x * 0.001; 
	y[0] = msg->pose.pose.position.y * 0.001;  
	
	// get rotation
	tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, 
											msg->pose.pose.orientation.y, 
											msg->pose.pose.orientation.z,
											msg->pose.pose.orientation.w);
											
   tf::Matrix3x3(q).getEulerYPR(theta[0], pitch, roll); // convert to radians

}

void Callbackone(const nav_msgs::Odometry::ConstPtr& msg)
{
	double pitch, roll;
	x[1] = msg->pose.pose.position.x * 0.001; 
	y[1] = msg->pose.pose.position.y * 0.001;  
	
	// get rotationp
	tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, 
											msg->pose.pose.orientation.y, 
											msg->pose.pose.orientation.z,
											msg->pose.pose.orientation.w);
											
   tf::Matrix3x3(q).getEulerYPR(theta[1], pitch, roll); // convert to radians
}

void Callbacktwo(const nav_msgs::Odometry::ConstPtr& msg)
{
	double pitch, roll;
	x[2] = msg->pose.pose.position.x * 0.001; 
	y[2] = msg->pose.pose.position.y * 0.001;  
	
	// get rotation
	tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, 
											msg->pose.pose.orientation.y, 
											msg->pose.pose.orientation.z,
											msg->pose.pose.orientation.w);
											
   tf::Matrix3x3(q).getEulerYPR(theta[2], pitch, roll); // convert to radians
}

void Callbackthree(const nav_msgs::Odometry::ConstPtr& msg)
{
	double pitch, roll;
	x[3] = msg->pose.pose.position.x * 0.001; 
	y[3] = msg->pose.pose.position.y * 0.001;  
	
	// get rotation
	tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, 
											msg->pose.pose.orientation.y, 
											msg->pose.pose.orientation.z,
											msg->pose.pose.orientation.w);
											
   tf::Matrix3x3(q).getEulerYPR(theta[3], pitch, roll); // convert to radians
}

void Callbackfour(const nav_msgs::Odometry::ConstPtr& msg)
{
	double pitch, roll;
	x[4] = msg->pose.pose.position.x * 0.001; 
	y[4] = msg->pose.pose.position.y * 0.001;  
	
	// get rotation
	tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, 
											msg->pose.pose.orientation.y, 
											msg->pose.pose.orientation.z,
											msg->pose.pose.orientation.w);
											
   tf::Matrix3x3(q).getEulerYPR(theta[4], pitch, roll); // convert to radians
}

void Callbackfive(const nav_msgs::Odometry::ConstPtr& msg)
{
	double pitch, roll;
	x[5] = msg->pose.pose.position.x * 0.001; 
	y[5] = msg->pose.pose.position.y * 0.001;  
	
	// get rotation
	tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, 
											msg->pose.pose.orientation.y, 
											msg->pose.pose.orientation.z,
											msg->pose.pose.orientation.w);
		 									
   tf::Matrix3x3(q).getEulerYPR(theta[5], pitch, roll); // convert to radians
}
 
void ConnectCallback(const connectivity_controller_old::Connectivity_oldMsgs::ConstPtr& msg) {
	trMx[0] = msg->trMx[0];
	trMx[1] = msg->trMx[1];
	trMx[2] = msg->trMx[2];
	trMx[3] = msg->trMx[3];
	trMx[4] = msg->trMx[4];
	trMx[5] = msg->trMx[5];
	
	trMy[0] = msg->trMy[0];
	trMy[1] = msg->trMy[1];
	trMy[2] = msg->trMy[2];
	trMy[3] = msg->trMy[3];
	trMy[4] = msg->trMy[4];
	trMy[5] = msg->trMy[5];
	
	u_ca_x[0] = msg->u_ca_x[0];
	u_ca_x[1] = msg->u_ca_x[1];
	u_ca_x[2] = msg->u_ca_x[2];
	u_ca_x[3] = msg->u_ca_x[3];
	u_ca_x[4] = msg->u_ca_x[4];
	u_ca_x[5] = msg->u_ca_x[5];
	
	u_ca_y[0] = msg->u_ca_y[0];
	u_ca_y[1] = msg->u_ca_y[1];
	u_ca_y[2] = msg->u_ca_y[2];
	u_ca_y[3] = msg->u_ca_y[3];
	u_ca_y[4] = msg->u_ca_y[4];
	u_ca_y[5] = msg->u_ca_y[5];
	
	u_fc_x[0] = msg->u_fc_x[0];
	u_fc_x[1] = msg->u_fc_x[1];
	u_fc_x[2] = msg->u_fc_x[2];
	u_fc_x[3] = msg->u_fc_x[3];
	u_fc_x[4] = msg->u_fc_x[4];
	u_fc_x[5] = msg->u_fc_x[5];
	
	u_fc_y[0] = msg->u_fc_y[0];
	u_fc_y[1] = msg->u_fc_y[1];
	u_fc_y[2] = msg->u_fc_y[2];
	u_fc_y[3] = msg->u_fc_y[3];
	u_fc_y[4] = msg->u_fc_y[4];
	u_fc_y[5] = msg->u_fc_y[5];
	
	
	detM = msg->detM;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control");


	ros::NodeHandle n;
	ros::NodeHandle private_n("~");
	
	private_n.param<std::string>("robotname", robot_name, "gizmo");
	
	private_n.param<std::string>("robot1name", robot1_name, "gizmo");
	private_n.param<std::string>("robot2name", robot2_name, "hal");
	private_n.param<std::string>("robot3name", robot3_name, "coyote");
	private_n.param<std::string>("robot4name", robot4_name, "road_runner");
	private_n.param<std::string>("robot5name", robot5_name, "darius");
	private_n.param<std::string>("robot6name", robot6_name, "caesar");

	dynamic_reconfigure::Server<connectivity_controller_old::Control_oldVariablesConfig> server;
  	dynamic_reconfigure::Server<connectivity_controller_old::Control_oldVariablesConfig>::CallbackType f;
  	f = boost::bind(&reconfigureCallback, _1, _2);
 	server.setCallback(f);
 	
 	ros::Subscriber subzero = n.subscribe(  robot1_name + "/odom", 10, Callbackzero);
	ros::Subscriber subone =  n.subscribe(  robot2_name + "/odom", 10, Callbackone);
	ros::Subscriber subtwo =  n.subscribe(  robot3_name + "/odom", 10, Callbacktwo);
	ros::Subscriber subthree =n.subscribe(  robot4_name + "/odom", 10, Callbackthree);
	ros::Subscriber subfour =  n.subscribe(  robot5_name + "/odom", 10, Callbackfour);
	ros::Subscriber subfive =n.subscribe(  robot6_name + "/odom", 10, Callbackfive);
	 
	ros::Subscriber subconnect = n.subscribe( "/connectivity", 10, ConnectCallback);
	
	ros::Publisher dia_pub = n.advertise<connectivity_controller_old::Control_oldMsgs>(robot_name + "/dia", 10);
	 
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
	ros::Publisher vel = n.advertise<geometry_msgs::Twist>(robot_name + "/cmd_vel", 10);
	
	ros::Rate loop_rate(100); // 100 Hz

	
	ros::spinOnce(); 
	
	while (ros::ok())
	{   
		MatrixXd S(12,3);
		S << 	1,	0,	(-5*y[0] + y[1] + y[2] + y[3] + y[4] + y[5])/6.0,
				0,	1,	(5*x[0] - x[1] - x[2] - x[3] - x[4] - x[5])/6.0,
				1,	0,	(y[0] - 5*y[1] + y[2] + y[3] + y[4] + y[5])/6.0,
				0,	1,	(-x[0] + 5*x[1] - x[2] - x[3] - x[4] - x[5])/6.0,
				1,	0,	(y[0] + y[1] - 5*y[2] + y[3] + y[4] + y[5])/6.0,
				0,	1,	(-x[0] - x[1] + 5*x[2] - x[3] - x[4] - x[5])/6.0,
				1,	0,	(y[0] + y[1] + y[2] - 5*y[3] + y[4] + y[5])/6.0,
				0,	1,	(-x[0] - x[1] - x[2] + 5*x[3] - x[4] - x[5])/6.0,
				1,	0,	(y[0] + y[1] + y[2] + y[3] - 5*y[4] + y[5])/6.0,
				0,	1,	(-x[0] - x[1] - x[2] - x[3] + 5*x[4] - x[5])/6.0,
				1,	0,	(y[0] + y[1] + y[2] + y[3] + y[4] - 5*y[5])/6.0,
				0,	1,	(-x[0] - x[1] - x[2] - x[3] - x[4] + 5*x[5])/6.0;
				
		double x_bar = (x[0] + x[1] + x[2] + x[3] + x[4] + x[5])/6.0;		
		double y_bar = (y[0] + y[1] + y[2] + y[3] + y[4] + y[5])/6.0;
		double psi = atan2(y[2] - y[0], x[2] - x[0]);
		
		



		if (psi > PI){
			psi = psi - 2*PI;
		} else if (psi < -PI){
			psi = psi + 2*PI;
		}
	
		
		double x_bar_desired = 0.25;
		double y_bar_desired = 0.5;
		double psi_desired = 3*PI/4;
		double k_bar = 1;
			
		MatrixXd eta(3,1);
		eta << 	(x_bar - x_bar_desired),
				(y_bar - y_bar_desired),
				(psi - psi_desired);

		//double xd[6]={ 0.5457, 1.9729, 1.237,-0.7, -0.3, -0.9};	
		//double yd[6]={-1.5740,-1.5810,-1.539,-0.7, -1.54, -1.54};
		double xd[6]={ 0.5457, 1.9729, 1.237,-0.7, -0.3, -0.9};	
		double yd[6]={ -1.5740,-1.5810,-1.539,-0.7, -1.54, -1.54};
		//double kconn = 1.0,kca = 2.0e-02,kfc = 1.0; // 1 means same influence as connectivity. final gain on coll avoid = k*kca
		double kconn = 1.0e-01,kca = 8.0e-02,kfc = 1.0;
		double uconnx = kconn*(((pow(alphaOverBar,2)-pow(alphaUnderBar,2))
				     *(pow(detM, 2)-pow(alphaOverBar,2)))
				     /(pow(pow(detM, 2)-pow(alphaUnderBar,2),3))) 
				     *pow(detM,2)*trMx[id];
		double ufx = -kfc*(x[id]-xd[id]);	// Formation control
		double ucx = kca*u_ca_x[id];		// Collision avoidance
				 
		double uconny = kconn*(((pow(alphaOverBar,2)-pow(alphaUnderBar,2))
				     *(pow(detM, 2)-pow(alphaOverBar,2)))
				     /(pow(pow(detM, 2)-pow(alphaUnderBar,2),3))) 
				     *pow(detM,2)*trMy[id];
		double ufy = -kfc*(y[id]-yd[id]);	// Formation control
		double ucy = kca*u_ca_y[id];		// Collision avoidance
		
		if ( detM < alphaUnderBar )
		{
			uconnx = 0.0;
			uconny = 0.0;
		}
		else if (detM > alphaOverBar )
		{
			uconnx = 0.0;
			uconny = 0.0;
		}
		
//		if(id==3){
//			ufx = 0.0;
//			ufy = 0.0;
//		}	
		
		MatrixXd u_bar = -k_bar*S*eta;

		double ux = uconnx + ufx +ucx;
		ux = uconnx + u_bar(2*id,0) + ucx;
		//ux = ufx;
//		ux = uconnx + ucx;
		double uy = uconny + ufy + ucy;

		uy = uconny + u_bar(2*id + 1,0) + ucy;
		//uy = ufy;
//		uy = uconny +ucy;
		
		double thetad = atan2(uconny/kconn + ufy/kfc + ucy/kca, uconnx/kconn + ufx/kfc + ucx/kca);
		thetad = atan2(uconny/kconn + u_bar(2*id+1,0)/k_bar + ucy/kca, uconnx/kconn + u_bar(2*id,0)/k_bar + ucx/kca);
//		double thetad = atan2(ufy/kfc, ufx/kfc);
		//double thetad = atan2(uconny/kconn + ucy/kca, uconnx/kconn + ucx/kca);
		//double thetad = atan2(uconny/kconn + ufy/kfc, uconnx/kconn + ufx/kfc);
		double etheta = theta[id] - thetad;

		/*		
		double vconn = k*1000*cos(etheta)*sqrt(pow(ux,2) + pow(uy,2));
		double vf = k*1000*cos(etheta)*sqrt(pow(ufx,2) + pow(ufy,2));
		double vca = k*1*cos(etheta)*sqrt(pow(ucx,2) + pow(ucy,2));
		*/
		
		double v = k*1000*cos(etheta)*sqrt(pow(ux,2) + pow(uy,2));;
		double w = -ktheta*etheta;
		
		
		// One of the agents moves in a circle
		/*		
		if(id==3){
			v=-300;
			w=1/4.0;
		}
		*/	

		if ( power ) {
		// Send wheel velocities to driver
			geometry_msgs::Twist msg;
			msg.linear.x = v;
			msg.linear.y = x_bar;
			msg.linear.z = y_bar;
			msg.angular.x = psi;
			msg.angular.y = detM;
			msg.angular.z = w;
			vel.publish(msg);
		} else if(fabs(x[id]) > 2000 || fabs(y[id]) > 2000 || !power){
			geometry_msgs::Twist msg;
			msg.linear.x = 0;
			msg.linear.y = theta[0];
			msg.linear.z = theta[1];
			msg.angular.x = theta[2];
			msg.angular.y = theta[3];
			msg.angular.z = 0;
			vel.publish(msg);
		}

		/*
		create_driver::WheelVelocities msg;
		msg.rightWheelVelocity = (int)rwv;
		msg.leftWheelVelocity = (int)lwv;
		vel_pub.publish(msg);
		*/
		// Update Diagnostics information
		connectivity_controller_old::Control_oldMsgs dmsg; 
		dmsg.ux = ux;
		dmsg.uy = uy;
		dmsg.etheta = etheta;
		dmsg.thetad = thetad;
		dmsg.uconny = uconny;
		dmsg.ufy = ufy;
		dmsg.ucy = ucy;
		dmsg.uconny = uconnx;
		dmsg.ufy = ufx;
		dmsg.ucy = ucx;
		dmsg.psi = psi;

		// dmsg.theta_old = ux;
		// dmsg.thetad_old = uy;
		// dmsg.theta = x[id];
		// dmsg.thetad = y[id];
		dia_pub.publish(dmsg);
		
		ros::spinOnce(); 

		loop_rate.sleep();
	}
	
}
