#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "distributed_connectivity_controller/ControlMsgs.h"
#include "distributed_connectivity_controller/DistConnectivityMsgs.h"
#include "dynamic_reconfigure/server.h"
#include "distributed_connectivity_controller/ControlVariablesConfig.h"
#include <math.h>
#include <algorithm>
#include "ros/console.h"
#include "stdio.h"
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btMatrix3x3.h> 
#include "nav_msgs/Odometry.h"

#define PI	3.1415926535
#define rho	133.334
#define alphaOverBar 3.0
#define alphaUnderBar 0
#define NUM_ROBOTS 6


double k = 0.5; // linear velocity gain
double ktheta = 5.0; // angular velocity gain
double rd = 1.0;	    // Radius Desired
double alpha = 1.0;	    // parameter controlling desired orientation

bool power = true;
bool killSwitch = true;
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

double x[NUM_ROBOTS] = {0.0}; 
double y[NUM_ROBOTS] = {0.0};
double theta[NUM_ROBOTS] = {0.0};

double u_conn_x[NUM_ROBOTS] = {0.0};
double u_conn_y[NUM_ROBOTS] = {0.0};
double u_ca_x[NUM_ROBOTS] = {0.0};
double u_ca_y[NUM_ROBOTS] = {0.0};
double u_fc_x[NUM_ROBOTS] = {0.0};
double u_fc_y[NUM_ROBOTS] = {0.0};	
double lambda2[NUM_ROBOTS] = {0.0};


void reconfigureCallback(distributed_connectivity_controller::ControlVariablesConfig &config, uint32_t level) {
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
	btQuaternion q = btQuaternion(msg->pose.pose.orientation.x, 
											msg->pose.pose.orientation.y, 
											msg->pose.pose.orientation.z,
											msg->pose.pose.orientation.w);
											
   btMatrix3x3(q).getEulerYPR(theta[0], pitch, roll); // convert to radians
}

void Callbackone(const nav_msgs::Odometry::ConstPtr& msg)
{
	double pitch, roll;
	x[1] = msg->pose.pose.position.x * 0.001; 
	y[1] = msg->pose.pose.position.y * 0.001;  
	
	// get rotationp
	btQuaternion q = btQuaternion(msg->pose.pose.orientation.x, 
											msg->pose.pose.orientation.y, 
											msg->pose.pose.orientation.z,
											msg->pose.pose.orientation.w);
											
   btMatrix3x3(q).getEulerYPR(theta[1], pitch, roll); // convert to radians
}

void Callbacktwo(const nav_msgs::Odometry::ConstPtr& msg)
{
	double pitch, roll;
	x[2] = msg->pose.pose.position.x * 0.001; 
	y[2] = msg->pose.pose.position.y * 0.001;  
	
	// get rotation
	btQuaternion q = btQuaternion(msg->pose.pose.orientation.x, 
											msg->pose.pose.orientation.y, 
											msg->pose.pose.orientation.z,
											msg->pose.pose.orientation.w);
											
   btMatrix3x3(q).getEulerYPR(theta[2], pitch, roll); // convert to radians
}

void Callbackthree(const nav_msgs::Odometry::ConstPtr& msg)
{
	double pitch, roll;
	x[3] = msg->pose.pose.position.x * 0.001; 
	y[3] = msg->pose.pose.position.y * 0.001;  
	
	// get rotation
	btQuaternion q = btQuaternion(msg->pose.pose.orientation.x, 
											msg->pose.pose.orientation.y, 
											msg->pose.pose.orientation.z,
											msg->pose.pose.orientation.w);
											
   btMatrix3x3(q).getEulerYPR(theta[3], pitch, roll); // convert to radians
}

void Callbackfour(const nav_msgs::Odometry::ConstPtr& msg)
{
	double pitch, roll;
	x[4] = msg->pose.pose.position.x * 0.001; 
	y[4] = msg->pose.pose.position.y * 0.001;  
	
	// get rotation
	btQuaternion q = btQuaternion(msg->pose.pose.orientation.x, 
											msg->pose.pose.orientation.y, 
											msg->pose.pose.orientation.z,
											msg->pose.pose.orientation.w);
											
   btMatrix3x3(q).getEulerYPR(theta[4], pitch, roll); // convert to radians
}

void Callbackfive(const nav_msgs::Odometry::ConstPtr& msg)
{
	double pitch, roll;
	x[5] = msg->pose.pose.position.x * 0.001; 
	y[5] = msg->pose.pose.position.y * 0.001;  
	
	// get rotation
	btQuaternion q = btQuaternion(msg->pose.pose.orientation.x, 
											msg->pose.pose.orientation.y, 
											msg->pose.pose.orientation.z,
											msg->pose.pose.orientation.w);
											
   btMatrix3x3(q).getEulerYPR(theta[5], pitch, roll); // convert to radians
}

void ConnectCallback(const distributed_connectivity_controller::DistConnectivityMsgs::ConstPtr& msg) {
	u_conn_x[0] = msg->u_conn_x[0];
	u_conn_x[1] = msg->u_conn_x[1];
	u_conn_x[2] = msg->u_conn_x[2];
	u_conn_x[3] = msg->u_conn_x[3];
	u_conn_x[4] = msg->u_conn_x[4];
	u_conn_x[5] = msg->u_conn_x[5];

	u_conn_y[0] = msg->u_conn_y[0];
	u_conn_y[1] = msg->u_conn_y[1];
	u_conn_y[2] = msg->u_conn_y[2];
	u_conn_y[3] = msg->u_conn_y[3];
	u_conn_y[4] = msg->u_conn_y[4];
	u_conn_y[5] = msg->u_conn_y[5];

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
	
	lambda2[0] = msg->lambda2[0];
	lambda2[1] = msg->lambda2[1];
	lambda2[2] = msg->lambda2[2];
	lambda2[3] = msg->lambda2[3];
	lambda2[4] = msg->lambda2[4];
	lambda2[5] = msg->lambda2[5];
	killSwitch = msg->power;
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
	private_n.param<std::string>("robot5name", robot5_name, "coyote");
	private_n.param<std::string>("robot6name", robot6_name, "road_runner");

	dynamic_reconfigure::Server<distributed_connectivity_controller::ControlVariablesConfig> server;
  	dynamic_reconfigure::Server<distributed_connectivity_controller::ControlVariablesConfig>::CallbackType f;
  	f = boost::bind(&reconfigureCallback, _1, _2);
 	server.setCallback(f);
 	
 	ros::Subscriber subzero = n.subscribe(  robot1_name + "/odom", 10, Callbackzero);
	ros::Subscriber subone =  n.subscribe(  robot2_name + "/odom", 10, Callbackone);
	ros::Subscriber subtwo =  n.subscribe(  robot3_name + "/odom", 10, Callbacktwo);
	ros::Subscriber subthree =n.subscribe(  robot4_name + "/odom", 10, Callbackthree);
	ros::Subscriber subfour = n.subscribe(  robot5_name + "/odom", 10, Callbackfour);
	ros::Subscriber subfive = n.subscribe(  robot6_name + "/odom", 10, Callbackfive);
	
	ros::Subscriber subconnect = n.subscribe( "/connectivity", 10, ConnectCallback);
	
	ros::Publisher dia_pub = n.advertise<distributed_connectivity_controller::ControlMsgs>(robot_name + "/dia", 10);
	
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

	double begin = ros::Time::now().toSec();
	
	ros::spinOnce(); 
	
	while (ros::ok())
	{   

	
		double kconn = 1.0e-01,kca = 2.0e-02,kfc = 1.0; // 1 means same influence as connectivity. final gain on coll avoid = k*kca
		
		
		if ( lambda2[id]< alphaUnderBar )
		{
			u_conn_x[id]= 0.0;
			u_conn_y[id] = 0.0;
		}
		else if (lambda2[id] > alphaOverBar )
		{
			u_conn_x[id]= 0.0;
			u_conn_y[id] = 0.0;
		}	

//	To make hal less responsive to connectivity
//		u_conn_x[3] =0.1*u_conn_x[3];
//		u_conn_y[3] =0.1*u_conn_y[3];
			
		
		double ux = kconn*u_conn_x[id] + kfc*u_fc_x[id] + kca*u_ca_x[id];
		//ux = kconn*u_conn_x[id] +  kfc*u_fc_x[id];
		//ux = kconn*u_conn_x[id] + kca*u_ca_x[id];
		double uy = kconn*u_conn_y[id] + kfc*u_fc_y[id] + kca*u_ca_y[id];
		//uy = kconn*u_conn_y[id] + kfc*u_fc_y[id];
		//uy = kconn*u_conn_y[id] + kca*u_ca_y[id];
		
		/*if(id==3)
		{
			ux = 0.1*kconn*u_conn_x[id] + kfc*u_fc_x[id] + kca*u_ca_x[id];
			uy = 0.1*kconn*u_conn_y[id] + kfc*u_fc_y[id] + kca*u_ca_y[id];
		}*/
		
		
		double thetad = atan2(uy,ux);
//		double thetad = atan2(u_conn_y[id] + u_fc_y[id] + u_ca_y[id], u_conn_x[id] + u_fc_x[id] + u_ca_x[id]);
		//double thetad = atan2(u_conn_y[id] + u_fc_y[id],  u_conn_x[id] +  u_fc_x[id]);
//		double thetad = atan2(u_conn_y[id] + u_ca_y[id], u_conn_x[id] + u_ca_x[id]);
		double etheta = theta[id] - thetad;
		
/*		if(id==3){
			ux = kconn*u_conn_x[id] +  kca*u_ca_x[id];
			uy = kconn*u_conn_y[id] +  kca*u_ca_y[id];
			thetad = atan2(u_conn_y[id] + u_ca_y[id],  u_conn_x[id] + u_ca_x[id]);
			etheta = theta[id] - thetad;			
		}*/
		
		
		/*		
		double vconn = k*1000*cos(etheta)*sqrt(pow(ux,2) + pow(uy,2));
		double vf = k*1000*cos(etheta)*sqrt(pow(ufx,2) + pow(ufy,2));
		double vca = k*1*cos(etheta)*sqrt(pow(ucx,2) + pow(ucy,2));
		*/


				
		double v = k*1000*cos(etheta)*sqrt(pow(ux,2) + pow(uy,2));
		double w = -ktheta*etheta;
		
		if (id == 220) {
			double h = pow(x[id], 2) + pow(y[id], 2) - pow(rd,2);
			double thetad = atan2(y[id],x[id]) + PI/2 + atan(alpha*h);
			if (thetad > PI){
				thetad = thetad - 2*PI;
			} else if (thetad < -PI){
				thetad = thetad + 2*PI;
			}
		
			if ((theta[id] > PI/2) && (thetad < -PI/2)){
				thetad = thetad+2*PI;
			}
		
			if ((theta[id] < -PI/2) && (thetad > PI/2)){
				thetad = thetad-2*PI;
			}		
			
		
			w = -ktheta*(theta[id]-thetad);
			v = 200.00;
		
		}
		
		// One of the agents moves in a circle
			
/*		if(id==3){
			v=-300;
			w=1/4.0;
		}*/
		
		


		if ( (power&&killSwitch) && (ros::Time::now().toSec()-begin) > 4.0) {
		// Send wheel velocities to driver
			geometry_msgs::Twist msg;
			msg.linear.x = v;
			msg.linear.y = 0;
			msg.linear.z = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;
			msg.angular.z = w;
			vel.publish(msg);
		} else if(fabs(x[id]) > 2000 || fabs(y[id]) > 2000 || !(power&&killSwitch) ||(ros::Time::now().toSec()-begin) <= 4.0 ){
			geometry_msgs::Twist msg;
			msg.linear.x = 0;
			msg.linear.y = 0;
			msg.linear.z = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;
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
		distributed_connectivity_controller::ControlMsgs dmsg; 
		dmsg.theta_old = 0;
		dmsg.thetad_old = 0;
		dmsg.theta = 0;
		dmsg.thetad = 0;
		dia_pub.publish(dmsg);
		
		ros::spinOnce(); 

		loop_rate.sleep();
	}
	
}
