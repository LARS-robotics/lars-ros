#include "vision_based_localization/VisionBasedLocalizationMsgs.h"
#include "ros/ros.h"
#include <vision_based_adaptive_following/AdaptiveControlVariablesConfig.h>
#include "geometry_msgs/Twist.h"

#include "dynamic_reconfigure/server.h"

#include <math.h>
#include <algorithm>
#include "ros/console.h"
#include "stdio.h"
//#include <LinearMath/btQuaternion.h>
//#include <LinearMath/btMatrix3x3.h> 
//#include "nav_msgs/Odometry.h"

#define PI	3.1415926535

//double vl = 0.2;		// Leader Linear Velocity
//double wl = 0.108;		// Leader Angular Velocity
double k = 1;
double kv = 1.0;		// Velocity Gain
double kw = 1.2;		// Rotation Gain
double ksigma = 0.0;	// Estimator Gain
double vmax = 0.5;		// Maximum Velocity
double epsilon = 0.25;	// Angle Bound
double rho_min = 0.3;	// Minimum Distance
double rhod = 0.9*1000;		// Rho Desired
double psid = PI/8.0;		// Psi Desired
double rd = 1.2;	    // Radius Desired
double alpha = 1.0;	    // Radius Desired
bool power = true;		// ON/OFF

double rho = 0.0;
double psi = 0.0;
double gamm = 0.0;




void Callbackzero(const vision_based_localization::VisionBasedLocalizationMsgs::ConstPtr& msg)
{
	psi = msg->psi;
	gamm = msg->gamma;
	rho = msg->rho;
}

void reconfigureCallback(vision_based_adaptive_following::AdaptiveControlVariablesConfig &config, uint32_t level) 
{

	//wl = config.wl;
	kv = config.kv;
	kw = config.kw;
	ksigma = config.ksigma;
	vmax = config.vmax;
	epsilon = config.epsilon;
	rho_min = config.rho_min;
	rhod = config.rhod;
	psid = config.psid;
	power = config.power;
}

double sigmaIntegrator(double gamma, double psi, double rho, double dT) 
{
	static double rhs_old = 0.0;
	static double sigma_old = 0.0;
	if ( fabs(rho-0.0) < 1e-10 && fabs(psi-0.0) < 1e-10 && fabs(gamma-0.0) < 1e-10 ) {
		rhs_old = 0.8*rhs_old;
		sigma_old = 0.8*sigma_old;
	} else {
		double rhs = (ksigma/4.0)*cos(gamma - psi)*(rho - rhod);
		double sigma = sigma_old + (dT/2)*(rhs_old + rhs);
		rhs_old = rhs;
		sigma_old = sigma;
		
	}
	
	return sigma_old;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vision_based_adaptive_following_node");

	ros::NodeHandle n;
	ros::NodeHandle private_n("~");



	dynamic_reconfigure::Server<vision_based_adaptive_following::AdaptiveControlVariablesConfig> server;
  	dynamic_reconfigure::Server<vision_based_adaptive_following::AdaptiveControlVariablesConfig>::CallbackType f;
  	f = boost::bind(&reconfigureCallback, _1, _2);
 	server.setCallback(f);
 	
	ros::Subscriber subzero = n.subscribe( "localization", 1, Callbackzero);
	
	
	ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	
	
	double begin;
	
	do { 
		begin = ros::Time::now().toSec();
	} while ( begin == 0 );
	
	ros::Rate loop_rate(30); // 100 Hz
	
	ros::spinOnce(); 
	

	
	bool firstExecution = true;
	bool firstExecutiond = true;
	
	double t = ros::Time::now().toSec() - begin;
	double t_old = t;
	double v = 0.0;
	double w = 0.0;

	while (ros::ok())
	{
		t = ros::Time::now().toSec() - begin;
		double dT = t - t_old;
		t_old = t;

		double sigma = sigmaIntegrator( gamm, psi, rho, dT );

		if ( fabs(rho-0.0) < 1e-10 && fabs(psi-0.0) < 1e-10 && fabs(gamm-0.0) < 1e-10 ) {
			v = 0.8*fmod(v, 500.0);
			w = 0.8*w;
		} else {
			v = (kv*(rho - rhod) + sigma*cos(gamm - psi))/cos(psi);
			w = (v*sin(psi))/rho + sigma*sin(gamm - psi)/rho + kw*(psi - psid);
		}

		if ( v != v || w != w ) {
			ROS_INFO("rho = %f", rho);
			ROS_INFO("psi = %f", psi); 
			ROS_INFO("gamma = %f", gamm);
		}

		//ROS_INFO("rho = %f", rho);
		//ROS_INFO("psi = %f", psi); 
		//ROS_INFO("gamma = %f", gamm); 
		//ROS_INFO("v = %f", v); 
		//ROS_INFO("kv*(rho - rhod) = %f", kv*(rho - rhod)); 
		//ROS_INFO("sigma*cos(gamm - psi) = %f", sigma*cos(gamm - psi));
		//ROS_INFO("cos(psi) = %f", cos(psi)); 	
		
		if ( power ) { //&& !isnan(v) && !isnan(w) ) {
		// Send wheel velocities to driver
			geometry_msgs::Twist msg;
			msg.linear.x = v/1000.0;
			msg.linear.y = 0;
			msg.linear.z = sigma;
			msg.angular.x = 0;
			msg.angular.y = 0;
			msg.angular.z = -w;
			cmd_vel.publish(msg);
		} else {
			geometry_msgs::Twist msg;
			msg.linear.x = 0;
			msg.linear.y = 0;
			msg.linear.z = sigma;
			msg.angular.x = 0;
			msg.angular.y = 0;
			msg.angular.z = 0;
			cmd_vel.publish(msg);
		}
		ros::spinOnce();
		loop_rate.sleep();
			
		
	}
}
