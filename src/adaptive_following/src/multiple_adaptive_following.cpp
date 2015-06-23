#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "adaptive_following/AdaptiveFollowingMsgs.h"
#include "dynamic_reconfigure/server.h"
#include "adaptive_following/AdaptiveControlVariablesConfig.h"
#include <math.h>
#include <algorithm>
#include "ros/console.h"
#include "stdio.h"
//#include <LinearMath/btQuaternion.h>
//#include <LinearMath/btMatrix3x3.h> 
#include "Eigen/Core"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h" 

#define PI	3.1415926535

double vl = 0.2;		// Leader Linear Velocity
double wl = 0.108;		// Leader Angular Velocity
double kv = 1.25;		// Velocity Gain
double kw = 1.2;		// Rotation Gain
double ksigma = 1.0;	// Estimator Gain
double vmax = 0.5;		// Maximum Velocity
double epsilon = 0.25;	// Angle Bound
double rho_min = 0.3;	// Minimum Distance
double rhod = 0.7;		// Rho Desired
double psid = 0.0;		// Psi Desired
bool power = true;		// ON/OFF

std::string follower_robot_name;
std::string leader_robot_name;

std::string robot1_name;
std::string robot2_name;

double x[2] = {0.0}; 
double y[2] = {0.0};
double theta[2] = {0.0};

double rhs_old = 0.0;
double sigma_old = 0.0;

int follower = 0;
int leader = 1;

void Callbackzero(const nav_msgs::Odometry::ConstPtr& msg)
{
	double pitch, roll;
	x[0] = msg->pose.pose.position.x * 0.001; 
	y[0] = msg->pose.pose.position.y * 0.001 + 1;  
	
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
	y[1] = msg->pose.pose.position.y * 0.001 + 1;  
	
	// get rotationp
	tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, 
											msg->pose.pose.orientation.y, 
											msg->pose.pose.orientation.z,
											msg->pose.pose.orientation.w);
											
   tf::Matrix3x3(q).getEulerYPR(theta[1], pitch, roll); // convert to radians
}

void reconfigureCallback(adaptive_following::AdaptiveControlVariablesConfig &config, uint32_t level) 
{
	vl = config.vl;
	wl = config.wl;
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
	double rhs = (ksigma/4.0)*cos(gamma - psi)*(rho - rhod);
	
	double sigma = sigma_old + (dT/2)*(rhs_old + rhs);
	
	rhs_old = rhs;
	sigma_old = sigma;
	
	return sigma;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "adaptive_following");

	ros::NodeHandle n;
	ros::NodeHandle private_n("~");
	
	private_n.param<std::string>("followerrobotname", follower_robot_name, "coyote");
	private_n.param<std::string>("leaderrobotname", leader_robot_name, "hal");
	
	private_n.param<std::string>("robot1name", robot1_name, "coyote");
	private_n.param<std::string>("robot2name", robot2_name, "hal");

	dynamic_reconfigure::Server<adaptive_following::AdaptiveControlVariablesConfig> server;
  	dynamic_reconfigure::Server<adaptive_following::AdaptiveControlVariablesConfig>::CallbackType f;
  	f = boost::bind(&reconfigureCallback, _1, _2);
 	server.setCallback(f);
 	
	ros::Subscriber subzero = n.subscribe(  robot1_name + "/odom", 10, Callbackzero);
	ros::Subscriber subone =  n.subscribe(  robot2_name + "/odom", 10, Callbackone);

	ros::Publisher dia_pub = private_n.advertise<adaptive_following::AdaptiveFollowingMsgs>( leader_robot_name + "/dia", 10);
	
	if ( follower_robot_name.compare(robot1_name) == 0 ) {
		follower = 0;
		leader = 1;
	} else if ( follower_robot_name.compare(robot2_name) == 0) {
		follower = 1;
		leader = 0;
	} 
	
	
	ros::Publisher follower_vel = n.advertise<geometry_msgs::Twist>(follower_robot_name + "/cmd_vel", 10);
	
	double begin;
	
	do { 
		begin = ros::Time::now().toSec();
	} while ( begin == 0 );
	
	ros::Rate loop_rate(100); // 100 Hz
	
	ros::spinOnce(); 
	
	double rho = 0.0;
	double psi = 0.0;
	double gamma = 0.0;
	double sigma = 0.0;
	
	double theta_relvec;
	
	double theta_out = 0.0;
	double theta_relvec_out = 0.0;
	double theta_old = theta[follower];
	double theta_relvec_old = theta_relvec;
	
	int theta_loop = 0;
	int theta_relvec_loop = 0;
	
	bool firstExecution = true;
	bool firstExecutiond = true;
	
	double t = ros::Time::now().toSec() - begin;
	double t_old = t;
	
	while (ros::ok())
	{
		t = ros::Time::now().toSec() - begin;
		double dT = t - t_old;
		t_old = t;
		
		if ( !firstExecution) {
			int n = theta_loop;
			if( (theta_old - theta[follower] )> PI/2 ){
	   			n = n+1;
	   		} else if((theta[follower] - theta_old ) > PI/2 ){
				n = n-1;
			}
			theta_old = theta[follower];
			theta_out = theta[follower] + 2 * n * PI;
			theta_loop = n;
			
		} else {
			theta_old = theta[follower];
			firstExecution = false;
		}
		
		theta_relvec = atan2((y[leader] - y[follower]), (x[leader] - x[follower]));
		
		if ( !firstExecutiond) {
			int m = theta_relvec_loop;
			if(( theta_relvec_old -theta_relvec )> PI/2){
	   			m = m+1;
	   		} else if((theta_relvec - theta_relvec_old ) > PI/2){
				m = m-1;
			}
			theta_relvec_old = theta_relvec;
			theta_relvec_out = theta_relvec + 2 * m * PI;
			
			theta_relvec_loop = m;
			
		} else {
			theta_relvec_old = theta_relvec;
			firstExecutiond = false;
		}
		
		rho = sqrt(pow((x[leader] - x[follower]), 2) + pow((y[leader] - y[follower]), 2));
		
		psi = theta_relvec_out - theta_out;
		
		gamma = theta[leader] - theta[follower];
		
		sigma = sigmaIntegrator( gamma, psi, rho, dT);
		
		double v = (kv*(rho - rhod) + sigma*cos(gamma - psi))/cos(psi);
		double w = (v*sin(psi))/rho + sigma*sin(gamma - psi)/rho + kw*(psi - psid);
		
		if ( power ) {
		// Send wheel velocities to driver
			geometry_msgs::Twist msg;
			msg.linear.x = v*1000;
			msg.linear.y = 0;
			msg.linear.z = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;
			msg.angular.z = w;
			follower_vel.publish(msg);
			
		} else {
			geometry_msgs::Twist msg;
			msg.linear.x = 0;
			msg.linear.y = 0;
			msg.linear.z = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;
			msg.angular.z = 0;
			follower_vel.publish(msg);
			sigma_old = 0.0;
		}

		adaptive_following::AdaptiveFollowingMsgs dmsg; 
		dmsg.rho = rho;
		dmsg.psi = psi;
		dmsg.gamma = gamma;
		dmsg.sigma = sigma;
		dmsg.theta_follower = theta[follower];
		dmsg.theta_leader = theta[leader];
		dia_pub.publish(dmsg);
		
		ros::spinOnce(); 

		loop_rate.sleep();
	}
	
}
