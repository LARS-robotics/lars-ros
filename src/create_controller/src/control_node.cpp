#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "create_controller/ControlMsgs.h"
#include "dynamic_reconfigure/server.h"
#include "create_controller/ControlVariablesConfig.h"
#include <math.h>
#include <algorithm>
#include "ros/console.h"
#include "stdio.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include "eigen_conversions/eigen_msg.h" 

#define PI	3.1415926535
#define rho	133.334



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

double x[4] = {0.0}; 
double y[4] = {0.0};
double theta[4] = {0.0};
double centroid_x(0), centroid_y(0), radius(0);

int id = 0;

void Callbackcentroid(const geometry_msgs::Point::ConstPtr& msg)
{
	centroid_x = msg->x; 
	centroid_y = msg->y;  	
	radius = msg->z;
}

void Callbackzero(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	x[0] = msg->transform.translation.x*1000;
	y[0] = msg->transform.translation.y*1000;
	
	Eigen::Quaterniond eigen_q;
	tf::quaternionMsgToEigen(msg->transform.rotation, eigen_q);
	tf::Quaternion q;
	tf::quaternionEigenToTF(eigen_q, q);
	double pitch, roll;											
	tf::Matrix3x3(q).getEulerYPR(theta[0], pitch, roll); // convert to radians
}

void Callbackone(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	x[1] = msg->transform.translation.x*1000;
	y[1] = msg->transform.translation.y*1000;
	
	Eigen::Quaterniond eigen_q;
	tf::quaternionMsgToEigen(msg->transform.rotation, eigen_q);
	tf::Quaternion q;
	tf::quaternionEigenToTF(eigen_q, q);
	double pitch, roll;											
	tf::Matrix3x3(q).getEulerYPR(theta[1], pitch, roll); // convert to radians
}

void Callbacktwo(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	x[2] = msg->transform.translation.x*1000;
	y[2] = msg->transform.translation.y*1000;
	
	Eigen::Quaterniond eigen_q;
	tf::quaternionMsgToEigen(msg->transform.rotation, eigen_q);
	tf::Quaternion q;
	tf::quaternionEigenToTF(eigen_q, q);
	double pitch, roll;											
	tf::Matrix3x3(q).getEulerYPR(theta[2], pitch, roll); // convert to radians
}

void Callbackthree(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	x[3] = msg->transform.translation.x*1000;
	y[3] = msg->transform.translation.y*1000;
	
	Eigen::Quaterniond eigen_q;
	tf::quaternionMsgToEigen(msg->transform.rotation, eigen_q);
	tf::Quaternion q;
	tf::quaternionEigenToTF(eigen_q, q);
	double pitch, roll;											
	tf::Matrix3x3(q).getEulerYPR(theta[3], pitch, roll); // convert to radians
}

void reconfigureCallback(create_controller::ControlVariablesConfig &config, uint32_t level) {
	k = config.k;
	ktheta = config.ktheta;
	power = config.power;
	//xd = config.xd;
	//yd = config.yd;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control");

	ros::NodeHandle n;
	ros::NodeHandle private_n("~");
	
	private_n.param<std::string>("robotname", robot_name, "gandhi");
	
	private_n.param<std::string>("robot1name", robot1_name, "gandhi");
	private_n.param<std::string>("robot2name", robot2_name, "alexander");
	private_n.param<std::string>("robot3name", robot3_name, "suleyman");
	private_n.param<std::string>("robot4name", robot4_name, "hiawatha");

	dynamic_reconfigure::Server<create_controller::ControlVariablesConfig> server;
  	dynamic_reconfigure::Server<create_controller::ControlVariablesConfig>::CallbackType f;
  	f = boost::bind(&reconfigureCallback, _1, _2);
 	server.setCallback(f);
 	
	ros::Subscriber subzero = n.subscribe(  robot1_name + "/tf", 10, Callbackzero);
	ros::Subscriber subone =  n.subscribe(  robot2_name + "/tf", 10, Callbackone);
	ros::Subscriber subtwo =  n.subscribe(  robot3_name + "/tf", 10, Callbacktwo);
	ros::Subscriber subthree =n.subscribe(  robot4_name + "/tf", 10, Callbackthree);
	
	ros::Subscriber centroid_sub = n.subscribe("centroid", 10, Callbackcentroid);
	
	ros::Publisher dia_pub = n.advertise<create_controller::ControlMsgs>(robot_name + "/dia", 10);
	
	if ( robot_name.compare("gandhi") == 0 ) {
		id = 0;
	} else if ( robot_name.compare("alexander") == 0) {
		id = 1;
	} else if ( robot_name.compare("suleyman") == 0) {
		id = 2;
	} else if ( robot_name.compare("hiawatha") == 0) {
		id = 3;
	}
	
	//ros::Publisher vel_pub = n.advertise<create_driver::WheelVelocities>(robot_name + "_vel", 10);
	ros::Publisher vel = n.advertise<geometry_msgs::Twist>(robot_name + "/cmd_vel", 10);
	// centroid of formation 
	double xcbar0 = 0;
	double ycbar0 = 0; 
	
	double R = 6000;	// force field max range
	double r = 300;	// force field min range
	
	// formation
	double a = 500;
	double deltax[4] = {a, 0, -a, 0};
	double deltay[4] = {0, a, 0, -a};
	
	double begin;
	
	do { 
		begin = ros::Time::now().toSec();
	} while ( begin == 0 );
	
	ros::Rate loop_rate(100); // 100 Hz
	
	ros::spinOnce(); 
	
	double thetad_out = 0.0;
	double theta_out = 0.0;
	double theta_old = theta[id];
	
	int theta_loop = 0;
	
	double thetad;
	double thetad_old = thetad;
	bool firstExecution = true;
	bool firstExecutiond = true;
	
	int thetad_loop = 0;
	
	while (ros::ok())
	{
		double t = ros::Time::now().toSec() - begin;
		
		if ( !firstExecution) {
			int n = theta_loop;
			if( (theta_old - theta[id] )> PI/2 ){
	   			n = n+1;
	   		} else if((theta[id] - theta_old ) > PI/2 ){
				n = n-1;
			}
			theta_old = theta[id];
			theta_out = theta[id] + 2 * n * PI;
			theta_loop = n;
			
		} else {
			theta_old = theta[id];
			firstExecution = false;
		}
		/* // Single Robot Control
		double r =sqrt( pow((x - xd), 2) + pow((y - yd), 2) );
		
		double thetad = atan2((y - yd), (x - xd));
		
		double v = -k*r;
		
		//double w = -ktheta*(theta - thetad);
		double w = 0.0;
		if(thetad < 0.0){
			w = -ktheta*( theta - ( thetad + PI ) );
		}
		else{
			w = -ktheta*( theta - ( thetad - PI ) );
		}
		
		double lwv = -v - rho * w;
		double rwv = -v + rho * w;
	
		create_driver::WheelVelocities msg;

		msg.rightWheelVelocity = (int)rwv;
		msg.leftWheelVelocity = (int)lwv;
		
		chatter_pub.publish(msg);
		*/
		
		// Multiple robot formation control 
		
		double da[4][4] = {{0.0}};	// distances between robots
		da[0][1] = sqrt(pow(x[0]-x[1],2) + pow(y[0]-y[1],2));
		da[0][2] = sqrt(pow(x[0]-x[2],2) + pow(y[0]-y[2],2));
		da[0][3] = sqrt(pow(x[0]-x[3],2) + pow(y[0]-y[3],2));
		da[1][0] = sqrt(pow(x[1]-x[0],2) + pow(y[1]-y[0],2));
		da[1][2] = sqrt(pow(x[1]-x[2],2) + pow(y[1]-y[2],2));
		da[1][3] = sqrt(pow(x[1]-x[3],2) + pow(y[1]-y[3],2));
		da[2][0] = sqrt(pow(x[2]-x[0],2) + pow(y[2]-y[0],2));
		da[2][1] = sqrt(pow(x[2]-x[1],2) + pow(y[2]-y[1],2));
		da[2][3] = sqrt(pow(x[2]-x[3],2) + pow(y[2]-y[3],2));
		da[3][0] = sqrt(pow(x[3]-x[0],2) + pow(y[3]-y[0],2));
		da[3][1] = sqrt(pow(x[3]-x[1],2) + pow(y[3]-y[1],2));
		da[3][2] = sqrt(pow(x[3]-x[2],2) + pow(y[3]-y[2],2));
	
		// Centroid of formation as the middle between all robots
		xcbar0 = 0.25*(x[0] + x[1] + x[2] + x[3]) + centroid_x; 
		ycbar0 = 0.25*(y[0] + y[1] + y[2] + y[3]) + centroid_y;
		
		a += radius;
		
		if(a > 1.5*500) {
			a = 1.5*500;
		} else if( a < 0.75*500 ) {
			a = 0.75*500;
		}
		
		deltax[0] = a;
		deltax[1] = 0;
		deltax[2] = -a;
		deltax[3] = 0;

		deltay[0] = 0;
		deltay[1] = a;
		deltay[2] = 0;
		deltay[3] = -a;
		
		//xcbar0 = 1250*cos(t/4.0) + 300;
		//ycbar0 = 1250*sin(t/4.0);
		
		double v = 0.0; // linear velocity
		double w = 0.0; // angular velocity
		double ex = x[id] - xcbar0 - deltax[id];
		double ey = y[id] - ycbar0 - deltay[id];
		double dVdx[4] = {0.0};
		double dVdy[4] = {0.0};
		
		for (int j = 0; j < 4; j++) {
				if( (da[id][j] > r) && (da[id][j] < R)  ){
					dVdx[j] = 4*(pow(R,2)-pow(r,2)) * (pow(da[id][j],2)-pow(R,2)) / pow(pow(da[id][j],2)-pow(r,2),3) * (x[id]-x[j]);
					dVdy[j] = 4*(pow(R,2)-pow(r,2)) * (pow(da[id][j],2)-pow(R,2)) / pow(pow(da[id][j],2)-pow(r,2),3) * (y[id]-y[j]);
				}
		}
		
		double Ex = ex + dVdx[0] + dVdx[1] + dVdx[2] + dVdx[3];
		double Ey = ey + dVdy[0] + dVdy[1] + dVdy[2] + dVdy[3];
		thetad = atan2(Ey,Ex);
		
		if ( !firstExecutiond) {
			int m = thetad_loop;
			if((-thetad + thetad_old )> PI/2){
	   			m = m+1;
	   		} else if((thetad - thetad_old ) > PI/2){
				m = m-1;
			}
			thetad_old = thetad;
			thetad_out = thetad + 2 * m * PI;
			
			thetad_loop = m;
			
		} else {
			thetad_old = thetad;
			firstExecutiond = false;
		}
		
		if((-theta[id] + thetad )> PI*2){
	   			thetad_loop--;
	   	} /*else if((theta[id] - thetad ) > PI*2){
				theta_loop--;
		}
			*/
		double etheta = theta_out - thetad_out;
		
		//if(theta[id] < 0.0){
			v = -k*cos(etheta)*sqrt(pow(Ex,2)+pow(Ey,2));
			w = -ktheta*(etheta+PI);
		//}
		//else{
		//	v = -k*cos(etheta)*sqrt(pow(Ex,2)+pow(Ey,2));
		//	w = -ktheta*(etheta-PI);
		//}
		
		// convert to wheel velocities
		//double lwv = -v - rho * w;
		//double rwv = -v + rho * w;
//		lwv = -lwv;
//		rwv = -rwv;
		if ( power ) {
		// Send wheel velocities to driver
			geometry_msgs::Twist msg;
			msg.linear.x = v;
			msg.linear.y = 0;
			msg.linear.z = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;
			msg.angular.z = w;
			vel.publish(msg);
		} else {
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
		create_controller::ControlMsgs dmsg; 
		dmsg.theta_old = theta_old;
		dmsg.thetad_old = thetad_old;
		dmsg.theta = theta[id];
		dmsg.thetad = thetad;
		dmsg.etheta = theta[id]-thetad+PI;
		dia_pub.publish(dmsg);
		
		ros::spinOnce(); 

		loop_rate.sleep();
	}
	
}
