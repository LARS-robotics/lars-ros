
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "connectivity_controller_old/Connectivity_oldMsgs.h"
#include <math.h>
#include <algorithm>
#include "ros/console.h"
#include "stdio.h"
//#include <LinearMath/btQuaternion.h>
//#include <LinearMath/btMatrix3x3.h>
#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <ros/ros.h>

using namespace Eigen;

 

std::string robot1_name;
std::string robot2_name;
std::string robot3_name;
std::string robot4_name;
std::string robot5_name;
std::string robot6_name;

#define rho1 0.7
#define rho2 2.3

double x[6] = {0.0}; 
double y[6] = {0.0};
double theta[6] = {0.0};


double psi( double x )
{
	if ( x <= rho1 ) {
		return 1;
	} else if ( rho1 < x && x < rho2 ) {
		return ( exp(-1/(rho2-x)) / ( exp(-1/(rho2-x)) + exp(1/(rho1-x)) ) );
	} else if ( rho2 <= x ) {
		return 0;
	}
	return 0;
}
	
double psiPrime( double d )
{
	if ( d <= rho1 ) {
		return 0;
	} else if ( rho1 < d && d < rho2 ) {
		return (-0.5)*( pow((d-rho1),(-2))+pow((d-rho2),(-2)) ) / (1+  cosh(  (1/(-d+rho1))+(1/(-d+rho2)) )   );
	} else if ( rho2 <= d ) {
		return 0;
	}
	return 0;
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
	
	// get rotation
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control");

	ros::NodeHandle n;
	ros::NodeHandle private_n("~");
	

	
	private_n.param<std::string>("robot1name", robot1_name, "gandhi");
	private_n.param<std::string>("robot2name", robot2_name, "suleyman");
	private_n.param<std::string>("robot3name", robot3_name, "alexander");
	private_n.param<std::string>("robot4name", robot4_name, "hiawatha");
	private_n.param<std::string>("robot5name", robot5_name, "darius");
	private_n.param<std::string>("robot6name", robot6_name, "caesar");

 	
	ros::Subscriber subzero  = n.subscribe(  robot1_name + "/odom", 10, Callbackzero);
	ros::Subscriber subone   = n.subscribe(  robot2_name + "/odom", 10, Callbackone);
	ros::Subscriber subtwo   = n.subscribe(  robot3_name + "/odom", 10, Callbacktwo);
	ros::Subscriber subthree = n.subscribe(  robot4_name + "/odom", 10, Callbackthree);
	ros::Subscriber subfour  = n.subscribe(  robot5_name + "/odom", 10, Callbackfour);
	ros::Subscriber subfive  = n.subscribe(  robot6_name + "/odom", 10, Callbackfive);
	
	ros::Publisher pub = n.advertise<connectivity_controller_old::Connectivity_oldMsgs>("/connectivity", 10);
	
	ros::Rate loop_rate(100); // 100 Hz
	
	while (ros::ok())
	{
		double adjMatrix[6][6] = {{0.0}};
		double P_N1[1]    = {0.0};
		double P_N2[2][1] = {{-0.707106781186547}, 
							 { 0.707106781186547}};
		double P_N3[3][2] = {{-0.577350269189626, -0.577350269189626}, 
							 { 0.788675134594813, -0.211324865405187}, 
							 {-0.211324865405187,  0.788675134594813}};
		double P_N4[4][3] = {{-0.5,               -0.5,               -0.5}, 
							 { 0.833333333333333, -0.166666666666667, -0.166666666666667}, 
							 {-0.166666666666667,  0.833333333333333, -0.166666666666667}, 
							 {-0.166666666666667, -0.166666666666667,  0.833333333333333}};
		double P_N6[6][5] =		{{1/sqrt(2), 	-1/sqrt(6.0), -1/2.0/sqrt(3), 	-1/2.0/sqrt(5), 	-1/sqrt(30)},
								{0.0,        	sqrt(2.0/3),  -1/2.0/sqrt(3), 	-1/2.0/sqrt(5), 	-1/sqrt(30)},
								{0.0, 		 	0.0, 		sqrt(3.0)/2,    	-1/(2*sqrt(5)), -1/sqrt(30)},
								{0.0, 			0.0, 		0.0, 			2/sqrt(5), 		-1/sqrt(30)},
								{0.0, 			0.0, 		0.0,			0.0, 			sqrt(5.0/6)},
								{-1/sqrt(2), 	-1/sqrt(6), -1/2.0/sqrt(3), 	-1/2.0/sqrt(5), 	-1/sqrt(30)}};
		
		MatrixXd L(6,6);
		MatrixXd P(6,5);
		MatrixXd M(5,5);
	
		for (int i = 0; i < 6; i++) 
		{
			for (int j = 0; j < 6; j++)
			{
				adjMatrix[i][j] = psi( sqrt(pow(x[j]-x[i], 2) + pow(y[j]-y[i], 2)) );
			}
		}
		
		for (int i = 0; i < 6; i++) 
		{
			for (int j = 0; j < 6; j++)
			{
				if ( i == j ) {
					double rowSum = 0;
					for (int k = 0; k < 6; k++) {
						rowSum += adjMatrix[i][k];
					}
					L(i,j) = rowSum - adjMatrix[i][j];
				} else {
					L(i,j) = -adjMatrix[i][j];
				}
			}
		}
		
		P(0,0) = P_N6[0][0];
		P(0,1) = P_N6[0][1];
		P(0,2) = P_N6[0][2];
		P(0,3) = P_N6[0][3];
		P(0,4) = P_N6[0][4];
		P(1,0) = P_N6[1][0];
		P(1,1) = P_N6[1][1];
		P(1,2) = P_N6[1][2];
		P(1,3) = P_N6[1][3];
		P(1,4) = P_N6[1][4];
		P(2,0) = P_N6[2][0];
		P(2,1) = P_N6[2][1];
		P(2,2) = P_N6[2][2];
		P(2,3) = P_N6[2][3];
		P(2,4) = P_N6[2][4];
		P(3,0) = P_N6[3][0];
		P(3,1) = P_N6[3][1];
		P(3,2) = P_N6[3][2];
		P(3,3) = P_N6[3][3];
		P(3,4) = P_N6[3][4];
		P(4,0) = P_N6[4][0];
		P(4,1) = P_N6[4][1];
		P(4,2) = P_N6[4][2];
		P(4,3) = P_N6[4][3];
		P(4,4) = P_N6[4][4];
		P(5,0) = P_N6[5][0];
		P(5,1) = P_N6[5][1];
		P(5,2) = P_N6[5][2];
		P(5,3) = P_N6[5][3];
		P(5,4) = P_N6[5][4];
		
		M = P.transpose() * L * P;
		
		MatrixXd dLdx_0(6,6);
		MatrixXd dLdx_1(6,6);
		MatrixXd dLdx_2(6,6);
		MatrixXd dLdx_3(6,6);
		MatrixXd dLdx_4(6,6);
		MatrixXd dLdx_5(6,6);
		
		MatrixXd dLdy_0(6,6);
		MatrixXd dLdy_1(6,6);
		MatrixXd dLdy_2(6,6);
		MatrixXd dLdy_3(6,6);
		MatrixXd dLdy_4(6,6);
		MatrixXd dLdy_5(6,6);
		
		dLdx_0 << 0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0;
				  
		dLdx_1 << 0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0;
				  
		dLdx_2 << 0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0;
				  
		dLdx_3 << 0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0;
				  
		dLdx_4 << 0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0;
				  
		dLdx_5 << 0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0;
				  
		dLdy_0 << 0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0;
				  
		dLdy_1 << 0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0;
				  
		dLdy_2 << 0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0;
				  
		dLdy_3 << 0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0;
				  
		dLdy_4 << 0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0;
				  
		dLdy_5 << 0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0,
				  0,0,0,0,0,0;				  
				  
		
		MatrixXd dMdx_0(5,5);
		MatrixXd dMdx_1(5,5);
		MatrixXd dMdx_2(5,5);
		MatrixXd dMdx_3(5,5);
		MatrixXd dMdx_4(5,5);
		MatrixXd dMdx_5(5,5);
		
		MatrixXd dMdy_0(5,5);
		MatrixXd dMdy_1(5,5);
		MatrixXd dMdy_2(5,5);
		MatrixXd dMdy_3(5,5);
		MatrixXd dMdy_4(5,5);
		MatrixXd dMdy_5(5,5);
		
			
		dMdx_0 << 0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0;
				  
		dMdx_1 << 0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0;
				  
		dMdx_2 << 0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0;
				  		
		dMdx_3 << 0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0;
				  
		dMdx_4 << 0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0;
				  		
		dMdx_5 << 0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0;
				  
		dMdy_0 << 0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0;
				  
		dMdy_1 << 0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0;	
		
		dMdy_2 << 0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0;
				  		
		dMdy_3 << 0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0;
				  
		dMdy_4 << 0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0;
				  		
		dMdy_5 << 0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0,
				  0,0,0,0,0;
				  
		MatrixXd CAMatrix_x(6,6);		
		MatrixXd CAMatrix_y(6,6);
						  	
		CAMatrix_x << 	0,0,0,0,0,0,
				  		0,0,0,0,0,0,
				  		0,0,0,0,0,0,
				  		0,0,0,0,0,0,
				  		0,0,0,0,0,0,
				  		0,0,0,0,0,0;	

		CAMatrix_y << 	0,0,0,0,0,0,
				  		0,0,0,0,0,0,
				  		0,0,0,0,0,0,
				  		0,0,0,0,0,0,
				  		0,0,0,0,0,0,
				  		0,0,0,0,0,0;
				 		
		double u_ca_x[6]={0.0};				 		 
		double u_ca_y[6]={0.0};				 		 
		
		double u_fc_x[6]={0.0};				 		 
		double u_fc_y[6]={0.0};	

		double xd[6]={ 0.5457, 1.9729, 1.237,-0.7, -0.3, -0.9};	
		double yd[6]={-1.5740,-1.5810,-1.539,-0.7, -1.54, -1.54};
								
		double R = 	0.7;	  
		double r =  0.3300;
		
		double whatever = 0.0;

		
		// Collision Avoidance for each robot starts here:
		for(int i = 0; i<6; i++)
		{
			for (int j=i+1; j<6; j++) 
			{
				double D = sqrt( pow(x[i] - x[j], 2) + pow(y[i] - y[j], 2));
				if(D<R && D>r){
					CAMatrix_x(i,j) = -4*(pow(R,2)-pow(r,2))*(pow(D,2)-pow(R,2))/pow((pow(D,2)-pow(r,2)),3)*(x[i]-x[j]);
		        	CAMatrix_y(i,j) = -4*(pow(R,2)-pow(r,2))*(pow(D,2)-pow(R,2))/pow((pow(D,2)-pow(r,2)),3)*(y[i]-y[j]);
            	}
            	else{
            		CAMatrix_x(i,j) = 0;
            		CAMatrix_y(i,j) = 0;
            	}
            	CAMatrix_x(j,i)=-CAMatrix_x(i,j);
	       		CAMatrix_y(j,i)=-CAMatrix_y(i,j);
			}
		}
		
		for(int i = 0; i < 6; i++)
		{
			double sumx = 0;
			double sumy = 0;
			for (int j=0; j<6; j++) 
			{
				sumx = sumx + CAMatrix_x(i,j);
				sumy = sumy + CAMatrix_y(i,j);
			}
			u_ca_x[i] = sumx;
			u_ca_y[i] = sumy;
			// sneak in the formation control
			u_fc_x[i] = -(x[i]-xd[i]);
			u_fc_y[i] = -(y[i]-yd[i]);
		}
		
		// Collision Avoidance ends here
			
		// Calculate dLdx and dLdy for each robot 0 to 3
		for(int i = 0; i<6; i++)
		{
			for (int j=i+1; j<6; j++) 
			{
				double D = sqrt( pow(x[i] - x[j], 2) + pow(y[i] - y[j], 2));
				double dAdD = psiPrime(D);
				std::cerr << D << std::endl << std::endl;
				
				whatever =x[i] - x[j];
				
				if( 0 == i ) {
					double dDdx = (x[i]-x[j]) / D;							
					double dDdy = (y[i]-y[j]) / D;
							
					dLdx_0(i,j) = dAdD*dDdx;
            		dLdy_0(i,j) = dAdD*dDdy;        		
            		
				}
				
				if( 0 == j ) {
					double dDdx = -(x[i]-x[j]) /D;							
					double dDdy = -(y[i]-y[j]) / D;
							
					dLdx_0(i,j) = dAdD*dDdx;
            		dLdy_0(i,j) = dAdD*dDdy;
                  		
				}
				
           	
            	dLdx_0(j,i) = dLdx_0(i,j);
            	dLdy_0(j,i) = dLdy_0(i,j);
            
			}
		}
		
		for(int i = 0; i < 6; i++)
		{
			double sumx = 0;
			double sumy = 0;
			for(int j = 0; j<6; j++)
			{
				sumx += dLdx_0(i,j);
				sumy += dLdy_0(i,j);
			} 
			dLdx_0(i,i) = -sumx;
            dLdy_0(i,i) = -sumy;
		}
		
		for(int i = 0; i<6; i++)
		{
			for (int j=i+1; j<6; j++) 
			{
				double D = sqrt( pow(x[i] - x[j], 2) + pow(y[i] - y[j], 2));
				double dAdD = psiPrime(D);
				
				if( 1 == i ) {
					double dDdx = (x[i]-x[j]) / D;							
					double dDdy = (y[i]-y[j]) / D;
							
					dLdx_1(i,j) = dAdD*dDdx;
            		dLdy_1(i,j) = dAdD*dDdy;
				}
				
				if( 1 == j ) {
					double dDdx = -(x[i]-x[j]) / D;							
					double dDdy = -(y[i]-y[j]) / D;
							
					dLdx_1(i,j) = dAdD*dDdx;
            		dLdy_1(i,j) = dAdD*dDdy;
				}
				
            	
            	dLdx_1(j,i) = dLdx_1(i,j);
            	dLdy_1(j,i) = dLdy_1(i,j);
			}
		}
		
		for(int i = 0; i < 6; i++)
		{
			double sumx = 0;
			double sumy = 0;
			for(int j = 0; j<6; j++)
			{
				sumx += dLdx_1(i,j);
				sumy += dLdy_1(i,j);
			} 
			dLdx_1(i,i) = -sumx;
            dLdy_1(i,i) = -sumy;
		}
		
		for(int i = 0; i<6; i++)
		{
			for (int j=i+1; j<6; j++) 
			{
				double D = sqrt( pow(x[i] - x[j], 2) + pow(y[i] - y[j], 2));
				double dAdD = psiPrime(D);

				
				if( 2 == i ) {
					double dDdx = (x[i]-x[j]) / D;							
					double dDdy = (y[i]-y[j]) / D;
							
					dLdx_2(i,j) = dAdD*dDdx;
            		dLdy_2(i,j) = dAdD*dDdy;
				}
				
				if( 2 == j ) {
					double dDdx = -(x[i]-x[j]) / D;							
					double dDdy = -(y[i]-y[j]) / D;
							
					dLdx_2(i,j) = dAdD*dDdx;
            		dLdy_2(i,j) = dAdD*dDdy;
				}
				
            	
            	dLdx_2(j,i) = dLdx_2(i,j);
            	dLdy_2(j,i) = dLdy_2(i,j);
			}
		}
		
		for(int i = 0; i < 6; i++)
		{
			double sumx = 0;
			double sumy = 0;
			for(int j = 0; j<6; j++)
			{
				sumx += dLdx_2(i,j);
				sumy += dLdy_2(i,j);
			} 
			dLdx_2(i,i) = -sumx;
            dLdy_2(i,i) = -sumy;
		}
		
		for(int i = 0; i<6; i++)
		{
			for (int j=i+1; j<6; j++) 
			{
				double D = sqrt( pow(x[i] - x[j], 2) + pow(y[i] - y[j], 2));
				double dAdD = psiPrime(D);
				
				if( 3 == i ) {
					double dDdx = (x[i]-x[j]) / D;							
					double dDdy = (y[i]-y[j]) / D;
							
					dLdx_3(i,j) = dAdD*dDdx;
            		dLdy_3(i,j) = dAdD*dDdy;
				}
				
				if( 3 == j ) {
					double dDdx = -(x[i]-x[j]) / D;							
					double dDdy = -(y[i]-y[j]) / D;
							
					dLdx_3(i,j) = dAdD*dDdx;
            		dLdy_3(i,j) = dAdD*dDdy;
				}
				
            	
            	dLdx_3(j,i) = dLdx_3(i,j);
            	dLdy_3(j,i) = dLdy_3(i,j);
			}
		}
		
		for(int i = 0; i < 6; i++)
		{
			double sumx = 0;
			double sumy = 0;
			for(int j = 0; j<6; j++)
			{
				sumx += dLdx_3(i,j);
				sumy += dLdy_3(i,j);
			} 
			dLdx_3(i,i) = -sumx;
            dLdy_3(i,i) = -sumy;
		}
		
		for(int i = 0; i<6; i++)
		{
			for (int j=i+1; j<6; j++) 
			{
				double D = sqrt( pow(x[i] - x[j], 2) + pow(y[i] - y[j], 2));
				double dAdD = psiPrime(D);

				
				if( 4 == i ) {
					double dDdx = (x[i]-x[j]) / D;							
					double dDdy = (y[i]-y[j]) / D;
							
					dLdx_4(i,j) = dAdD*dDdx;
            		dLdy_4(i,j) = dAdD*dDdy;
				}
				
				if( 4 == j ) {
					double dDdx = -(x[i]-x[j]) / D;							
					double dDdy = -(y[i]-y[j]) / D;
							
					dLdx_4(i,j) = dAdD*dDdx;
            		dLdy_4(i,j) = dAdD*dDdy;
				}
				
            	
            	dLdx_4(j,i) = dLdx_4(i,j);
            	dLdy_4(j,i) = dLdy_4(i,j);
			}
		}
		
		for(int i = 0; i < 6; i++)
		{
			double sumx = 0;
			double sumy = 0;
			for(int j = 0; j<6; j++)
			{
				sumx += dLdx_4(i,j);
				sumy += dLdy_4(i,j);
			} 
			dLdx_4(i,i) = -sumx;
            dLdy_4(i,i) = -sumy;
		}
		
		for(int i = 0; i<6; i++)
		{
			for (int j=i+1; j<6; j++) 
			{
				double D = sqrt( pow(x[i] - x[j], 2) + pow(y[i] - y[j], 2));
				double dAdD = psiPrime(D);
				
				if( 5 == i ) {
					double dDdx = (x[i]-x[j]) / D;							
					double dDdy = (y[i]-y[j]) / D;
							
					dLdx_5(i,j) = dAdD*dDdx;
            		dLdy_5(i,j) = dAdD*dDdy;
				}
				
				if( 5 == j ) {
					double dDdx = -(x[i]-x[j]) / D;							
					double dDdy = -(y[i]-y[j]) / D;
							
					dLdx_5(i,j) = dAdD*dDdx;
            		dLdy_5(i,j) = dAdD*dDdy;
				}
				
            	
            	dLdx_5(j,i) = dLdx_5(i,j);
            	dLdy_5(j,i) = dLdy_5(i,j);
			}
		}
		
		for(int i = 0; i < 6; i++)
		{
			double sumx = 0;
			double sumy = 0;
			for(int j = 0; j<6; j++)
			{
				sumx += dLdx_5(i,j);
				sumy += dLdy_5(i,j);
			} 
			dLdx_5(i,i) = -sumx;
            dLdy_5(i,i) = -sumy;
		}

		
		dMdx_0 = P.transpose() * dLdx_0 * P;
		dMdx_1 = P.transpose() * dLdx_1 * P;
		dMdx_2 = P.transpose() * dLdx_2 * P;
		dMdx_3 = P.transpose() * dLdx_3 * P;
		dMdx_4 = P.transpose() * dLdx_4 * P;
		dMdx_5 = P.transpose() * dLdx_5 * P;
		
		dMdy_0 = P.transpose() * dLdy_0 * P;
		dMdy_1 = P.transpose() * dLdy_1 * P;
		dMdy_2 = P.transpose() * dLdy_2 * P;
		dMdy_3 = P.transpose() * dLdy_3 * P;
		dMdy_4 = P.transpose() * dLdy_4 * P;
		dMdy_5 = P.transpose() * dLdy_5 * P;	
		
		double trMx_0 = (M.inverse()*dMdx_0).trace();
		double trMx_1 = (M.inverse()*dMdx_1).trace();
		double trMx_2 = (M.inverse()*dMdx_2).trace();
		double trMx_3 = (M.inverse()*dMdx_3).trace();
		double trMx_4 = (M.inverse()*dMdx_4).trace();
		double trMx_5 = (M.inverse()*dMdx_5).trace();
		
		double trMy_0 = (M.inverse()*dMdy_0).trace();
		double trMy_1 = (M.inverse()*dMdy_1).trace();
		double trMy_2 = (M.inverse()*dMdy_2).trace();
		double trMy_3 = (M.inverse()*dMdy_3).trace();
		double trMy_4 = (M.inverse()*dMdy_4).trace();
		double trMy_5 = (M.inverse()*dMdy_5).trace();
		
		double detM = M.determinant();
		
		connectivity_controller_old::Connectivity_oldMsgs msg; 

		msg.trMx[0] = trMx_0;
		msg.trMx[1] = trMx_1;
		msg.trMx[2] = trMx_2;
		msg.trMx[3] = trMx_3;
		msg.trMx[4] = trMx_4;
		msg.trMx[5] = trMx_5;
		
		msg.dMdy0[0] = dMdy_4(0,0);
		msg.dMdy0[1] = dMdy_4(0,1);
		msg.dMdy0[2] = dMdy_4(0,2);
		msg.dMdy0[3] = dMdy_4(0,3);
		msg.dMdy0[4] = dMdy_4(0,4);
		
		msg.dMdy1[0] = dMdy_4(1,0);
		msg.dMdy1[1] = dMdy_4(1,1);
		msg.dMdy1[2] = dMdy_4(1,2);
		msg.dMdy1[3] = dMdy_4(1,3);
		msg.dMdy1[4] = dMdy_4(1,4);
		
		msg.dMdy2[0] = dMdy_4(2,0);
		msg.dMdy2[1] = dMdy_4(2,1);
		msg.dMdy2[2] = dMdy_4(2,2);
		msg.dMdy2[3] = dMdy_4(2,3);
		msg.dMdy2[4] = dMdy_4(2,4);
		
		msg.dMdy3[0] = dMdy_4(3,0);
		msg.dMdy3[1] = dMdy_4(3,1);
		msg.dMdy3[2] = dMdy_4(3,2);
		msg.dMdy3[3] = dMdy_4(3,3);
		msg.dMdy3[4] = dMdy_4(3,4);
		
		msg.dMdy4[0] = dMdy_4(4,0);
		msg.dMdy4[1] = dMdy_4(4,1);
		msg.dMdy4[2] = dMdy_4(4,2);
		msg.dMdy4[3] = dMdy_4(4,3);
		msg.dMdy4[4] = dMdy_4(4,4);
		
		msg.trMy[0] = trMy_0;
		msg.trMy[1] = trMy_1;
		msg.trMy[2] = trMy_2;
		msg.trMy[3] = trMy_3;
		msg.trMy[4] = trMy_4;
		msg.trMy[5] = trMy_5;
		
		msg.detM = detM;
		
		msg.M0[0] = M(0,0);
		msg.M0[1] = M(0,1);
		msg.M0[2] = M(0,2);
		msg.M0[3] = M(0,3);
		msg.M0[4] = M(0,4);
		
		msg.M1[0] = M(1,0);
		msg.M1[1] = M(1,1);
		msg.M1[2] = M(1,2);
		msg.M1[3] = M(1,3);
		msg.M1[4] = M(1,4);
		
		msg.M2[0] = M(2,0);
		msg.M2[1] = M(2,1);
		msg.M2[2] = M(2,2);
		msg.M2[3] = M(2,3);
		msg.M2[4] = M(2,4);
		
		msg.M3[0] = M(3,0);
		msg.M3[1] = M(3,1);
		msg.M3[2] = M(3,2);
		msg.M3[3] = M(3,3);
		msg.M3[4] = M(3,4);
		
		msg.M4[0] = M(4,0);
		msg.M4[1] = M(4,1);
		msg.M4[2] = M(4,2);
		msg.M4[3] = M(4,3);
		msg.M4[4] = M(4,4);
	
		msg.L0[0] = L(0,0);
		msg.L0[1] = L(0,1);
		msg.L0[2] = L(0,2);
		msg.L0[3] = L(0,3);
		msg.L0[4] = L(0,4);
		msg.L0[5] = L(0,5);
		
		msg.L1[0] = L(1,0);
		msg.L1[1] = L(1,1);
		msg.L1[2] = L(1,2);
		msg.L1[3] = L(1,3);
		msg.L1[4] = L(1,4);
		msg.L1[5] = L(1,5);
		
		msg.L2[0] = L(2,0);
		msg.L2[1] = L(2,1);
		msg.L2[2] = L(2,2);
		msg.L2[3] = L(2,3);
		msg.L2[4] = L(2,4);
		msg.L2[5] = L(2,5);
		
		msg.L3[0] = L(3,0);
		msg.L3[1] = L(3,1);
		msg.L3[2] = L(3,2);
		msg.L3[3] = L(3,3);
		msg.L3[4] = L(3,4);
		msg.L3[5] = L(3,5);
		
		msg.L4[0] = L(4,0);
		msg.L4[1] = L(4,1);
		msg.L4[2] = L(4,2);
		msg.L4[3] = L(4,3);
		msg.L4[4] = L(4,4);
		msg.L4[5] = L(4,5);
		
		msg.L5[0] = L(5,0);
		msg.L5[1] = L(5,1);
		msg.L5[2] = L(5,2);
		msg.L5[3] = L(5,3);
		msg.L5[4] = L(5,4);
		msg.L5[5] = L(5,5);
		
		msg.dLdx0[0] = dLdx_4(0,0);
		msg.dLdx0[1] = dLdx_4(0,1);
		msg.dLdx0[2] = dLdx_4(0,2);
		msg.dLdx0[3] = dLdx_4(0,3);
		msg.dLdx0[4] = dLdx_4(0,4);
		msg.dLdx0[5] = dLdx_4(0,5);
		
		msg.dLdx1[0] = dLdx_4(1,0);
		msg.dLdx1[1] = dLdx_4(1,1);
		msg.dLdx1[2] = dLdx_4(1,2);
		msg.dLdx1[3] = dLdx_4(1,3);
		msg.dLdx1[4] = dLdx_4(1,4);
		msg.dLdx1[5] = dLdx_4(1,5);
		
		msg.dLdx2[0] = dLdx_4(2,0);
		msg.dLdx2[1] = dLdx_4(2,1);
		msg.dLdx2[2] = dLdx_4(2,2);
		msg.dLdx2[3] = dLdx_4(2,3);
		msg.dLdx2[4] = dLdx_4(2,4);
		msg.dLdx2[5] = dLdx_4(2,5);
		
		msg.dLdx3[0] = dLdx_4(3,0);
		msg.dLdx3[1] = dLdx_4(3,1);
		msg.dLdx3[2] = dLdx_4(3,2);
		msg.dLdx3[3] = dLdx_4(3,3);
		msg.dLdx3[4] = dLdx_4(3,4);
		msg.dLdx3[5] = dLdx_4(3,5);
		
		msg.dLdx4[0] = dLdx_4(4,0);
		msg.dLdx4[1] = dLdx_4(4,1);
		msg.dLdx4[2] = dLdx_4(4,2);
		msg.dLdx4[3] = dLdx_4(4,3);
		msg.dLdx4[4] = dLdx_4(4,4);
		msg.dLdx4[5] = dLdx_4(4,5);
		
		msg.dLdx5[0] = dLdx_4(5,0);
		msg.dLdx5[1] = dLdx_4(5,1);
		msg.dLdx5[2] = dLdx_4(5,2);
		msg.dLdx5[3] = dLdx_4(5,3);
		msg.dLdx5[4] = dLdx_4(5,4);
		msg.dLdx5[5] = dLdx_4(5,5);
		
		msg.u_ca_x[0] = u_ca_x[0];
		msg.u_ca_x[1] = u_ca_x[1];
		msg.u_ca_x[2] = u_ca_x[2];
		msg.u_ca_x[3] = u_ca_x[3];
		msg.u_ca_x[4] = u_ca_x[4];
		msg.u_ca_x[5] = u_ca_x[5];
	
		msg.u_ca_y[0] = u_ca_y[0];
		msg.u_ca_y[1] = u_ca_y[1];
		msg.u_ca_y[2] = u_ca_y[2];
		msg.u_ca_y[3] = u_ca_y[3];
		msg.u_ca_y[4] = u_ca_y[4];
		msg.u_ca_y[5] = u_ca_y[5];
		
		msg.u_fc_x[0] = u_fc_x[0];
		msg.u_fc_x[1] = u_fc_x[1];
		msg.u_fc_x[2] = u_fc_x[2];
		msg.u_fc_x[3] = u_fc_x[3];
		msg.u_fc_x[4] = u_fc_x[4];
		msg.u_fc_x[5] = u_fc_x[5];
	
		msg.u_fc_y[0] = u_fc_y[0];
		msg.u_fc_y[1] = u_fc_y[1];
		msg.u_fc_y[2] = u_fc_y[2];
		msg.u_fc_y[3] = u_fc_y[3];
		msg.u_fc_y[4] = u_fc_y[4];
		msg.u_fc_y[5] = u_fc_y[5];
		
		msg.CAMatrix_x_0[0]  = CAMatrix_x(0,0);
		msg.CAMatrix_x_0[1]  = CAMatrix_x(0,1);
		msg.CAMatrix_x_0[2]  = CAMatrix_x(0,2);
		msg.CAMatrix_x_0[3]  = CAMatrix_x(0,3);
		msg.CAMatrix_x_0[4]  = CAMatrix_x(0,2);
		msg.CAMatrix_x_0[5]  = CAMatrix_x(0,3);
		
		msg.CAMatrix_x_1[0]  = CAMatrix_x(1,0);
		msg.CAMatrix_x_1[1]  = CAMatrix_x(1,1);
		msg.CAMatrix_x_1[2]  = CAMatrix_x(1,2);
		msg.CAMatrix_x_1[3]  = CAMatrix_x(1,3);
		msg.CAMatrix_x_1[4]  = CAMatrix_x(1,4);
		msg.CAMatrix_x_1[5]  = CAMatrix_x(1,5);
		
		msg.CAMatrix_x_2[0]  = CAMatrix_x(2,0);
		msg.CAMatrix_x_2[1]  = CAMatrix_x(2,1);
		msg.CAMatrix_x_2[2]  = CAMatrix_x(2,2);
		msg.CAMatrix_x_2[3]  = CAMatrix_x(2,3);
		msg.CAMatrix_x_2[4]  = CAMatrix_x(2,4);
		msg.CAMatrix_x_2[5]  = CAMatrix_x(2,5);
		
		msg.CAMatrix_x_3[0]  = CAMatrix_x(3,0);
		msg.CAMatrix_x_3[1]  = CAMatrix_x(3,1);
		msg.CAMatrix_x_3[2]  = CAMatrix_x(3,2);
		msg.CAMatrix_x_3[3]  = CAMatrix_x(3,3);
		msg.CAMatrix_x_3[4]  = CAMatrix_x(3,4);
		msg.CAMatrix_x_3[5]  = CAMatrix_x(3,5);
		
		msg.CAMatrix_x_4[0]  = CAMatrix_x(4,0);
		msg.CAMatrix_x_4[1]  = CAMatrix_x(4,1);
		msg.CAMatrix_x_4[2]  = CAMatrix_x(4,2);
		msg.CAMatrix_x_4[3]  = CAMatrix_x(4,3);
		msg.CAMatrix_x_4[4]  = CAMatrix_x(4,4);
		msg.CAMatrix_x_4[5]  = CAMatrix_x(4,5);
		
		msg.CAMatrix_x_5[0]  = CAMatrix_x(5,0);
		msg.CAMatrix_x_5[1]  = CAMatrix_x(5,1);
		msg.CAMatrix_x_5[2]  = CAMatrix_x(5,2);
		msg.CAMatrix_x_5[3]  = CAMatrix_x(5,3);
		msg.CAMatrix_x_5[4]  = CAMatrix_x(5,4);
		msg.CAMatrix_x_5[5]  = CAMatrix_x(5,5);
		
		msg.CAMatrix_y_0[0]  = CAMatrix_y(0,0);
		msg.CAMatrix_y_0[1]  = CAMatrix_y(0,1);
		msg.CAMatrix_y_0[2]  = CAMatrix_y(0,2);
		msg.CAMatrix_y_0[3]  = CAMatrix_y(0,3);
		msg.CAMatrix_y_0[4]  = CAMatrix_y(0,4);
		msg.CAMatrix_y_0[5]  = CAMatrix_y(0,5);
		
		msg.CAMatrix_y_1[0]  = CAMatrix_y(1,0);
		msg.CAMatrix_y_1[1]  = CAMatrix_y(1,1);
		msg.CAMatrix_y_1[2]  = CAMatrix_y(1,2);
		msg.CAMatrix_y_1[3]  = CAMatrix_y(1,3);
		msg.CAMatrix_y_1[4]  = CAMatrix_y(1,4);
		msg.CAMatrix_y_1[5]  = CAMatrix_y(1,5);
		
		msg.CAMatrix_y_2[0]  = CAMatrix_y(2,0);
		msg.CAMatrix_y_2[1]  = CAMatrix_y(2,1);
		msg.CAMatrix_y_2[2]  = CAMatrix_y(2,2);
		msg.CAMatrix_y_2[3]  = CAMatrix_y(2,3);
		msg.CAMatrix_y_2[4]  = CAMatrix_y(2,4);
		msg.CAMatrix_y_2[5]  = CAMatrix_y(2,5);
		
		msg.CAMatrix_y_3[0]  = CAMatrix_y(3,0);
		msg.CAMatrix_y_3[1]  = CAMatrix_y(3,1);
		msg.CAMatrix_y_3[2]  = CAMatrix_y(3,2);
		msg.CAMatrix_y_3[3]  = CAMatrix_y(3,3);
		msg.CAMatrix_y_3[4]  = CAMatrix_y(3,4);
		msg.CAMatrix_y_3[5]  = CAMatrix_y(3,5);
		
		msg.CAMatrix_y_4[0]  = CAMatrix_y(4,0);
		msg.CAMatrix_y_4[1]  = CAMatrix_y(4,1);
		msg.CAMatrix_y_4[2]  = CAMatrix_y(4,2);
		msg.CAMatrix_y_4[3]  = CAMatrix_y(4,3);
		msg.CAMatrix_y_4[4]  = CAMatrix_y(4,4);
		msg.CAMatrix_y_4[5]  = CAMatrix_y(4,5);
		
		msg.CAMatrix_y_5[0]  = CAMatrix_y(5,0);
		msg.CAMatrix_y_5[1]  = CAMatrix_y(5,1);
		msg.CAMatrix_y_5[2]  = CAMatrix_y(5,2);
		msg.CAMatrix_y_5[3]  = CAMatrix_y(5,3);
		msg.CAMatrix_y_5[4]  = CAMatrix_y(5,4);
		msg.CAMatrix_y_5[5]  = CAMatrix_y(5,5);
  

		pub.publish(msg);
	
		ros::spinOnce(); 

		loop_rate.sleep();
	}
}


	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
