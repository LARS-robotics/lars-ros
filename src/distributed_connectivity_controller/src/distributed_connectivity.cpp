#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "distributed_connectivity_controller/DistConnectivityMsgs.h"
#include "dynamic_reconfigure/server.h"
#include "distributed_connectivity_controller/ControlVariablesConfig.h"
#include <math.h>
#include <algorithm>
#include <vector>
#include "ros/console.h"
#include "stdio.h"
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btMatrix3x3.h> 
#include "nav_msgs/Odometry.h"
#include <Eigen/Dense>
#include <Eigen/LU> 

#define NUM_ROBOTS 6


#define PI	3.1415926535
#define rho	133.334
#define rho1	0.7
#define rho2	2.3

#define dT		0.001

#define alphaOverBar 3.0
#define alphaUnderBar 0

using namespace std;
using namespace Eigen;

std::string robot1_name;
std::string robot2_name;
std::string robot3_name;
std::string robot4_name;
std::string robot5_name;
std::string robot6_name;

double x[NUM_ROBOTS] = {0.0}; 
double y[NUM_ROBOTS] = {0.0};
double theta[NUM_ROBOTS] = {0.0};

bool power = true;

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

void reconfigureCallback(distributed_connectivity_controller::ControlVariablesConfig &config, uint32_t level) {
	//k = config.k;
	//ktheta = config.ktheta;
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
	
	ros::Publisher pub = n.advertise<distributed_connectivity_controller::DistConnectivityMsgs>("/connectivity", 10);
	
	ros::Rate loop_rate(100); // 100 Hz
	
	ros::spinOnce(); 
	
	VectorXd v2[2] = {VectorXd::Random(NUM_ROBOTS), VectorXd::Random(NUM_ROBOTS)};
	VectorXd y1[2] = {VectorXd::Random(NUM_ROBOTS), VectorXd::Random(NUM_ROBOTS)};
	VectorXd y2[2] = {VectorXd::Random(NUM_ROBOTS), VectorXd::Random(NUM_ROBOTS)};
	VectorXd w1[2] = {VectorXd::Random(NUM_ROBOTS), VectorXd::Random(NUM_ROBOTS)};
	VectorXd w2[2] = {VectorXd::Random(NUM_ROBOTS), VectorXd::Random(NUM_ROBOTS)};
	
	VectorXd dv2[2] = {VectorXd::Random(NUM_ROBOTS), VectorXd::Random(NUM_ROBOTS)};
	VectorXd dy1[2] = {VectorXd::Random(NUM_ROBOTS), VectorXd::Random(NUM_ROBOTS)};
	VectorXd dy2[2] = {VectorXd::Random(NUM_ROBOTS), VectorXd::Random(NUM_ROBOTS)};
	VectorXd dw1[2] = {VectorXd::Random(NUM_ROBOTS), VectorXd::Random(NUM_ROBOTS)};
	VectorXd dw2[2] = {VectorXd::Random(NUM_ROBOTS), VectorXd::Random(NUM_ROBOTS)};
	
	VectorXd Lv2 = VectorXd::Random(NUM_ROBOTS);
	VectorXd Ly1 = VectorXd::Random(NUM_ROBOTS);
	VectorXd Lw1 = VectorXd::Random(NUM_ROBOTS);
	VectorXd Ly2 = VectorXd::Random(NUM_ROBOTS);
	VectorXd Lw2 = VectorXd::Random(NUM_ROBOTS);
	
	while (ros::ok())
	{
		double adjMatrix[NUM_ROBOTS][NUM_ROBOTS];
		double adjMatrixunw[NUM_ROBOTS][NUM_ROBOTS];
		int Ni[NUM_ROBOTS]={0};
		int Nind[NUM_ROBOTS][NUM_ROBOTS]={{0}};
		MatrixXd P2 = ArrayXXd::Zero(2,1);
		MatrixXd P3 = ArrayXXd::Zero(3,2);
		MatrixXd P4 = ArrayXXd::Zero(4,3);
		MatrixXd P5 = ArrayXXd::Zero(5,4);
		MatrixXd P6 = ArrayXXd::Zero(6,5);
		MatrixXd M = ArrayXXd::Zero(NUM_ROBOTS-1,NUM_ROBOTS-1);
		MatrixXd L = ArrayXXd::Zero(NUM_ROBOTS,NUM_ROBOTS);
		MatrixXd Lunw = ArrayXXd::Zero(NUM_ROBOTS,NUM_ROBOTS);
		
		P2(0,0) = -0.707106781186547;
		P2(1,0) =  0.707106781186547;		
		
		P3(0,0) = -0.577350269189626;
		P3(0,1) = -0.577350269189626;
		P3(1,0) =  0.788675134594813;
		P3(1,1) = -0.211324865405187;
		P3(2,0) = -0.211324865405187;
		P3(2,1) =  0.788675134594813;
							 
							 
		P4(0,0) = -0.5;
		P4(0,1) = -0.5;
		P4(0,2) = -0.5;
		P4(1,0) =  0.833333333333333;
		P4(1,1) = -0.166666666666667;
		P4(1,2) = -0.166666666666667;
		P4(2,0) = -0.166666666666667;
		P4(2,1) =  0.833333333333333;
		P4(2,2) = -0.166666666666667;
		P4(3,0) = -0.166666666666667;
		P4(3,1) = -0.166666666666667;
		P4(3,2) =  0.833333333333333;
		
		double P_N5[5][4] ={	{1/sqrt(2),-(1/sqrt(6)),-1/(2.*sqrt(3)),-1/(2.*sqrt(5))},
   								{0,sqrt(0.6666666666666666),-1/(2.*sqrt(3)),-1/(2.*sqrt(5))},
   								{0,0,sqrt(3)/2.,-1/(2.*sqrt(5))},
   								{0,0,0,2/sqrt(5)},
   								{-(1/sqrt(2)),-(1/sqrt(6)),-1/(2.*sqrt(3)),-1/(2.*sqrt(5))}
   							};
   							
   		for (int i = 0; i < 5; i++)
   		{
   			for (int j = 0; j < 4; j++)
   			{
   				P5(i,j) = P_N5[i][j]; 
   			}
   		}
		
		double P_N6[6][5] =		{{1/sqrt(2), 	-1/sqrt(6.0), -1/2.0/sqrt(3), 	-1/2.0/sqrt(5), 	-1/sqrt(30)},
								{0.0,        	sqrt(2.0/3),  -1/2.0/sqrt(3), 	-1/2.0/sqrt(5), 	-1/sqrt(30)},
								{0.0, 		 	0.0, 		sqrt(3.0)/2,    	-1/(2*sqrt(5)), -1/sqrt(30)},
								{0.0, 			0.0, 		0.0, 			2/sqrt(5), 		-1/sqrt(30)},
								{0.0, 			0.0, 		0.0,			0.0, 			sqrt(5.0/6)},
								{-1/sqrt(2), 	-1/sqrt(6), -1/2.0/sqrt(3), 	-1/2.0/sqrt(5), 	-1/sqrt(30)}};
		
		MatrixXd P = ArrayXXd::Zero(NUM_ROBOTS,NUM_ROBOTS-1);
		
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
		
		P6 = P;
		
		// Compute the agency matrix of the whole network 
		for (int i = 0; i < NUM_ROBOTS; i++) 
		{
			for (int j = 0; j < NUM_ROBOTS; j++)
			{
				adjMatrix[i][j]=0.0;
				if(i!=j){
					adjMatrix[i][j] = psi( sqrt(pow(x[j]-x[i], 2) + pow(y[j]-y[i], 2)) );
					if (adjMatrix[i][j] >0){
						adjMatrixunw[i][j] = 1;
					}
				}
				//adjMatrix[i][j] = ( sqrt(pow(x[j]-x[i], 2) + pow(y[j]-y[i], 2)) );
				if((adjMatrix[i][j] > 0) || (i==j))
				{
					Nind[i][Ni[i]] =j;Ni[i]++;
				}
				
			} 
			Ni[i]--;// to account for the Ni++ when i==j
			
		}

	/*	ROS_INFO("positions");
		ROS_INFO("%f, %f, %f, %f\n",  x[0], x[1], x[2], x[3] );
		ROS_INFO("%f, %f, %f, %f\n",  y[0], y[1], y[2], y[3] );
				
		ROS_INFO("adj");
		ROS_INFO("%f, %f, %f, %f\n", adjMatrix[0][0], adjMatrix[0][1], adjMatrix[0][2], adjMatrix[0][3]);
		ROS_INFO("%f, %f, %f, %f\n", adjMatrix[1][0], adjMatrix[1][1], adjMatrix[1][2], adjMatrix[1][3]);
		ROS_INFO("%f, %f, %f, %f\n", adjMatrix[2][0], adjMatrix[2][1], adjMatrix[2][2], adjMatrix[2][3]);
		ROS_INFO("%f, %f, %f, %f\n", adjMatrix[3][0], adjMatrix[3][1], adjMatrix[3][2], adjMatrix[3][3]);	
		
	

		
						
		ROS_INFO("start\n");
		ROS_INFO("%d, %d, %d, %d\n", Ni[0], Ni[1], Ni[2], Ni[3]);
		ROS_INFO("%d, %d, %d, %d\n", Nind[0][0], Nind[0][1], Nind[0][2], Nind[0][3]);
		ROS_INFO("%d, %d, %d, %d\n", Nind[1][0], Nind[1][1], Nind[1][2], Nind[1][3]);
		ROS_INFO("%d, %d, %d, %d\n", Nind[2][0], Nind[2][1], Nind[2][2], Nind[2][3]);
		ROS_INFO("%d, %d, %d, %d\n", Nind[3][0], Nind[3][1], Nind[3][2], Nind[3][3]);
		*/
		for (int i = 0; i < NUM_ROBOTS; i++) 
		{
			for (int j = 0; j < NUM_ROBOTS; j++)
			{ 
				L(i,j)=0;
				Lunw(i,j)=0;
				if ( i == j ) {
					double rowSum = 0;
					double rowSumunw = 0;
					for (int k = 0; k < NUM_ROBOTS; k++) {
						rowSum += adjMatrix[i][k];
						rowSumunw += adjMatrixunw[i][k];
					}
					L(i,j) = rowSum - adjMatrix[i][j];
					Lunw(i,j) = rowSumunw - adjMatrixunw[i][j];
				} else {
					L(i,j) = -adjMatrix[i][j];
					Lunw(i,j) = -adjMatrixunw[i][j];
				}
			}
		}
		
	
		M = P.transpose() * L * P;	
		
		ROS_INFO("P");
		ROS_INFO("%f, %f, %f, %f, %f\n", P(0,0), P(0,1), P(0,2), P(0,3), P(0,4));
		ROS_INFO("%f, %f, %f, %f, %f\n", P(1,0), P(1,1), P(1,2), P(1,3), P(1,4));
		ROS_INFO("%f, %f, %f, %f, %f\n", P(2,0), P(2,1), P(2,2), P(2,3), P(2,4));
		ROS_INFO("%f, %f, %f, %f, %f\n", P(3,0), P(3,1), P(3,2), P(3,3), P(3,4));
		ROS_INFO("%f, %f, %f, %f, %f\n", P(4,0), P(4,1), P(4,2), P(4,3), P(4,4));
		ROS_INFO("%f, %f, %f, %f, %f\n", P(5,0), P(5,1), P(5,2), P(5,3), P(5,4));
		
		ROS_INFO("L");
		ROS_INFO("%f, %f, %f, %f, %f, %f\n", L(0,0), L(0,1), L(0,2), L(0,3), L(0,4), L(0,5));
		ROS_INFO("%f, %f, %f, %f, %f, %f\n", L(1,0), L(1,1), L(1,2), L(1,3), L(1,4), L(1,5));
		ROS_INFO("%f, %f, %f, %f, %f, %f\n", L(2,0), L(2,1), L(2,2), L(2,3), L(2,4), L(2,5));
		ROS_INFO("%f, %f, %f, %f, %f, %f\n", L(3,0), L(3,1), L(3,2), L(3,3), L(3,4), L(3,5));
		ROS_INFO("%f, %f, %f, %f, %f, %f\n", L(4,0), L(4,1), L(4,2), L(4,3), L(4,4), L(4,5));
		ROS_INFO("%f, %f, %f, %f, %f, %f\n", L(5,0), L(5,1), L(5,2), L(5,3), L(5,4), L(5,5));	
		
		ROS_INFO("M");
		ROS_INFO("%f, %f, %f, %f, %f\n", M(0,0), M(0,1), M(0,2), M(0,3), M(0,4));
		ROS_INFO("%f, %f, %f, %f, %f\n", M(1,0), M(1,1), M(1,2), M(1,3), M(1,4));
		ROS_INFO("%f, %f, %f, %f, %f\n", M(2,0), M(2,1), M(2,2), M(2,3), M(2,4));
		ROS_INFO("%f, %f, %f, %f, %f\n", M(3,0), M(3,1), M(3,2), M(3,3), M(3,4));
		ROS_INFO("%f, %f, %f, %f, %f\n", M(4,0), M(4,1), M(4,2), M(4,3), M(4,4));

		
		
		
		// Construct the individual adjacency matrices
		// adjacency 0 
		double adjMatrix_0[Ni[0]+1][Ni[0]+1];
		for(int i = 0; i < Ni[0]+1;  i++)
		{
			for(int j = 0; j < Ni[0]+1;  j++)
			{
				adjMatrix_0[i][j] = 0;
			}
		}			
		for(int i=0; i< Ni[0]+1;i++)
		{
			for(int j=0; j< Ni[0]+1;j++)
			{	
				adjMatrix_0[i][j] = adjMatrix[Nind[0][i]][Nind[0][j]];
			}
		}
		
		// adjacency 1 
		double adjMatrix_1[Ni[1]+1][Ni[1]+1];
		for(int i = 0; i < Ni[1]+1;  i++)
		{
			for(int j = 0; j < Ni[1]+1;  j++)
			{
				adjMatrix_1[i][j] = 0;
			}
		}	
		for(int i=0; i< Ni[1]+1;i++)
		{
			for(int j=0; j< Ni[1]+1;j++)
			{	
				adjMatrix_1[i][j] = adjMatrix[(Nind[1][i])][(Nind[1][j])];
			}
		}		
		
		// adjacency 2 
		double adjMatrix_2[Ni[2]+1][Ni[2]+1];
		for(int i = 0; i < Ni[2]+1;  i++)
		{
			for(int j = 0; j < Ni[2]+1;  j++)
			{
				adjMatrix_2[i][j] = 0;
			}
		}	
		for(int i=0; i< Ni[2]+1;i++)
		{
			for(int j=0; j< Ni[2]+1;j++)
			{	
				adjMatrix_2[i][j] = adjMatrix[(Nind[2][i])][(Nind[2][j])];
			}
		}		
		
		// adjacency 3 
		double adjMatrix_3[Ni[3]+1][Ni[3]+1];
		for(int i = 0; i < Ni[3]+1;  i++)
		{
			for(int j = 0; j < Ni[3]+1;  j++)
			{
				adjMatrix_3[i][j] = 0;
			}
		}			
		for(int i=0; i< Ni[3]+1;i++)
		{
			for(int j=0; j< Ni[3]+1;j++)
			{	
				adjMatrix_3[i][j] = adjMatrix[Nind[3][i]][Nind[3][j]];
			}
		}
		
		// adjacency 4 
		double adjMatrix_4[Ni[4]+1][Ni[4]+1];
		for(int i = 0; i < Ni[4]+1;  i++)
		{
			for(int j = 0; j < Ni[4]+1;  j++)
			{
				adjMatrix_4[i][j] = 0;
			}
		}			
		for(int i=0; i< Ni[4]+1;i++)
		{
			for(int j=0; j< Ni[4]+1;j++)
			{	
				adjMatrix_4[i][j] = adjMatrix[Nind[4][i]][Nind[4][j]];
			}
		}
		
		// adjacency 5 
		double adjMatrix_5[Ni[5]+1][Ni[5]+1];
		for(int i = 0; i < Ni[5]+1;  i++)
		{
			for(int j = 0; j < Ni[5]+1;  j++)
			{
				adjMatrix_5[i][j] = 0;
			}
		}			
		for(int i=0; i< Ni[5]+1;i++)
		{
			for(int j=0; j< Ni[5]+1;j++)
			{	
				adjMatrix_5[i][j] = adjMatrix[Nind[5][i]][Nind[5][j]];
			}
		}
/*
		ROS_INFO("adj_3");
		ROS_INFO("%f, %f, %f, %f\n", adjMatrix_3[0][0], adjMatrix_3[0][1], adjMatrix_3[0][2], adjMatrix_3[0][3]);
		ROS_INFO("%f, %f, %f, %f\n", adjMatrix_3[1][0], adjMatrix_3[1][1], adjMatrix_3[1][2], adjMatrix_3[1][3]);
		ROS_INFO("%f, %f, %f, %f\n", adjMatrix_3[2][0], adjMatrix_3[2][1], adjMatrix_3[2][2], adjMatrix_3[2][3]);
		ROS_INFO("%f, %f, %f, %f\n", adjMatrix_3[3][0], adjMatrix_3[3][1], adjMatrix_3[3][2], adjMatrix_3[3][3]);	
		
				
		ROS_INFO("begin");
		ROS_INFO("%d, %d, %d, %d\n", adjMatrix_3[0][0], adjMatrix_3[0][1], adjMatrix_3[0][2], adjMatrix_3[0][3]);
		ROS_INFO("%d, %d, %d, %d\n", adjMatrix_3[1][0], adjMatrix_3[1][1], adjMatrix_3[1][2], adjMatrix_3[1][3]);
		ROS_INFO("%d, %d, %d, %d\n", adjMatrix_3[2][0], adjMatrix_3[2][1], adjMatrix_3[2][2], adjMatrix_3[2][3]);
		ROS_INFO("%d, %d, %d, %d\n", adjMatrix_3[3][0], adjMatrix_3[3][1], adjMatrix_3[3][2], adjMatrix_3[3][3]);		*/
		
		// Construct the Lks and Mks	
		// Need to check if these initialize to zero or garbage !!	
		MatrixXd L0 = ArrayXXd::Zero(Ni[0]+1,Ni[0]+1);
		MatrixXd M0 = ArrayXXd::Zero(Ni[0],Ni[0]);
		MatrixXd L1 = ArrayXXd::Zero(Ni[1]+1,Ni[1]+1);
		MatrixXd M1 = ArrayXXd::Zero(Ni[1],Ni[1]);
		MatrixXd L2 = ArrayXXd::Zero(Ni[2]+1,Ni[2]+1);
		MatrixXd M2 = ArrayXXd::Zero(Ni[2],Ni[2]);
		MatrixXd L3 = ArrayXXd::Zero(Ni[3]+1,Ni[3]+1);
		MatrixXd M3 = ArrayXXd::Zero(Ni[3],Ni[3]);
		
		MatrixXd L4 = ArrayXXd::Zero(Ni[4]+1,Ni[4]+1);
		MatrixXd M4 = ArrayXXd::Zero(Ni[4],Ni[4]);
		MatrixXd L5 = ArrayXXd::Zero(Ni[5]+1,Ni[5]+1);
		MatrixXd M5 = ArrayXXd::Zero(Ni[5],Ni[5]);
		
		// L0
		for (int i = 0; i < Ni[0]+1; i++) 
		{
			for (int j = 0; j < Ni[0]+1; j++)
			{	L0(i,j)=0;
				if ( i == j ) {
					double rowSum = 0;
					for (int k = 0; k < Ni[0]+1; k++) {
						rowSum += adjMatrix_0[i][k];
					}
					L0(i,j) = rowSum - adjMatrix_0[i][j];
				} else {
					L0(i,j) = -adjMatrix_0[i][j];
				}
			}
		}

		// L1
		for (int i = 0; i < Ni[1]+1; i++) 
		{
			for (int j = 0; j < Ni[1]+1; j++)
			{	L1(i,j)=0;
				if ( i == j ) {
					double rowSum = 0;
					for (int k = 0; k < Ni[1]+1; k++) {
						rowSum += adjMatrix_1[i][k];
					}
					L1(i,j) = rowSum - adjMatrix_1[i][j];
				} else {
					L1(i,j) = -adjMatrix_1[i][j];
				}
			}
		}

		// L2
		for (int i = 0; i < Ni[2]+1; i++) 
		{
			for (int j = 0; j < Ni[2]+1; j++)
			{	L2(i,j)=0;
				if ( i == j ) {
					double rowSum = 0;
					for (int k = 0; k < Ni[2]+1; k++) {
						rowSum += adjMatrix_2[i][k];
					}
					L2(i,j) = rowSum - adjMatrix_2[i][j];
				} else {
					L2(i,j) = -adjMatrix_2[i][j];
				}
			}
		}		
		// L3
		for (int i = 0; i < Ni[3]+1; i++) 
		{
			for (int j = 0; j < Ni[3]+1; j++)
			{	L3(i,j)=0;
				if ( i == j ) {
					double rowSum = 0;
					for (int k = 0; k < Ni[3]+1; k++) {
						rowSum += adjMatrix_3[i][k];
					}
					L3(i,j) = rowSum - adjMatrix_3[i][j];
				} else {
					L3(i,j) = -adjMatrix_3[i][j];
				}
			}		
		}	
		
		// L4
		for (int i = 0; i < Ni[4]+1; i++) 
		{
			for (int j = 0; j < Ni[4]+1; j++)
			{	L4(i,j)=0;
				if ( i == j ) {
					double rowSum = 0;
					for (int k = 0; k < Ni[4]+1; k++) {
						rowSum += adjMatrix_4[i][k];
					}
					L4(i,j) = rowSum - adjMatrix_4[i][j];
				} else {
					L4(i,j) = -adjMatrix_4[i][j];
				}
			}		
		}	
		
		// L5
		for (int i = 0; i < Ni[5]+1; i++) 
		{
			for (int j = 0; j < Ni[5]+1; j++)
			{	L5(i,j)=0;
				if ( i == j ) {
					double rowSum = 0;
					for (int k = 0; k < Ni[5]+1; k++) {
						rowSum += adjMatrix_5[i][k];
					}
					L5(i,j) = rowSum - adjMatrix_5[i][j];
				} else {
					L5(i,j) = -adjMatrix_5[i][j];
				}
			}		
		}		
			
		//M0 
		if (Ni[0] == 1)
			M0 = P2.transpose() * L0 * P2;
			
		if (Ni[0] == 2)
			M0 = P3.transpose() * L0 * P3;
			
		if (Ni[0] == 3)
			M0 = P4.transpose() * L0 * P4;
			
		if (Ni[0] == 4)
			M0 = P5.transpose() * L0 * P5;
			
		if (Ni[0] == 5)
			M0 = P6.transpose() * L0 * P6;
		

		//M1 
		if (Ni[1] == 1)
			M1 = P2.transpose() * L1 * P2;
			
		if (Ni[1] == 2)
			M1 = P3.transpose() * L1 * P3;
			
		if (Ni[1] == 3)
			M1 = P4.transpose() * L1 * P4;
			
		if (Ni[1] == 4)
			M1 = P5.transpose() * L1 * P5;
			
		if (Ni[1] == 5)
			M1 = P6.transpose() * L1 * P6;

		//M2 
		if (Ni[2] == 1)
			M2 = P2.transpose() * L2 * P2;
			
		if (Ni[2] == 2)
			M2 = P3.transpose() * L2 * P3;
			
		if (Ni[2] == 3)
			M2 = P4.transpose() * L2 * P4;
			
		if (Ni[2] == 4)
			M2 = P5.transpose() * L2 * P5;
			
		if (Ni[2] == 5)
			M2 = P6.transpose() * L2 * P6;

		//M3 
		if (Ni[3] == 1)
			M3 = P2.transpose() * L3 * P2;
			
		if (Ni[3] == 2)
			M3 = P3.transpose() * L3 * P3;
			
		if (Ni[3] == 3)
			M3 = P4.transpose() * L3 * P4;
			
		if (Ni[3] == 4)
			M3 = P5.transpose() * L3 * P5;
			
		if (Ni[3] == 5)
			M3 = P6.transpose() * L3 * P6;
			
			
		//M4 
		if (Ni[4] == 1)
			M4 = P2.transpose() * L4 * P2;
			
		if (Ni[4] == 2)
			M4 = P3.transpose() * L4 * P3;
			
		if (Ni[4] == 3)
			M4 = P4.transpose() * L4 * P4;
		
		if (Ni[4] == 4)
			M4 = P5.transpose() * L4 * P5;
			
		if (Ni[4] == 5)
			M4 = P6.transpose() * L4 * P6;


		//M5 
		if (Ni[5] == 1)
			M5 = P2.transpose() * L5 * P2;
			
		if (Ni[5] == 2)
			M5 = P3.transpose() * L5 * P3;
			
		if (Ni[5] == 3)
			M5 = P4.transpose() * L5 * P4;
			
		if (Ni[5] == 4)
			M5 = P5.transpose() * L5 * P5;
			
		if (Ni[5] == 5)
			M5 = P6.transpose() * L5 * P6;
			
	   	// We have computed all M_k and L_ks;
	   	// Now we compute the dLk_dx and dLk_dx


		MatrixXd dLdx_0 = ArrayXXd::Zero(Ni[0]+1,Ni[0]+1);
		MatrixXd dLdx_1 = ArrayXXd::Zero(Ni[1]+1,Ni[1]+1);
		MatrixXd dLdx_2 = ArrayXXd::Zero(Ni[2]+1,Ni[2]+1);
		MatrixXd dLdx_3 = ArrayXXd::Zero(Ni[3]+1,Ni[3]+1);
		MatrixXd dLdx_4 = ArrayXXd::Zero(Ni[4]+1,Ni[4]+1);
		MatrixXd dLdx_5 = ArrayXXd::Zero(Ni[5]+1,Ni[5]+1);
		
		MatrixXd dLdy_0 = ArrayXXd::Zero(Ni[0]+1,Ni[0]+1);
		MatrixXd dLdy_1 = ArrayXXd::Zero(Ni[1]+1,Ni[1]+1);
		MatrixXd dLdy_2 = ArrayXXd::Zero(Ni[2]+1,Ni[2]+1);
		MatrixXd dLdy_3 = ArrayXXd::Zero(Ni[3]+1,Ni[3]+1);
		MatrixXd dLdy_4 = ArrayXXd::Zero(Ni[4]+1,Ni[4]+1);
		MatrixXd dLdy_5 = ArrayXXd::Zero(Ni[5]+1,Ni[5]+1);
		
		MatrixXd dMdx_0 = ArrayXXd::Zero(Ni[0],Ni[0]);
		MatrixXd dMdx_1 = ArrayXXd::Zero(Ni[1],Ni[1]);
		MatrixXd dMdx_2 = ArrayXXd::Zero(Ni[2],Ni[2]);
		MatrixXd dMdx_3 = ArrayXXd::Zero(Ni[3],Ni[3]);
		MatrixXd dMdx_4 = ArrayXXd::Zero(Ni[4],Ni[4]);
		MatrixXd dMdx_5 = ArrayXXd::Zero(Ni[5],Ni[5]);
		
		MatrixXd dMdy_0 = ArrayXXd::Zero(Ni[0],Ni[0]);
		MatrixXd dMdy_1 = ArrayXXd::Zero(Ni[1],Ni[1]);
		MatrixXd dMdy_2 = ArrayXXd::Zero(Ni[2],Ni[2]);
		MatrixXd dMdy_3 = ArrayXXd::Zero(Ni[3],Ni[3]);
		MatrixXd dMdy_4 = ArrayXXd::Zero(Ni[4],Ni[4]);
		MatrixXd dMdy_5 = ArrayXXd::Zero(Ni[5],Ni[5]);

		// Calculate dLdx and dLdy for each robot 0
		int currind = 0;
		for(int i = 0; i<Ni[currind]+1; i++)
		{
			for (int j=i+1; j<Ni[currind]+1; j++) 
			{   
				dLdx_0(i,j)=0;
				dLdy_0(i,j)=0;  
				double D = sqrt( pow(x[ Nind[currind][i] ] - x[  Nind[currind][j] ], 2) + pow(y[ Nind[currind][i] ] - y[ Nind[currind][j] ], 2));
				double dAdD = psiPrime(D);				
				
				//whatever =x[i] - x[j];
				
				if( currind == Nind[currind][i] ) {
					double dDdx = (x[  Nind[currind][i] ]-x[  Nind[currind][j] ]) / D;							
					double dDdy = (y[  Nind[currind][i] ]-y[  Nind[currind][j] ]) / D;
							
					dLdx_0(i,j) = dAdD*dDdx;
            		dLdy_0(i,j) = dAdD*dDdy;        		
            		
				}
				
				if( currind == Nind[currind][j]  ) {
					double dDdx = -(x[  Nind[currind][i] ]-x[  Nind[currind][j] ]) / D;							
					double dDdy = -(y[  Nind[currind][i] ]-y[  Nind[currind][j] ]) / D;
							
					dLdx_0(i,j) = dAdD*dDdx;
            		dLdy_0(i,j) = dAdD*dDdy;
                  		
				}
				
           	
            	dLdx_0(j,i) = dLdx_0(i,j);
            	dLdy_0(j,i) = dLdy_0(i,j);
            
			}
		}
		
		for(int i = 0; i < Ni[currind]+1; i++)
		{
			double sumx = 0;
			double sumy = 0;
			for(int j = 0; j<Ni[currind]+1; j++)
			{
				sumx += dLdx_0(i,j);
				sumy += dLdy_0(i,j);
			} 
			dLdx_0(i,i) = -sumx;
            dLdy_0(i,i) = -sumy;
		}
				
		// Calculate dLdx and dLdy for robot 1
		currind = 1;
		for(int i = 0; i<Ni[currind]+1; i++)
		{
			for (int j=i+1; j<Ni[currind]+1; j++) 
			{
				dLdx_1(i,j)=0;
				dLdy_1(i,j)=0;  			
				double D = sqrt( pow(x[ Nind[currind][i] ] - x[  Nind[currind][j] ], 2) + pow(y[ Nind[currind][i] ] - y[ Nind[currind][j] ], 2));
				double dAdD = psiPrime(D);				
				
				//whatever =x[i] - x[j];
				
				if( currind == Nind[currind][i] ) {
					double dDdx = (x[  Nind[currind][i] ]-x[  Nind[currind][j] ]) / D;							
					double dDdy = (y[  Nind[currind][i] ]-y[  Nind[currind][j] ]) / D;
							
					dLdx_1(i,j) = dAdD*dDdx;
            		dLdy_1(i,j) = dAdD*dDdy;        		
            		
				}
				
				if( currind == Nind[currind][j]  ) {
					double dDdx = -(x[  Nind[currind][i] ]-x[  Nind[currind][j] ]) / D;							
					double dDdy = -(y[  Nind[currind][i] ]-y[  Nind[currind][j] ]) / D;
							
					dLdx_1(i,j) = dAdD*dDdx;
            		dLdy_1(i,j) = dAdD*dDdy;
                  		
				}
				
           	
            	dLdx_1(j,i) = dLdx_1(i,j);
            	dLdy_1(j,i) = dLdy_1(i,j);
            
			}
		}
		
		for(int i = 0; i < Ni[currind]+1; i++)
		{
			double sumx = 0;
			double sumy = 0;
			for(int j = 0; j<Ni[currind]+1; j++)
			{
				sumx += dLdx_1(i,j);
				sumy += dLdy_1(i,j);
			} 
			dLdx_1(i,i) = -sumx;
            dLdy_1(i,i) = -sumy;
		}
		// Finished dLdx_1 dLdy_1	
		
		// Calculate dLdx and dLdy for robot 2
		currind = 2;
		for(int i = 0; i<Ni[currind]+1; i++)
		{
			for (int j=i+1; j<Ni[currind]+1; j++) 
			{
				dLdx_2(i,j)=0;
				dLdy_2(i,j)=0;  			
				double D = sqrt( pow(x[ Nind[currind][i] ] - x[  Nind[currind][j] ], 2) + pow(y[ Nind[currind][i] ] - y[ Nind[currind][j] ], 2));
				double dAdD = psiPrime(D);				
				
				//whatever =x[i] - x[j];
				
				if( currind == Nind[currind][i] ) {
					double dDdx = (x[  Nind[currind][i] ]-x[  Nind[currind][j] ]) / D;							
					double dDdy = (y[  Nind[currind][i] ]-y[  Nind[currind][j] ]) / D;
							
					dLdx_2(i,j) = dAdD*dDdx;
            		dLdy_2(i,j) = dAdD*dDdy;        		
            		
				}
				
				if( currind == Nind[currind][j]  ) {
					double dDdx = -(x[  Nind[currind][i] ]-x[  Nind[currind][j] ]) / D;							
					double dDdy = -(y[  Nind[currind][i] ]-y[  Nind[currind][j] ]) / D;
							
					dLdx_2(i,j) = dAdD*dDdx;
            		dLdy_2(i,j) = dAdD*dDdy;
                  		
				}
				
           	
            	dLdx_2(j,i) = dLdx_2(i,j);
            	dLdy_2(j,i) = dLdy_2(i,j);
            
			}
		}
		
		for(int i = 0; i < Ni[currind]+1; i++)
		{
			double sumx = 0;
			double sumy = 0;
			for(int j = 0; j<Ni[currind]+1; j++)
			{
				sumx += dLdx_2(i,j);
				sumy += dLdy_2(i,j);
			} 
			dLdx_2(i,i) = -sumx;
            dLdy_2(i,i) = -sumy;
		}
		// Finished dLdx_2 dLdy_2	

		// Calculate dLdx and dLdy for robot 3
		currind = 3;
		for(int i = 0; i<Ni[currind]+1; i++)
		{
			for (int j=i+1; j<Ni[currind]+1; j++) 
			{
				dLdx_3(i,j)=0;
				dLdy_3(i,j)=0;  			
				double D = sqrt( pow(x[ Nind[currind][i] ] - x[  Nind[currind][j] ], 2) + pow(y[ Nind[currind][i] ] - y[ Nind[currind][j] ], 2));
				double dAdD = psiPrime(D);				
				
				//whatever =x[i] - x[j];
				
				if( currind == Nind[currind][i] ) {
					double dDdx = (x[  Nind[currind][i] ]-x[  Nind[currind][j] ]) / D;							
					double dDdy = (y[  Nind[currind][i] ]-y[  Nind[currind][j] ]) / D;
							
					dLdx_3(i,j) = dAdD*dDdx;
            		dLdy_3(i,j) = dAdD*dDdy;        		
            		
				}
				
				if( currind == Nind[currind][j]  ) {
					double dDdx = -(x[  Nind[currind][i] ]-x[  Nind[currind][j] ]) / D;							
					double dDdy = -(y[  Nind[currind][i] ]-y[  Nind[currind][j] ]) / D;
							
					dLdx_3(i,j) = dAdD*dDdx;
            		dLdy_3(i,j) = dAdD*dDdy;
                  		
				}
				
           	
            	dLdx_3(j,i) = dLdx_3(i,j);
            	dLdy_3(j,i) = dLdy_3(i,j);
            
			}
		}
		
		for(int i = 0; i < Ni[currind]+1; i++)
		{
			double sumx = 0;
			double sumy = 0;
			for(int j = 0; j<Ni[currind]+1; j++)
			{
				sumx += dLdx_3(i,j);
				sumy += dLdy_3(i,j);
			} 
			dLdx_3(i,i) = -sumx;
            dLdy_3(i,i) = -sumy;
		}
		// Finished dLdx_3 dLdy_3	
		
		// Calculate dLdx and dLdy for robot 4
		currind = 4;
		for(int i = 0; i<Ni[currind]+1; i++)
		{
			for (int j=i+1; j<Ni[currind]+1; j++) 
			{
				dLdx_4(i,j)=0;
				dLdy_4(i,j)=0;  			
				double D = sqrt( pow(x[ Nind[currind][i] ] - x[  Nind[currind][j] ], 2) + pow(y[ Nind[currind][i] ] - y[ Nind[currind][j] ], 2));
				double dAdD = psiPrime(D);				
				
				//whatever =x[i] - x[j];
				
				if( currind == Nind[currind][i] ) {
					double dDdx = (x[  Nind[currind][i] ]-x[  Nind[currind][j] ]) / D;							
					double dDdy = (y[  Nind[currind][i] ]-y[  Nind[currind][j] ]) / D;
							
					dLdx_4(i,j) = dAdD*dDdx;
            		dLdy_4(i,j) = dAdD*dDdy;        		
            		
				}
				
				if( currind == Nind[currind][j]  ) {
					double dDdx = -(x[  Nind[currind][i] ]-x[  Nind[currind][j] ]) / D;							
					double dDdy = -(y[  Nind[currind][i] ]-y[  Nind[currind][j] ]) / D;
							
					dLdx_4(i,j) = dAdD*dDdx;
            		dLdy_4(i,j) = dAdD*dDdy;
                  		
				}
				
           	
            	dLdx_4(j,i) = dLdx_4(i,j);
            	dLdy_4(j,i) = dLdy_4(i,j);
            
			}
		}
		
		for(int i = 0; i < Ni[currind]+1; i++)
		{
			double sumx = 0;
			double sumy = 0;
			for(int j = 0; j<Ni[currind]+1; j++)
			{
				sumx += dLdx_4(i,j);
				sumy += dLdy_4(i,j);
			} 
			dLdx_4(i,i) = -sumx;
            dLdy_4(i,i) = -sumy;
		}
		// Finished dLdx_4 dLdy_4	

		// Calculate dLdx and dLdy for robot 5
		currind = 5;
		for(int i = 0; i<Ni[currind]+1; i++)
		{
			for (int j=i+1; j<Ni[currind]+1; j++) 
			{
				dLdx_5(i,j)=0;
				dLdy_5(i,j)=0;  			
				double D = sqrt( pow(x[ Nind[currind][i] ] - x[  Nind[currind][j] ], 2) + pow(y[ Nind[currind][i] ] - y[ Nind[currind][j] ], 2));
				double dAdD = psiPrime(D);				
				
				//whatever =x[i] - x[j];
				
				if( currind == Nind[currind][i] ) {
					double dDdx = (x[  Nind[currind][i] ]-x[  Nind[currind][j] ]) / D;							
					double dDdy = (y[  Nind[currind][i] ]-y[  Nind[currind][j] ]) / D;
							
					dLdx_5(i,j) = dAdD*dDdx;
            		dLdy_5(i,j) = dAdD*dDdy;        		
            		
				}
				
				if( currind == Nind[currind][j]  ) {
					double dDdx = -(x[  Nind[currind][i] ]-x[  Nind[currind][j] ]) / D;							
					double dDdy = -(y[  Nind[currind][i] ]-y[  Nind[currind][j] ]) / D;
							
					dLdx_5(i,j) = dAdD*dDdx;
            		dLdy_5(i,j) = dAdD*dDdy;
                  		
				}
				
           	
            	dLdx_5(j,i) = dLdx_5(i,j);
            	dLdy_5(j,i) = dLdy_5(i,j);
            
			}
		}
		
		for(int i = 0; i < Ni[currind]+1; i++)
		{
			double sumx = 0;
			double sumy = 0;
			for(int j = 0; j<Ni[currind]+1; j++)
			{
				sumx += dLdx_5(i,j);
				sumy += dLdy_5(i,j);
			} 
			dLdx_5(i,i) = -sumx;
            dLdy_5(i,i) = -sumy;
		}
		// Finished dLdx_5 dLdy_5		


		if (Ni[0] == 1){
			dMdx_0 = P2.transpose() * dLdx_0 * P2;
			dMdy_0 = P2.transpose() * dLdy_0 * P2;
		}
		if (Ni[0] == 2){
			dMdx_0 = P3.transpose() * dLdx_0 * P3;
			dMdy_0 = P3.transpose() * dLdy_0 * P3;
		}
		if (Ni[0] == 3){
			dMdx_0 = P4.transpose() * dLdx_0 * P4;
			dMdy_0 = P4.transpose() * dLdy_0 * P4;
		}
		if (Ni[0] == 4){
			dMdx_0 = P5.transpose() * dLdx_0 * P5;
			dMdy_0 = P5.transpose() * dLdy_0 * P5;
		}
		if (Ni[0] == 5){
			dMdx_0 = P6.transpose() * dLdx_0 * P6;
			dMdy_0 = P6.transpose() * dLdy_0 * P6;
		}

		if (Ni[1] == 1){
			dMdx_1 = P2.transpose() * dLdx_1 * P2;
			dMdy_1 = P2.transpose() * dLdy_1 * P2;
		}
		if (Ni[1] == 2){
			dMdx_1 = P3.transpose() * dLdx_1 * P3;
			dMdy_1 = P3.transpose() * dLdy_1 * P3;
		}
		if (Ni[1] == 3){

			dMdx_1 = P4.transpose() * dLdx_1 * P4;
			dMdy_1 = P4.transpose() * dLdy_1 * P4;
		}
		if (Ni[1] == 4){
			dMdx_1 = P5.transpose() * dLdx_1 * P5;
			dMdy_1 = P5.transpose() * dLdy_1 * P5;
		}
		if (Ni[1] == 5){

			dMdx_1 = P6.transpose() * dLdx_1 * P6;
			dMdy_1 = P6.transpose() * dLdy_1 * P6;
		}
		
		if (Ni[2] == 1){
			dMdx_2 = P2.transpose() * dLdx_2 * P2;
			dMdy_2 = P2.transpose() * dLdy_2 * P2;
		}
		if (Ni[2] == 2){
			dMdx_2 = P3.transpose() * dLdx_2 * P3;
			dMdy_2 = P3.transpose() * dLdy_2 * P3;
		}
		if (Ni[2] == 3){
			dMdx_2 = P4.transpose() * dLdx_2 * P4;
			dMdy_2 = P4.transpose() * dLdy_2 * P4;
		}
		if (Ni[2] == 4){
			dMdx_2 = P5.transpose() * dLdx_2 * P5;
			dMdy_2 = P5.transpose() * dLdy_2 * P5;
		}
		if (Ni[2] == 5){
			dMdx_2 = P6.transpose() * dLdx_2 * P6;
			dMdy_2 = P6.transpose() * dLdy_2 * P6;
		}

		if (Ni[3] == 1){
			dMdx_3 = P2.transpose() * dLdx_3 * P2;
			dMdy_3 = P2.transpose() * dLdy_3 * P2;
		}
		if (Ni[3] == 2){
			dMdx_3 = P3.transpose() * dLdx_3 * P3;
			dMdy_3 = P3.transpose() * dLdy_3 * P3;
		}
		if (Ni[3] == 3){
			dMdx_3 = P4.transpose() * dLdx_3 * P4;
			dMdy_3 = P4.transpose() * dLdy_3 * P4;
		}
		if (Ni[3] == 4){
			dMdx_3 = P5.transpose() * dLdx_3 * P5;
			dMdy_3 = P5.transpose() * dLdy_3 * P5;
		}
		if (Ni[3] == 5){
			dMdx_3 = P6.transpose() * dLdx_3 * P6;
			dMdy_3 = P6.transpose() * dLdy_3 * P6;
		}
		
		if (Ni[4] == 1){
			dMdx_4 = P2.transpose() * dLdx_4 * P2;
			dMdy_4 = P2.transpose() * dLdy_4 * P2;
		}
		if (Ni[4] == 2){
			dMdx_4 = P3.transpose() * dLdx_4 * P3;
			dMdy_4 = P3.transpose() * dLdy_4 * P3;
		}
		if (Ni[4] == 3){
			dMdx_4 = P4.transpose() * dLdx_4 * P4;
			dMdy_4 = P4.transpose() * dLdy_4 * P4;
		}
		if (Ni[4] == 4){
			dMdx_4 = P5.transpose() * dLdx_4 * P5;
			dMdy_4 = P5.transpose() * dLdy_4 * P5;
		}
		if (Ni[4] == 5){
			dMdx_4 = P6.transpose() * dLdx_4 * P6;
			dMdy_4 = P6.transpose() * dLdy_4 * P6;
		}

		if (Ni[5] == 1){
			dMdx_5 = P2.transpose() * dLdx_5 * P2;
			dMdy_5 = P2.transpose() * dLdy_5 * P2;
		}
		if (Ni[5] == 2){
			dMdx_5 = P3.transpose() * dLdx_5 * P3;
			dMdy_5 = P3.transpose() * dLdy_5 * P3;
		}
		if (Ni[5] == 3){
			dMdx_5 = P4.transpose() * dLdx_5 * P4;
			dMdy_5 = P4.transpose() * dLdy_5 * P4;
		}
		if (Ni[5] == 4){
			dMdx_5 = P5.transpose() * dLdx_5 * P5;
			dMdy_5 = P5.transpose() * dLdy_5 * P5;
		}
		if (Ni[5] == 5){
			dMdx_5 = P6.transpose() * dLdx_5 * P6;
			dMdy_5 = P6.transpose() * dLdy_5 * P6;
		}					
					
		double trMx_0 = (M0.inverse()*dMdx_0).trace();
		double trMx_1 = (M1.inverse()*dMdx_1).trace();
		double trMx_2 = (M2.inverse()*dMdx_2).trace();
		double trMx_3 = (M3.inverse()*dMdx_3).trace();
		double trMx_4 = (M4.inverse()*dMdx_4).trace();
		double trMx_5 = (M5.inverse()*dMdx_5).trace();
		
		double trMy_0 = (M0.inverse()*dMdy_0).trace();
		double trMy_1 = (M1.inverse()*dMdy_1).trace();
		double trMy_2 = (M2.inverse()*dMdy_2).trace();
		double trMy_3 = (M3.inverse()*dMdy_3).trace();
		double trMy_4 = (M4.inverse()*dMdy_4).trace();
		double trMy_5 = (M5.inverse()*dMdy_5).trace();

		double detM = M.determinant();
		double Vcgain[NUM_ROBOTS] = {0.0};
		
		
		// find eigenvalues 
		double lambda = 0;
		
		EigenSolver<MatrixXd> es(M);
		
		double eigenvalues[] = { es.eigenvalues()[0].real(),
								es.eigenvalues()[1].real(),
								 es.eigenvalues()[2].real(),
								es.eigenvalues()[3].real(),
								 es.eigenvalues()[4].real() };
								 
		vector<double> eigenList (eigenvalues, eigenvalues + sizeof(eigenvalues) / sizeof(double) );
		
		sort(eigenList.begin(), eigenList.end());
		
		lambda = eigenList[0];
		
		// assign smallest to lambda2
		
		// Integrators

		Lv2 = L*v2[0];
		Ly1 = Lunw*y1[0];
		Lw1 = Lunw*w1[0];
		Ly2 = Lunw*y2[0];
		Lw2 = Lunw*w2[0];
		//ROS_INFO("L[0][0] = %f", Lunw(0, 0));
		//ROS_INFO("Ly2 = %f, %f, %f, %f", Ly2[0], Ly2[1], Ly2[2],Ly2[3]);
		double lambda2[NUM_ROBOTS] = {0.0};
		double k1,k2,k3,gamma,kp,ki;
		k1 = 18;
		k2 = 3;
		k3 = 60;
		kp = 250;
		ki = 22;
		gamma = 25;
		for (int i = 0; i < NUM_ROBOTS; i++) {

			dv2[1](i) = -k1*y1[0](i) - k2*Lv2(i) - k3*( (y2[0](i) - 1) *v2[0](i));
			dy1[1](i) = gamma*(v2[0](i)-y1[0](i)) - kp*Ly1(i)+ki*Lw1(i);
			dw1[1](i) = -ki*Ly1(i);
			dy2[1](i) = gamma*((v2[0](i)*v2[0](i))-y2[0](i)) -kp*Ly2(i) +ki*Lw2(i);
			dw2[1](i) = -ki*Ly2(i);
			
			//ROS_INFO("dy2[0](0) = %f", dy2[0](0));
			//ROS_INFO("dy2[1](0) = %f", dy2[1](0));
			v2[1](i) = v2[0](i) + (dT/2)*(dv2[1](i) + dv2[0](i));
			y1[1](i) = y1[0](i) + (dT/2)*(dy1[1](i) + dy1[0](i));
			w1[1](i) = w1[0](i) + (dT/2)*(dw1[1](i) + dw1[0](i));
			y2[1](i) = y2[0](i) + (dT/2)*(dy2[1](i) + dy2[0](i));
			w2[1](i) = w2[0](i) + (dT/2)*(dw2[1](i) + dw2[0](i));
			//ROS_INFO("y2[0](0) = %f", y2[0](0));
			lambda2[i] = (1-y2[1](i))*(k3/k2);

			dv2[0](i) = dv2[1](i);
			dy1[0](i) = dy1[1](i);
			dw1[0](i) = dw1[1](i);
			dy2[0](i) = dy2[1](i);
			dw2[0](i) = dw2[1](i);
		
			v2[0](i) = v2[1](i);
			y1[0](i) = y1[1](i);
			w1[0](i) = w1[1](i);
			y2[0](i) = y2[1](i);
			w2[0](i) = w2[1](i);
		}
		// End Integrators
		
		
		
		Vcgain[0] = 4*(((pow(alphaOverBar,2)-pow(alphaUnderBar,2))*(pow(lambda2[0], 2)-pow(alphaOverBar,2)))
				     /(pow(pow(lambda2[0], 2)-pow(alphaUnderBar,2),3)))*lambda2[0];
				     
		Vcgain[1] = 4*(((pow(alphaOverBar,2)-pow(alphaUnderBar,2))*(pow(lambda2[1], 2)-pow(alphaOverBar,2)))
				     /(pow(pow(lambda2[1], 2)-pow(alphaUnderBar,2),3)))*lambda2[1];
				     
		Vcgain[2] = 4*(((pow(alphaOverBar,2)-pow(alphaUnderBar,2))*(pow(lambda2[2], 2)-pow(alphaOverBar,2)))
				     /(pow(pow(lambda2[2], 2)-pow(alphaUnderBar,2),3)))*lambda2[2];
				     
		Vcgain[3] = 4*(((pow(alphaOverBar,2)-pow(alphaUnderBar,2))*(pow(lambda2[3], 2)-pow(alphaOverBar,2)))
				     /(pow(pow(lambda2[3], 2)-pow(alphaUnderBar,2),3)))*lambda2[3];
				     
		Vcgain[4] = 4*(((pow(alphaOverBar,2)-pow(alphaUnderBar,2))*(pow(lambda2[4], 2)-pow(alphaOverBar,2)))
				     /(pow(pow(lambda2[4], 2)-pow(alphaUnderBar,2),3)))*lambda2[4];
				     
		Vcgain[5] = 4*(((pow(alphaOverBar,2)-pow(alphaUnderBar,2))*(pow(lambda2[5], 2)-pow(alphaOverBar,2)))
				     /(pow(pow(lambda2[5], 2)-pow(alphaUnderBar,2),3)))*lambda2[5];
		
		
		
		double u_conn_x[NUM_ROBOTS]={Vcgain[0]*trMx_0,Vcgain[1]*trMx_1,Vcgain[2]*trMx_2,Vcgain[3]*trMx_3,Vcgain[4]*trMx_4,Vcgain[5]*trMx_5};
		double u_conn_y[NUM_ROBOTS]={Vcgain[0]*trMy_0,Vcgain[1]*trMy_1,Vcgain[2]*trMy_2,Vcgain[3]*trMy_3,Vcgain[4]*trMy_4,Vcgain[5]*trMy_5};
		
		
		
		/////////////////////////////////////////////////////////////
		// Now begin calculation of formation and collision avoidance
		
		
		MatrixXd CAMatrix_x(NUM_ROBOTS,NUM_ROBOTS);		
		MatrixXd CAMatrix_y(NUM_ROBOTS,NUM_ROBOTS);
						  	
		CAMatrix_x <<	0,0,0,0,0,0,
				 		0,0,0,0,0,0,
					 	0,0,0,0,0,0,
					 	0,0,0,0,0,0,
					 	0,0,0,0,0,0,
				 		0,0,0,0,0,0;

		CAMatrix_y <<	0,0,0,0,0,0,
				 		0,0,0,0,0,0,
					 	0,0,0,0,0,0,
					 	0,0,0,0,0,0,
					 	0,0,0,0,0,0,
				 		0,0,0,0,0,0;
				 		
		double u_ca_x[NUM_ROBOTS]={0.0};				 		 
		double u_ca_y[NUM_ROBOTS]={0.0};				 		 
		
		double u_fc_x[NUM_ROBOTS]={0.0};				 		 
		double u_fc_y[NUM_ROBOTS]={0.0};	

		//double xd[NUM_ROBOTS]={ 1.0, 0.0, -1.0,  1.0,  0.0, -0.9};	
		//double yd[NUM_ROBOTS]={ 0.5, 0.5,  0.5, -0.5, -0.5, -0.5};
		
		//double xd[NUM_ROBOTS]={ 0.5457, 1.9729, 1.237,-0.7, -0.3, -0.9};
		double xd[NUM_ROBOTS]={ -1.6, 1.9729, 1.237,0.5, -0.3, -0.9};		
		double yd[NUM_ROBOTS]={-1.5440,-1.5410,-1.539,-1.54, -1.54, -1.54};
		
		xd[0] = x[0];
		yd[0] = y[0];
		//double xd[4]={ 0.7, 0.7,-0.7,-0.7};	
		//double yd[4]={-0.7, 0.7, 0.7,-0.7};
		//double xd[4]={ 0.5457, -0.7, 1.237,1.8};	
		//double yd[4]={-1.5740,-0.7,-1.539,-1.6};
								
		double R = 	0.7;	  
		double r =  0.400;
		
		
		// Collision Avoidance for each robot starts here:
		for(int i = 0; i<NUM_ROBOTS; i++)
		{
			for (int j=i+1; j<NUM_ROBOTS; j++) 
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
		
		for(int i = 0; i < NUM_ROBOTS; i++)
		{
			double sumx = 0;
			double sumy = 0;
			
			for (int j=0; j<NUM_ROBOTS; j++) 
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
		
		
		// Collision Avoidance and Formation Control ends here

		
		//////////////////////////////////////////////////////////////
		//////////////  Send Data  ///////////////////////////////////
		
		distributed_connectivity_controller::DistConnectivityMsgs msg; 
		for(int i =0; i < NUM_ROBOTS; i++)
		{
			msg.u_conn_x[i] = u_conn_x[i];
			msg.u_conn_y[i] = u_conn_y[i];
			msg.u_ca_x[i] = u_ca_x[i];
			msg.u_ca_y[i] = u_ca_y[i];
			msg.u_fc_x[i] = u_fc_x[i];
			msg.u_fc_y[i] = u_fc_y[i];
			msg.lambda2[i] = lambda2[i];
		}
		
		
		msg.detM = detM;
		
		msg.lambda = lambda;
		
		msg.power = power;
		
		pub.publish(msg);
		

		
		ros::spinOnce(); 

		loop_rate.sleep();
	}
	
}































