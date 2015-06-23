// ROS includes
#include "ros/ros.h"
#include "ros/assert.h"
#include "dynamic_reconfigure/server.h"
#include "create_driver/vicon_driver.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>

// Library includes
#include <string>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm> 

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

// I need for the socket programming
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>  

#include "jingfu/control_formation.h"
#include <Eigen/Core>
#include <Eigen/Dense>
using Eigen::MatrixXd;
using namespace Eigen;

using namespace std;
using std::setw;
using std::setprecision;

/*
# define port 6001 // define port number
# define size 1024 // define data size
*/

// Constants
//const double PI = 3.1415926535;

/*
double x = 0; 
double y = 0;
double theta = 0;
double ox = 0;
double oy = 0;
*/

double joy_x,joy_y,joy_z;
int new_msg=0;
sensor_msgs::Joy joy_msg_in;
geometry_msgs::Vector3 v3_msg; 

/*server

double mode = 1;
int Modeserver = 0;
int sockfd, bindit, listento, new_fd, sin_size; 
//int count = 0;
struct sockaddr_in my_addr; // my address information 
struct sockaddr_in client_addr; // address information of connected machine 

*/

void joy_callback(const sensor_msgs::Joy joy_msg_in)
{
	//Take in joystick
	joy_x=joy_msg_in.axes[3];
	joy_y=joy_msg_in.axes[2];
	//joy_z=joy_msg_in.axes[2];
	printf("hello joy\n");
	printf("hello joy\n");
	//Take in time
	//msg_time=(double)ros::Time::now().toNSec();
    new_msg=1;
}

/*
void server() 
{ 	 
	printf("i am inside of server\n"); 
	
	sockfd = socket(AF_INET, SOCK_STREAM, 0); 
	if(sockfd<0) { 
		printf("Error socket\n"); 
		exit(1); 
	} 
	
	my_addr.sin_family = AF_INET; // host byte order, AF_INET = IPv4 internet protocols for Linux 
	my_addr.sin_addr.s_addr = INADDR_ANY; // use my address automatically 
	my_addr.sin_port = htons(port); 
	memset(&(my_addr.sin_zero),'\0',8); 
	// assign the address specified to the socket. 
	bindit = bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)); 
	if (bindit<0) { 
		printf("error bindit\n"); 
		exit(1); 
	} 
	printf("bind success\n"); 
	//listen the connection on created socket 
  	listento = listen(sockfd, 5); 
  	//error check for listen() 
  	if (listento<0) { 
       		printf ("error listento\n"); 
            	exit(1); 
  	} 
	printf("linsten success\n"); 
	sin_size = sizeof(struct sockaddr_in); 
	// accept a connection on listened socket 
	new_fd = accept(sockfd, (struct sockaddr *)&client_addr, (socklen_t*)&sin_size); 
	  //error check for accept() 
	if (new_fd < 0) {           
		 printf("accept() has failed!\n"); 
	  } 
	printf("accept success\n"); 
	cout << "server: got connection from " << inet_ntoa(client_addr.sin_addr) << endl; 
	Modeserver = 1;
	
} 
*/


int main(int argc, char **argv)
{
	// ROS Initalization
	ros::init(argc, argv, "control");
	
	ros::NodeHandle node;
	ros::NodeHandle private_node("~");

	ros::Publisher pub_v3;
	ros::Subscriber joy_sub;
	pub_v3 = node.advertise<geometry_msgs::Vector3>("joy_vel", 1); //send velocity for graphing on /joy_vel topic
	joy_sub = node.subscribe("/joy", 1, joy_callback); //suscribe to the joystick message
	
	ROS_INFO("Waiting for joystick message");
	ros::Rate rate(100.0);
	ROS_INFO("Starting Joy --> cmd_vel Node");

	// ROS Parameters

	// List of robot names
	vector<string> robot_names;
	XmlRpc::XmlRpcValue robot_list;
	private_node.getParam("robot_list", robot_list);
	ROS_ASSERT(robot_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for (int i = 0; i < robot_list.size(); i++) 
	{
		ROS_ASSERT(robot_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
		robot_names.push_back(static_cast<string>(robot_list[i]));
	}

	// Number of robots
	const int num_robots = robot_names.size();

	// Robot name
	string robot_name;
	private_node.param<std::string>("robotname", robot_name, "defaultname");

	int index = find(robot_names.begin(), robot_names.end(), robot_name) - robot_names.begin();

	// ROS Subscribers
	map<string, create_driver::ViconStream> vicon;
	vector<string>::iterator name_it;
	vector<ros::Subscriber> sub;
	for (name_it = robot_names.begin(); name_it != robot_names.end(); name_it++)
	{
		vicon.insert(pair<string, create_driver::ViconStream>(*name_it, create_driver::ViconStream()));
		sub.push_back(node.subscribe("/" + *name_it + "/tf", 10, &create_driver::ViconStream::callback, &vicon[*name_it]));
	}

	// ROS Publishers
	ros::Publisher vel = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	// ROS loop
	ros::Rate loop_rate(250); // 250 Hz

	ros::spinOnce();
	/// initial poses from Vicon///
	pose2D q10; q10.x = vicon[robot_names[0]].x();q10.y = vicon[robot_names[0]].y();q10.h = vicon[robot_names[0]].theta();
	pose2D q20; q20.x = vicon[robot_names[1]].x();q20.y = vicon[robot_names[1]].y();q20.h = vicon[robot_names[1]].theta();
	pose2D q30; q30.x = vicon[robot_names[2]].x();q30.y = vicon[robot_names[2]].y();q30.h = vicon[robot_names[2]].theta();
		
	/// initlal offests /// 
	double k = 0.8; // radius
	int mm = 3; 
	pose2D q1off; q1off.x = k*cos(1*PI/mm); q1off.y = k*sin(1*PI/mm); q1off.h = 0; 
	pose2D q2off; q2off.x = k*cos(3*PI/mm); q2off.y = k*sin(3*PI/mm); q2off.h = 0; 
	pose2D q3off; q3off.x = k*cos(5*PI/mm); q3off.y = k*sin(5*PI/mm); q3off.h = 0; 
        /// end of initial offsets ///

	double df12 = sqrt((q1off.x - q2off.x)*(q1off.x - q2off.x) + (q1off.y - q2off.y)*(q1off.y - q2off.y));
	double df13 = sqrt((q1off.x - q3off.x)*(q1off.x - q3off.x) + (q1off.y - q3off.y)*(q1off.y - q3off.y));
	double df23 = sqrt((q2off.x - q3off.x)*(q2off.x - q3off.x) + (q2off.y - q3off.y)*(q2off.y - q3off.y));
	
	pose2D q10_v; pose2D q20_v; pose2D q30_v; 
	Vector2d p12; Vector2d p13; Vector2d p23; 
	double d12=0; double d13=0; double d23=0; 
	

	Wheel vd;
	vd.l_v = 0.8;
	vd.a_v = 0;

	/// initial control inputs ///
	Wheel MyVel[3];
	MyVel[0].l_v = 0;
	MyVel[0].a_v = 0;

	MyVel[1].l_v = 0;
	MyVel[1].a_v = 0;


	MyVel[2].l_v = 0;
	MyVel[2].a_v = 0;

	/////////////////////////////

	control_formation r1;
	control_formation r2;
	control_formation r3;

	MatrixXd A(3,3); // adjacency matrix
	A <<0,1,1,
		1,0,1,
		1,1,0;

	pose2D virtualGoal_1; pose2D virtualGoal_2; pose2D virtualGoal_3; // virtual goal
	point2D po1; point2D po2; point2D po3; // position of obstacle
	
	
	while (ros::ok())
	{
		// Retrieve Vicon Data
		ros::spinOnce();

		/* An instance of this code is run on each robot in the formation.
		 *
		 * You can access each robot's position and oreintation through the following statements:
		 * vicon[robot_names[i]].x()
		 * vicon[robot_names[i]].y()
		 * vicon[robot_names[i]].theta()
		 * 
		 * The index of the robot running the instance of the code is contain in the "index" 
		 * variable. For example, if you wanted to get the distance of this robot from all of its
		 * neighbors you could run the following:
		 * double dist[num_robots];
		 * for (int i = 0; i < num_robots; i++) {
		 * 		dist[i] = sqrt(pow(vicon[robot_names[i]].x() - vicon[robot_names[index]].x(),2) 
		 * 				     + pow(vicon[robot_names[i]].y() - vicon[robot_names[index]].y(),2));
		 * }
		 */

		 /// virtual poses    
		q10_v.x = q10.x + q1off.x; q10_v.y = q10.y + q1off.y; q10_v.h = q10.h + q1off.h;
		q20_v.x = q20.x + q2off.x; q20_v.y = q20.y + q2off.y; q20_v.h = q20.h + q2off.h;
		q30_v.x = q30.x + q3off.x; q30_v.y = q30.y + q3off.y; q30_v.h = q30.h + q3off.h;
		
		int kk = 2;
		///
		/// virtual goals ///
		virtualGoal_1.x = (q20_v.x + q30_v.x)/kk;
		virtualGoal_1.y = (q20_v.y + q30_v.y)/kk;
		virtualGoal_1.h = (q20_v.h + q30_v.h)/kk;

		virtualGoal_2.x = (q10_v.x + q30_v.x)/kk;
		virtualGoal_2.y = (q10_v.y + q30_v.y)/kk;
		virtualGoal_2.h = (q10_v.h + q30_v.h)/kk;

		virtualGoal_3.x = (q20_v.x + q10_v.x)/kk;
		virtualGoal_3.y = (q20_v.y + q10_v.y)/kk;
		virtualGoal_3.h = (q20_v.h + q10_v.h)/kk;

		//////////////////////////////////
	
		p12(0) = q10.x - q20.x; p12(1) = q10.y - q20.y;
		p13(0) = q10.x - q30.x; p13(1) = q10.y - q30.y;
		
		
		p23(0) = q20.x - q30.x; p23(1) = q20.y - q30.y;
		
		/// Distance between robots ///
		d12 = p12.norm();
		d13 = p13.norm();
		

		d23 = p23.norm();
		

		//cout << "e12 = " << d12 - df12 << endl;
		//cout << "e13 = " << d13 - df13 << endl;
		//cout << "e23 = " << d23 - df23 << endl;
 
 		double d1[2] = {d12,d13};
 		int nneighbor = std::min_element(d1,d1+2) - d1;

 		switch (nneighbor) {
 			case 0:
 				po1.x = q20.x;
				po1.y = q20.y;
 				break;
 			case 1:
 				po1.x = q30.x;
				po1.y = q30.y;
 				break;
 		}

 		double d2[2] = {d12,d23};
 		nneighbor = std::min_element(d2,d2+2) - d2;

 		switch (nneighbor) {
 			case 0:
 				po2.x = q10.x;
				po2.y = q10.y;
 				break;
 			case 1:
 				po2.x = q30.x;
				po2.y = q30.y;
 				break;
 		}

 		double d3[2] = {d13,d23};
 		nneighbor = std::min_element(d3,d3+2) - d3;

 		switch (nneighbor) {
 			case 0:
 				po3.x = q10.x;
				po3.y = q10.y;
 				break;
 			case 1:
 				po3.x = q20.x;
				po3.y = q20.y;
 				break;
 		}


		MyVel[0] = r1.switching_w_obstacle(q10_v, virtualGoal_1, q10, po1, vd);
		q10.x = vicon[robot_names[0]].x();
		q10.y = vicon[robot_names[0]].y();
		q10.h = vicon[robot_names[0]].theta();
		q10.h = r1.between2PI(q10.h);

		MyVel[1] = r2.switching_w_obstacle(q20_v, virtualGoal_2, q20, po2, vd);
		q20.x = vicon[robot_names[1]].x();
		q20.y = vicon[robot_names[1]].y();
		q20.h = vicon[robot_names[1]].theta();
		q20.h = r2.between2PI(q20.h);

		MyVel[2] = r3.switching_w_obstacle(q30_v, virtualGoal_3, q30, po3, vd);
		q30.x = vicon[robot_names[2]].x();
		q30.y = vicon[robot_names[2]].y();
		q30.h = vicon[robot_names[2]].theta();
		q30.h = r3.between2PI(q30.h);

		
		if ((MyVel[0].l_v-MyVel[1].l_v)<=0.008 && (MyVel[0].l_v-MyVel[2].l_v)<=0.008)
		{
			vd.l_v = vd.l_v - 0.01;
		}
		/*
		if (vd.l_v<=0)
		{
			vd.l_v = 0;
		}
		*/

		double v = MyVel[index].l_v*300; // linear velocity output
		double w = MyVel[index].a_v; // angular velocity output

		
		// Send wheel velocities to driver
		geometry_msgs::Twist msg;
		msg.linear.x = v;
		msg.linear.y = 0;
		msg.linear.z = 0;
		msg.angular.x = 0;
		msg.angular.y = 0;
		msg.angular.z = w;
		vel.publish(msg);		
		
		loop_rate.sleep();
		new_msg = 0;
	}
	
	return 0;
}
