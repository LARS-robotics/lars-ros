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
//#include <tchar.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

#include "jingfu/control_formation.h"
#include <Eigen/Core>
#include <Eigen/Dense>
using Eigen::MatrixXd;
using namespace Eigen;

using namespace std;
using std::setw;
using std::setprecision;

double joy_x,joy_y,joy_r; 
double joy_a,joy_b,joy_l;
int new_msg=0;
sensor_msgs::Joy joy_msg_in;
geometry_msgs::Vector3 v3_msg; 
int shape = 1;  // triangle = 1; line = 2

void joy_callback(const sensor_msgs::Joy joy_msg_in)
{
	//Take in joystick
	joy_y=joy_msg_in.axes[3];
	joy_x=joy_msg_in.axes[4];
	joy_r=joy_msg_in.axes[5];

	joy_a=joy_msg_in.axes[0];
	joy_b=joy_msg_in.axes[1];
	joy_l=joy_msg_in.axes[2];


	//l1=joy_msg_in.buttons[0];
	//l2=joy_msg_in.buttons[1];
	//l3=joy_msg_in.buttons[2];	


	printf("hello joy\n");
	printf("hello joy\n");
	//Take in time
	//msg_time=(double)ros::Time::now().toNSec();
    	if (joy_x !=0||joy_y !=0)
	{
		new_msg=1;
	}	
	else if (joy_r !=1)
	{
		new_msg=0;
	}
	else
	{
		//new_msg=0;
		ROS_INFO("Waiting for joystick message");
	}

	if(joy_a !=0||joy_b !=0)
	{
		shape = 2;
	}
	else if (joy_l!=1)
	{
		shape = 1;
	}

}

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
	pose2D q40; q40.x = vicon[robot_names[3]].x();q40.y = vicon[robot_names[3]].y();q40.h = vicon[robot_names[3]].theta();
	pose2D quad; quad.x = vicon[robot_names[4]].x();quad.y = vicon[robot_names[4]].y();quad.h = vicon[robot_names[4]].theta();
	
	/// // Desired Formation shape or initlal offests /// 
	double k = 0.7; // radius
	int mm = 3; 
	
	pose2D q1off; pose2D q2off; pose2D q3off; pose2D q4off;
	
	
	pose2D q10_v; pose2D q20_v; pose2D q30_v; pose2D q40_v; 
	Vector2d p12; Vector2d p13; Vector2d p14; 
	Vector2d p23; Vector2d p24; Vector2d p34;
	double d12=0; double d13=0; double d14; double d23=0; double d24; double d34;
	
	Wheel vd;
	vd.l_v = 0.8;
	vd.a_v = 0;

	/// initial control inputs ///
	Wheel MyVel[4];
	MyVel[0].l_v = 0;
	MyVel[0].a_v = 0;

	MyVel[1].l_v = 0;
	MyVel[1].a_v = 0;

	MyVel[2].l_v = 0;
	MyVel[2].a_v = 0;

	MyVel[3].l_v = 0;
	MyVel[3].a_v = 0;

	/////////////////////////////
	control_formation r1;
	control_formation r2;
	control_formation r3;
	control_formation r4;
	////////////////////////////
	

	MatrixXd A(4,4); // adjacency matrix

	MatrixXd D(4,4); // Degree matrix


	pose2D virtualGoal_1; pose2D virtualGoal_2; pose2D virtualGoal_3; pose2D virtualGoal_4; // virtual goal
	point2D po1; point2D po2; point2D po3; point2D po4;// position of obstacle
	
	ofstream myfile_1("/home/heckert/utd-lars/src/jingfu/data/robot_1.txt",ios::out | ios::binary);
	ofstream myfile_2("/home/heckert/utd-lars/src/jingfu/data/robot_2.txt",ios::out | ios::binary);
	ofstream myfile_3("/home/heckert/utd-lars/src/jingfu/data/robot_3.txt",ios::out | ios::binary);
	ofstream myfile_4("/home/heckert/utd-lars/src/jingfu/data/robot_4.txt",ios::out | ios::binary);
	ofstream myfile_5("/home/heckert/utd-lars/src/jingfu/data/quad.txt",ios::out | ios::binary);
	int num=1;	
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

	switch (shape) 
	{
  		case 1:
			q1off.x = k*cos(1*PI/4); q1off.y = k*sin(1*PI/4); q1off.h = 0; 
			q2off.x = k*cos(3*PI/4); q2off.y = k*sin(3*PI/4); q2off.h = 0; 
			q3off.x = k*cos(5*PI/4); q3off.y = k*sin(5*PI/4); q3off.h = 0; 
			q4off.x = k*cos(7*PI/4); q4off.y = k*sin(7*PI/4); q4off.h = 0; 	    		
	    		break;
  		case 2:
	    		q1off.x = 1; q1off.y = 0; q1off.h = 0; 
			q2off.x = 2; q2off.y = 0; q2off.h = 0; 
			q3off.x = 3; q3off.y = 0; q3off.h = 0; 
			q4off.x = 4; q4off.y = 0; q4off.h = 0; 
	    		break;
  	}

	/// end of initial offsets ///

	double df12 = sqrt((q1off.x - q2off.x)*(q1off.x - q2off.x) + (q1off.y - q2off.y)*(q1off.y - q2off.y));
	double df13 = sqrt((q1off.x - q3off.x)*(q1off.x - q3off.x) + (q1off.y - q3off.y)*(q1off.y - q3off.y));
	double df14 = sqrt((q1off.x - q4off.x)*(q1off.x - q4off.x) + (q1off.y - q4off.y)*(q1off.y - q4off.y));
	
	double df23 = sqrt((q2off.x - q3off.x)*(q2off.x - q3off.x) + (q2off.y - q3off.y)*(q2off.y - q3off.y));
	double df24 = sqrt((q2off.x - q4off.x)*(q2off.x - q4off.x) + (q2off.y - q4off.y)*(q2off.y - q4off.y));
	
	double df34 = sqrt((q3off.x - q4off.x)*(q3off.x - q4off.x) + (q3off.y - q4off.y)*(q3off.y - q4off.y));


		/// virtual poses ///  
		q10_v.x = q10.x + q1off.x; q10_v.y = q10.y + q1off.y; q10_v.h = q10.h + q1off.h;
		q20_v.x = q20.x + q2off.x; q20_v.y = q20.y + q2off.y; q20_v.h = q20.h + q2off.h;
		q30_v.x = q30.x + q3off.x; q30_v.y = q30.y + q3off.y; q30_v.h = q30.h + q3off.h;
		q40_v.x = q40.x + q4off.x; q40_v.y = q40.y + q4off.y; q40_v.h = q40.h + q4off.h;
		
		int kk = 2;
		if(num % 4 == 0)
		{
			A <<0,1,1,1,
	    		    1,0,1,1,
	                    1,1,0,1,
	                    1,1,1,0;

			D <<3,0,0,0,
			    0,3,0,0,
			    0,0,3,0,
			    0,0,0,3;
		}
		else if(num % 4 == 1)
		{
			A <<0,1,0,1,
	    		    1,0,1,0,
	                    0,1,0,1,
	                    1,0,1,0;

			D <<2,0,0,0,
			    0,2,0,0,
			    0,0,2,0,
			    0,0,0,2;
		}
		else if(num % 4 == 2)
		{
			A <<0,1,1,0,
	    		    1,0,0,1,
	                    1,0,0,1,
	                    0,1,1,0;

			D <<2,0,0,0,
			    0,2,0,0,
			    0,0,2,0,
			    0,0,0,2;
		}
		else if(num % 4 == 3)
		{
			A <<0,1,0,0,
	    		    0,0,1,0,
	                    0,0,0,1,
	                    1,0,0,0;

			D <<1,0,0,0,
			    0,1,0,0,
			    0,0,1,0,
			    0,0,0,1;
		}

		num++;

		/// virtual goals ///

		MatrixXd virtualGoal(3,4);
		virtualGoal(0,0) = q10_v.x; virtualGoal(0,1) = q20_v.x; virtualGoal(0,2) = q30_v.x; virtualGoal(0,3) = q40_v.x;
		virtualGoal(1,0) = q10_v.y; virtualGoal(1,1) = q20_v.y; virtualGoal(1,2) = q30_v.y; virtualGoal(1,3) = q40_v.y;
		virtualGoal(2,0) = q10_v.h; virtualGoal(2,1) = q20_v.h; virtualGoal(2,2) = q30_v.h; virtualGoal(2,3) = q40_v.h;
		
		MatrixXd virtualGoal_temp(3,4);
		virtualGoal_temp = virtualGoal*A*D.inverse(); /// calculation virtual goal position ///

		virtualGoal_1.x = virtualGoal_temp(0,0);
		virtualGoal_1.y = virtualGoal_temp(1,0);
		virtualGoal_1.h = virtualGoal_temp(2,0);

		virtualGoal_2.x = virtualGoal_temp(0,1);
		virtualGoal_2.y = virtualGoal_temp(1,1);
		virtualGoal_2.h = virtualGoal_temp(2,1);

		virtualGoal_3.x = virtualGoal_temp(0,2);
		virtualGoal_3.y = virtualGoal_temp(1,2);
		virtualGoal_3.h = virtualGoal_temp(2,2);

		virtualGoal_4.x = virtualGoal_temp(0,3);
		virtualGoal_4.y = virtualGoal_temp(1,3);
		virtualGoal_4.h = virtualGoal_temp(2,3);
	
		//////////////////////////////////
		p12(0) = q10.x - q20.x; p12(1) = q10.y - q20.y;
		p13(0) = q10.x - q30.x; p13(1) = q10.y - q30.y;
		p14(0) = q10.x - q40.x; p14(1) = q10.y - q40.y;
		
		p23(0) = q20.x - q30.x; p23(1) = q20.y - q30.y;
		p24(0) = q20.x - q40.x; p24(1) = q20.y - q40.y;
		
		p34(0) = q30.x - q40.x; p34(1) = q30.y - q40.y;
		
		/// Distance between robots ///
		d12 = p12.norm();
		d13 = p13.norm();
		d14 = p14.norm();
		
		d23 = p23.norm();
		d24 = p24.norm();
		
		d34 = p34.norm();
		//////////////////////////////

		double d1[3] = {d12,d13,d14};
 		int nneighbor = std::min_element(d1,d1+3) - d1;

 		switch (nneighbor) {
 			case 0:
 				po1.x = q20.x;
				po1.y = q20.y;
 				break;
 			case 1:
 				po1.x = q30.x;
				po1.y = q30.y;
 				break;
 			case 2:
 				po1.x = q40.x;
				po1.y = q40.y;
 				break;
 			
 		}
 		double d2[3] = {d12,d23,d24};
 		nneighbor = std::min_element(d2,d2+3) - d2;

 		switch (nneighbor) {
 			case 0:
 				po2.x = q10.x;
				po2.y = q10.y;
 				break;
 			case 1:
 				po2.x = q30.x;
				po2.y = q30.y;
 				break;
 			case 2:
 				po2.x = q40.x;
				po2.y = q40.y;
 				break;
 		}

 		double d3[3] = {d13,d23,d34};
 		nneighbor = std::min_element(d3,d3+3) - d3;

 		switch (nneighbor) {
 			case 0:
 				po3.x = q10.x;
				po3.y = q10.y;
 				break;
 			case 1:
 				po3.x = q20.x;
				po3.y = q20.y;
 				break;
 			case 2:
 				po3.x = q40.x;
				po3.y = q40.y;
 				break;
 		}

 		double d4[3] = {d14,d24,d34};
 		nneighbor = std::min_element(d4,d4+3) - d4;

 		switch (nneighbor) {
 			case 0:
 				po4.x = q10.x;
				po4.y = q10.y;
 				break;
 			case 1:
 				po4.x = q20.x;
				po4.y = q20.y;
 				break;
 			case 2:
 				po4.x = q30.x;
				po4.y = q30.y;
 				break;
 			
 		}

		if (new_msg==0)
		{
			MyVel[0] = r1.switching_w_obstacle(q10_v, virtualGoal_1, q10, po1, vd); // robot 1
			MyVel[1] = r2.switching_w_obstacle(q20_v, virtualGoal_2, q20, po2, vd); // robot 2
			MyVel[2] = r3.switching_w_obstacle(q30_v, virtualGoal_3, q30, po3, vd); // robot 3
			MyVel[3] = r4.switching_w_obstacle(q40_v, virtualGoal_4, q40, po4, vd); // robot 3
		}
		else
		{
			

			virtualGoal_1.x = quad.x - q1off.x;	
			virtualGoal_1.y = quad.y - q1off.y;
			
			virtualGoal_2.x = quad.x - q2off.x;
			virtualGoal_2.y = quad.y - q2off.y;
			
			virtualGoal_3.x = quad.x - q3off.x;
			virtualGoal_3.y = quad.y - q3off.y;
			
			virtualGoal_4.x = quad.x - q4off.x;
			virtualGoal_4.y = quad.y - q4off.y;
			
/*	
			MyVel[0] = r1.singleRobot(virtualGoal_1, q10); // robot 1
			MyVel[0].l_v = 0.5*MyVel[0].l_v;
			MyVel[0].a_v = 0.5*MyVel[0].a_v; 
			MyVel[1] = r2.singleRobot(virtualGoal_2, q20); // robot 2
			int gain=3; 
			MyVel[1].l_v = gain*MyVel[1].l_v;
			MyVel[1].a_v = gain*MyVel[1].a_v; 
			MyVel[2] = r3.singleRobot(virtualGoal_3, q30); // robot 3
			MyVel[2].l_v = gain*MyVel[2].l_v;
			MyVel[2].a_v = gain*MyVel[2].a_v;

			MyVel[3] = r4.singleRobot(virtualGoal_4, q40); // robot 4
			MyVel[3].l_v = gain*MyVel[3].l_v;
			MyVel[3].a_v = gain*MyVel[3].a_v;
*/

			MyVel[0] = r1.single_w_obstacle(virtualGoal_1, q10,po1); // robot 1
			MyVel[1] = r2.single_w_obstacle(virtualGoal_2, q20,po2); // robot 2
			int gain=1; 
			MyVel[1].l_v = gain*MyVel[1].l_v;
			MyVel[1].a_v = gain*MyVel[1].a_v; 
			MyVel[2] = r3.single_w_obstacle(virtualGoal_3, q30,po3); // robot 3
			MyVel[2].l_v = gain*MyVel[2].l_v;
			MyVel[2].a_v = gain*MyVel[2].a_v;
			MyVel[3] = r4.single_w_obstacle(virtualGoal_4, q40,po4); // robot 4
			MyVel[3].l_v = gain*MyVel[3].l_v;
			MyVel[3].a_v = gain*MyVel[3].a_v;


		}
		

		q10.x = vicon[robot_names[0]].x();
		q10.y = vicon[robot_names[0]].y();
		q10.h = vicon[robot_names[0]].theta();
		q10.h = r1.between2PI(q10.h);

		q20.x = vicon[robot_names[1]].x();
		q20.y = vicon[robot_names[1]].y();
		q20.h = vicon[robot_names[1]].theta();
		q20.h = r2.between2PI(q20.h);

		q30.x = vicon[robot_names[2]].x();
		q30.y = vicon[robot_names[2]].y();
		q30.h = vicon[robot_names[2]].theta();
		q30.h = r3.between2PI(q30.h);


		q40.x = vicon[robot_names[3]].x();
		q40.y = vicon[robot_names[3]].y();
		q40.h = vicon[robot_names[3]].theta();
		q40.h = r3.between2PI(q40.h);

		// virtual leader or waypoint //
		quad.x = vicon[robot_names[4]].x();
		quad.y = vicon[robot_names[4]].y();
		quad.h = vicon[robot_names[4]].theta();

				
		if (abs(d12-df12)<=0.05 && abs(d13-df13)<=0.05 && abs(d23-df23)<=0.05)
		//if ((MyVel[0].l_v-MyVel[1].l_v)<=0.008 && (MyVel[0].l_v-MyVel[2].l_v)<=0.008)
		{
			vd.l_v = vd.l_v - 0.01;
		}
		
		if (vd.l_v<=0)
		{
			vd.l_v = 0;
		}
		

	myfile_1 <<q10.x<<setw(20)<<q10.y<<setw(20)<<q10.h<<setw(20)<<MyVel[0].l_v<<setw(20)<<MyVel[0].a_v<<setw(20)<<r1.zeta<<endl;
	myfile_2 <<q20.x<<setw(20)<<q20.y<<setw(20)<<q20.h<<setw(20)<<MyVel[1].l_v<<setw(20)<<MyVel[1].a_v<<setw(20)<<r2.zeta<<endl;
	myfile_3 <<q30.x<<setw(20)<<q30.y<<setw(20)<<q30.h<<setw(20)<<MyVel[2].l_v<<setw(20)<<MyVel[2].a_v<<setw(20)<<r3.zeta<<endl;
	myfile_4 <<q40.x<<setw(20)<<q40.y<<setw(20)<<q40.h<<setw(20)<<MyVel[3].l_v<<setw(20)<<MyVel[3].a_v<<setw(20)<<r4.zeta<<endl;
	myfile_5 <<quad.x<<setw(20)<<quad.y<<setw(20)<<quad.h<<endl;

		double v = MyVel[index].l_v*250; // linear velocity output
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
	}
	myfile_1.close();
	myfile_2.close();
	myfile_3.close();
	myfile_4.close();
	myfile_5.close();
	return 0;
}
