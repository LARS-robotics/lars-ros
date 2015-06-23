// ROS includes
#include "ros/ros.h"
#include "ros/assert.h"
#include "dynamic_reconfigure/server.h"
#include "create_driver/vicon_driver.h"
#include "geometry_msgs/Twist.h"

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
#include "jingfu/control_formation.h"
//#include "mobileRobotSystem.h"
#include <Eigen/Core>
#include <Eigen/Dense>
using Eigen::MatrixXd;
#include <iomanip>
using std::setw;
#include <cmath>
using namespace std;
using namespace Eigen;

// Constants
//const double PI = 3.1415926535;

int main(int argc, char **argv)
{
	// ROS Initalization
	ros::init(argc, argv, "control");
	
	ros::NodeHandle n;
	ros::NodeHandle private_n("~");
	
	// ROS Parameters

	// List of robot names
	vector<string> robot_names;
	XmlRpc::XmlRpcValue robot_list;
	private_n.getParam("robot_list", robot_list);
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
	private_n.param<std::string>("robotname", robot_name, "defaultname");

	int index = find(robot_names.begin(), robot_names.end(), robot_name) - robot_names.begin();

	// ROS Subscribers
	map<string, create_driver::ViconStream> vicon;
	vector<string>::iterator name_it;
	vector<ros::Subscriber> sub;
	for (name_it = robot_names.begin(); name_it != robot_names.end(); name_it++)
	{
		vicon.insert(pair<string, create_driver::ViconStream>(*name_it, create_driver::ViconStream()));
		sub.push_back(n.subscribe("/" + *name_it + "/tf", 10, &create_driver::ViconStream::callback, &vicon[*name_it]));
	}

	// ROS Publishers
	ros::Publisher vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	// ROS loop
	ros::Rate loop_rate(250); // 250 Hz

	ros::spinOnce();
	/// initial poses from Vicon///
	//Vector3d q10(0,0,0);
	//Vector3d q20(3,1,0);
	//Vector3d q30(1,2,0);
	pose2D q10; q10.x = vicon[robot_names[0]].x();q10.y = vicon[robot_names[0]].y();q10.h = vicon[robot_names[0]].theta();
	pose2D q20; q20.x = vicon[robot_names[1]].x();q20.y = vicon[robot_names[1]].y();q20.h = vicon[robot_names[1]].theta();
	pose2D q30; q30.x = vicon[robot_names[2]].x();q30.y = vicon[robot_names[2]].y();q30.h = vicon[robot_names[2]].theta();
	pose2D q40; q40.x = vicon[robot_names[3]].x();q40.y = vicon[robot_names[3]].y();q40.h = vicon[robot_names[3]].theta();
	// pose2D q50; q50.x = vicon[robot_names[4]].x();q50.y = vicon[robot_names[4]].y();q50.h = vicon[robot_names[4]].theta();
	
	/// initlal offests /// 
	double k = 1.25; // radius
	//Vector3d q1off(k*cos(1*PI/3), k*sin(1*PI/3),0);
    //Vector3d q2off(k*cos(3*PI/3), k*sin(3*PI/3),0);
    //Vector3d q3off(k*cos(5*PI/3), k*sin(5*PI/3),0); 
	pose2D q1off; q1off.x = k*cos(1*PI/4); q1off.y = k*sin(1*PI/4); q1off.h = 0; 
	pose2D q2off; q2off.x = k*cos(3*PI/4); q2off.y = k*sin(2*PI/4); q2off.h = 0; 
	pose2D q3off; q3off.x = k*cos(5*PI/4); q3off.y = k*sin(5*PI/4); q3off.h = 0; 
	pose2D q4off; q4off.x = k*cos(7*PI/4); q4off.y = k*sin(7*PI/4); q4off.h = 0; 
	// pose2D q5off; q5off.x = k*cos(10*PI/5); q5off.y = k*sin(10*PI/5); q5off.h = 0; 
    /// end of initial offsets ///

	double df12 = sqrt((q1off.x - q2off.x)*(q1off.x - q2off.x) + (q1off.y - q2off.y)*(q1off.y - q2off.y));
	double df13 = sqrt((q1off.x - q3off.x)*(q1off.x - q3off.x) + (q1off.y - q3off.y)*(q1off.y - q3off.y));
	double df14 = sqrt((q1off.x - q4off.x)*(q1off.x - q4off.x) + (q1off.y - q4off.y)*(q1off.y - q4off.y));
	// double df15 = sqrt((q1off.x - q5off.x)*(q1off.x - q5off.x) + (q1off.y - q5off.y)*(q1off.y - q5off.y));
	
	double df23 = sqrt((q2off.x - q3off.x)*(q2off.x - q3off.x) + (q2off.y - q3off.y)*(q2off.y - q3off.y));
	double df24 = sqrt((q2off.x - q4off.x)*(q2off.x - q4off.x) + (q2off.y - q4off.y)*(q2off.y - q4off.y));
	// double df25 = sqrt((q2off.x - q5off.x)*(q2off.x - q5off.x) + (q2off.y - q5off.y)*(q2off.y - q5off.y));

	double df34 = sqrt((q3off.x - q4off.x)*(q3off.x - q4off.x) + (q3off.y - q4off.y)*(q3off.y - q4off.y));
	// double df35 = sqrt((q3off.x - q5off.x)*(q3off.x - q5off.x) + (q3off.y - q5off.y)*(q3off.y - q5off.y));

	// double df45 = sqrt((q4off.x - q5off.x)*(q4off.x - q5off.x) + (q4off.y - q5off.y)*(q4off.y - q5off.y));

	pose2D q10_v; pose2D q20_v; pose2D q30_v; pose2D q40_v; //pose2D q50_v; 
	Vector2d p12; Vector2d p13; Vector2d p14; //Vector2d p15; 
	Vector2d p23; Vector2d p24; //Vector2d p25; 
	Vector2d p34; //Vector2d p35;
	//Vector2d p45;
	double d12=0; double d13=0; double d14=0; //double d15=0; 
	double d23=0; double d24=0; //double d25=0; 
	double d34=0; //double d35=0; 
	//double d45=0;

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

	// MyVel[4].l_v = 0;
	// MyVel[4].a_v = 0;
	/////////////////////////////

	control_formation r1;
	//mobileRobotSystem TestSys1;
	control_formation r2;
	//mobileRobotSystem TestSys2;
	control_formation r3;
	//mobileRobotSystem TestSys3;
	control_formation r4;
	//mobileRobotSystem TestSys4;
	// control_formation r5;
	// mobileRobotSystem TestSys5;
	double dt = 0.1;

	MatrixXd A(4,4); // adjacency matrix
	A <<0,1,1,1,
		1,0,1,1,
		1,1,0,1,
		1,1,1,0;
		// 1,1,1,1,0;


	pose2D virtualGoal_1; pose2D virtualGoal_2; pose2D virtualGoal_3; pose2D virtualGoal_4;

	point2D po1; point2D po2; point2D po3; point2D po4;// position of obstacle
		
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
		q40_v.x = q40.x + q4off.x; q40_v.y = q40.y + q4off.y; q40_v.h = q40.h + q4off.h;
		//q50_v.x = q50.x + q5off.x; q50_v.y = q50.y + q5off.y; q50_v.h = q50.h + q5off.h;
		int kk = 3;
		///
		/// virtual goals ///
		virtualGoal_1.x = (q20_v.x + q30_v.x + q40_v.x)/kk;// + q50_v.x;
		virtualGoal_1.y = (q20_v.y + q30_v.y + q40_v.y)/kk;// + q50_v.y;
		virtualGoal_1.h = (q20_v.h + q30_v.h + q40_v.h)/kk;// + q50_v.h;

		virtualGoal_2.x = (q10_v.x + q30_v.x + q40_v.x)/kk;// + q50_v.x;
		virtualGoal_2.y = (q10_v.y + q30_v.y + q40_v.y)/kk;// + q50_v.y;
		virtualGoal_2.h = (q10_v.h + q30_v.h + q40_v.h)/kk;// + q50_v.h;

		virtualGoal_3.x = (q20_v.x + q10_v.x + q40_v.x)/kk;// + q50_v.x;
		virtualGoal_3.y = (q20_v.y + q10_v.y + q40_v.y)/kk;// + q50_v.y;
		virtualGoal_3.h = (q20_v.h + q10_v.h + q40_v.h)/kk;// + q50_v.h;

		virtualGoal_4.x = (q20_v.x + q30_v.x + q10_v.x)/kk; //+ q50_v.x;
		virtualGoal_4.y = (q20_v.y + q30_v.y + q10_v.y)/kk;//+ q50_v.y;
		virtualGoal_4.h = (q20_v.h + q30_v.h + q10_v.h)/kk;// + q50_v.h;

		// virtualGoal_5.x = q20_v.x + q30_v.x + q40_v.x + q10_v.x;
		// virtualGoal_5.y = q20_v.y + q30_v.y + q40_v.y + q10_v.y;
		// virtualGoal_5.h = q20_v.h + q30_v.h + q40_v.h + q10_v.h;
		//////////////////////////////////
	
		p12(0) = q10.x - q20.x; p12(1) = q10.y - q20.y;
		p13(0) = q10.x - q30.x; p13(1) = q10.y - q30.y;
		p14(0) = q10.x - q40.x; p14(1) = q10.y - q40.y;
		//p15(0) = q10.x - q50.x; p15(1) = q10.y - q50.y;
		
		p23(0) = q20.x - q30.x; p23(1) = q20.y - q30.y;
		p24(0) = q20.x - q40.x; p24(1) = q20.y - q40.y;
		//p25(0) = q20.x - q50.x; p25(1) = q20.y - q50.y;

		p34(0) = q30.x - q40.x; p34(1) = q30.y - q40.y;
		//p35(0) = q30.x - q50.x; p35(1) = q30.y - q50.y;

		//p35(0) = q40.x - q50.x; p35(1) = q40.y - q50.y;

		/// Distance between robots ///
		d12 = p12.norm();
		d13 = p13.norm();
		d14 = p14.norm();
		//d15 = p15.norm();

		d23 = p23.norm();
		d24 = p24.norm();
		//d25 = p25.norm();

		d34 = p34.norm();
		//d35 = p35.norm();

		//d45 = p45.norm();

		cout << "e12 = " << d12 - df12 << endl;
		cout << "e13 = " << d13 - df13 << endl;
		cout << "e23 = " << d23 - df23 << endl;
 
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
 			// case 4:
 			// 	po1.x = q50.x;
				// po1.y = q50.y;
 			// 	break;
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
 			// case 4:
 			// 	po2.x = q50.x;
				// po2.y = q50.y;
 			// 	break;
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
 			// case 4:
 			// 	po3.x = q50.x;
				// po3.y = q50.y;
 			// 	break;
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
 			// case 4:
 			// 	po4.x = q50.x;
				// po4.y = q50.y;
 			// 	break;
 		}

 		// double d5[4] = {d15,d25,d35,d45};
 		// nneighbor = std::min_element(d5,d5+4) - d5;

 		// switch (nneighbor) {
 		// 	case 1:
 		// 		po5.x = q10.x;
			// 	po5.y = q10.y;
 		// 		break;
 		// 	case 2:
 		// 		po5.x = q20.x;
			// 	po5.y = q20.y;
 		// 		break;
 		// 	case 3:
 		// 		po5.x = q30.x;
			// 	po5.y = q30.y;
 		// 		break;
 		// 	case 4:
 		// 		po5.x = q40.x;
			// 	po5.y = q40.y;

 		// 		break;
 		// }



		MyVel[0] = r1.nonswitching_w_obstacle(q10_v, virtualGoal_1, q10, po1, vd);
		//q10 = TestSys1.DifftypeSys(dt, q10, MyVel[0]);
		q10.x = vicon[robot_names[0]].x();
		q10.y = vicon[robot_names[0]].y();
		q10.h = vicon[robot_names[0]].theta();
		q10.h = r1.between2PI(q10.h);

		MyVel[1] = r2.nonswitching_w_obstacle(q20_v, virtualGoal_2, q20, po2, vd);
		//q20 = TestSys2.DifftypeSys(dt, q20, MyVel[1]);
		q20.x = vicon[robot_names[1]].x();
		q20.y = vicon[robot_names[1]].y();
		q20.h = vicon[robot_names[1]].theta();
		q20.h = r2.between2PI(q20.h);

		MyVel[2] = r3.nonswitching_w_obstacle(q30_v, virtualGoal_3, q30, po3, vd);
		//q30 = TestSys3.DifftypeSys(dt, q30, MyVel[2]);
		q30.x = vicon[robot_names[2]].x();
		q30.y = vicon[robot_names[2]].y();
		q30.h = vicon[robot_names[2]].theta();

		q30.h = r3.between2PI(q30.h);

		MyVel[3] = r4.nonswitching_w_obstacle(q40_v, virtualGoal_4, q40, po4, vd);
		//q30 = TestSys3.DifftypeSys(dt, q30, MyVel[2]);
		q40.x = vicon[robot_names[3]].x();
		q40.y = vicon[robot_names[3]].y();
		q40.h = vicon[robot_names[3]].theta();

		q40.h = r4.between2PI(q40.h);

		// MyVel[4] = r5.nonswitching_w_obstacle(q50_v, virtualGoal_5, q50, po5, vd);
		// //q30 = TestSys3.DifftypeSys(dt, q30, MyVel[2]);
		// q50.x = vicon[robot_names[4]].x();
		// q50.y = vicon[robot_names[4]].y();
		// q50.h = vicon[robot_names[4]].theta();

		// q50.h = r5.between2PI(q50.h);

		if ((MyVel[0].l_v-MyVel[1].l_v)<=0.008 && (MyVel[0].l_v-MyVel[2].l_v)<=0.008
			&& (MyVel[0].l_v-MyVel[3].l_v)<=0.008 )
		{
			vd.l_v = vd.l_v - 0.02;
		}

		if (vd.l_v<=0)
		{
			vd.l_v = 0;
		}


		cout<<"Current State x, y, theta"<<endl;
 		cout<<q10.x<<setw(20)<<q10.y<<setw(20)<<q10.h<<endl;
		cout<<q20.x<<setw(20)<<q20.y<<setw(20)<<q20.h<<endl;
		cout<<q30.x<<setw(20)<<q30.y<<setw(20)<<q30.h<<endl;
 		cout<<endl;
		/*
		cout<<"Current velocities" <<endl;
		cout<<MyVel[0].l_v<<setw(20)<<endl;
		cout<<MyVel[1].l_v<<setw(20)<<endl;
		cout<<MyVel[2].l_v<<setw(20)<<endl;
 		cout<<endl;
		*/


		double v = MyVel[index].l_v*100; // linear velocity output
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
	
	return 0;
}
