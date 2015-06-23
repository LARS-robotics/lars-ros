/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, The University of Texas at Dallas
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of The University of Texas at Dallas nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
 
/*
 * Filename: connectivity_control_node.cpp
 * 
 * Description: This file contains the ROS node that calculates the velocity commands
 * for an individual iRobot Create.
 * 
 * Log
 * ----
 * 2014-02-21 File created by Hazen Eckert
 *
 */


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

// Local Package includes
#include "connectivity_controller/ControlVariablesConfig.h"
#include "connectivity_controller/EigenConnectivityMsgs.h"
#include "connectivity_controller/ControlMsgs.h"

using namespace std;
using namespace Eigen;

// Constants
const double PI = 3.1415926535;
const double alphaOverBar = 5;
const double alphaUnderBar = 0;

// Connectivity variables
vector<double> trace_Mx;
vector<double> trace_My;

vector<double> control_input_CA_x;
vector<double> control_input_CA_y;

vector<double> control_input_FC_x;
vector<double> control_input_FC_y;

vector<double> control_input_CC_x;
vector<double> control_input_CC_y;

double detM;

// Dynamic reconfigure variables
double k = 0.5; // linear velocity gain
double ktheta = 5.0; // angular velocity gain
bool power = true;
double x_bar_desired = 0.25;
double y_bar_desired = 0.5;
double psi_desired = 3*PI/4;
double k_bar = 1;
double kconn = 1.0e-01,kca = 2.0e-02,kfc = 1.0;
bool formation = true;

// Callbacks
void ConnectCallback(const connectivity_controller::EigenConnectivityMsgs::ConstPtr& msg) {
	trace_Mx.assign(msg->trMx.begin(), msg->trMx.end());
	trace_My.assign(msg->trMy.begin(), msg->trMy.end());

	control_input_CA_x.assign(msg->u_ca_x.begin(), msg->u_ca_x.end());
	control_input_CA_y.assign(msg->u_ca_y.begin(), msg->u_ca_y.end());

	control_input_FC_x.assign(msg->u_fc_x.begin(), msg->u_fc_x.end());
	control_input_FC_y.assign(msg->u_fc_y.begin(), msg->u_fc_y.end());

	control_input_CC_x.assign(msg->u_cc_x.begin(), msg->u_cc_x.end());
	control_input_CC_y.assign(msg->u_cc_y.begin(), msg->u_cc_y.end());

	detM = msg->detM;
}

void reconfigureCallback(connectivity_controller::ControlVariablesConfig &config, uint32_t level) {
	k = config.k;
	ktheta = config.ktheta;
	power = config.power;
	x_bar_desired = config.x_bar_desired;
	y_bar_desired = config.y_bar_desired;
	psi_desired = config.psi_desired;
	k_bar = config.k_bar;
	kconn = config.kconn;
	kca = config.kca;
	kfc = config.kfc;
	formation = config.formation;
}

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


	// ROS Dynamic Reconfigure

	dynamic_reconfigure::Server<connectivity_controller::ControlVariablesConfig> server;
  	dynamic_reconfigure::Server<connectivity_controller::ControlVariablesConfig>::CallbackType f;
  	f = boost::bind(&reconfigureCallback, _1, _2);
 	server.setCallback(f);


	// ROS Subscribers
	map<string, create_driver::ViconStream> vicon;
	vector<string>::iterator name_it;
	vector<ros::Subscriber> sub;
	for (name_it = robot_names.begin(); name_it != robot_names.end(); name_it++)
	{
		vicon.insert(pair<string, create_driver::ViconStream>(*name_it, create_driver::ViconStream()));
		sub.push_back(n.subscribe("/" + *name_it + "/tf", 10, &create_driver::ViconStream::callback, &vicon[*name_it]));
	}

	sub.push_back(n.subscribe( "/connectivity", 10, ConnectCallback));

	// ROS Publishers
	ros::Publisher vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	ros::Publisher dia = n.advertise<connectivity_controller::ControlMsgs>("dia", 10);
	

	// ROS loop
	ros::Rate loop_rate(250); // 250 Hz
	
	while (ros::ok())
	{
		// Retrieve Vicon Data
		ros::spinOnce();

		if(trace_Mx.size() < num_robots) {
			ROS_WARN("Connectivity is not publishing large enough vectors: %d < numrobots", trace_Mx.size());
			continue;
		}

		MatrixXd S(2*num_robots, 3);

		for (int i = 0; i < num_robots; i++)
		{
			Vector3d row;
			double x = -(num_robots - 1)*vicon[robot_names[i]].y();

			for (int j = 0; j < num_robots; j++)
			{
				if (j != i) {
					x += vicon[robot_names[j]].y();
				}
			}

			row << 1, 0, x/num_robots;

			S.row(2*i) = row;

			x = (num_robots - 1)*vicon[robot_names[i]].x();

			for (int j = 0; j < num_robots; j++)
			{
				if (j != i) {
					x -= vicon[robot_names[j]].x();
				}

			}

			row << 0, 1, x/num_robots;

			S.row(2*i + 1) = row;
		}

		double x_bar = 0.0;
		for (int i = 0; i < num_robots; i++)
		{
			x_bar += vicon[robot_names[i]].x();
		}
		x_bar /= ((double)num_robots);

		double y_bar = 0.0;
		for (int i = 0; i < num_robots; i++)
		{
			y_bar += vicon[robot_names[i]].y();
		}
		y_bar /= ((double)num_robots);

		double psi = atan2(vicon[robot_names[num_robots/2 -1]].y() - vicon[robot_names[0]].y(),
							vicon[robot_names[num_robots/2 -1]].x() - vicon[robot_names[0]].x());

		if (psi > PI){
			psi = psi - 2*PI;
		} else if (psi < -PI){
			psi = psi + 2*PI;
		}
			
		MatrixXd eta(3,1);
		eta << 	(x_bar - x_bar_desired),
				(y_bar - y_bar_desired),
				(psi - psi_desired);


		double uconnx = (((pow(alphaOverBar,2)-pow(alphaUnderBar,2))
				     *(pow(detM, 2)-pow(alphaOverBar,2)))
				     /(pow(pow(detM, 2)-pow(alphaUnderBar,2),3))) 
				     *pow(detM,2)*trace_Mx[index];

		double ufx = control_input_FC_x[index];	// Formation control
		double ucx = control_input_CA_x[index];		// Collision avoidance

		double uconny = (((pow(alphaOverBar,2)-pow(alphaUnderBar,2))
				     *(pow(detM, 2)-pow(alphaOverBar,2)))
				     /(pow(pow(detM, 2)-pow(alphaUnderBar,2),3))) 
				     *pow(detM,2)*trace_My[index];

		double ufy = control_input_FC_y[index];	// Formation control
		double ucy = control_input_CA_y[index];		// Collision avoidance
		
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

		MatrixXd u_bar = S*eta;

		double ux = kconn*uconnx + kfc*ufx + kca*ucx;
		if ( !formation ) {
			ux = kconn*uconnx - k_bar*u_bar(2*index,0) + kca*ucx;
		}
		ux = kconn*control_input_CC_x[index];

		double uy = kconn*uconny + kfc*ufy + kca*ucy;
		if ( !formation ) {
			uy = kconn*uconny - k_bar*u_bar(2*index + 1,0) + kca*ucy;
		}

		uy = kconn*control_input_CC_y[index];
	
		double thetad = atan2(uy, ux);
		if ( !formation ) {
			thetad = atan2(uconny - u_bar(2*index+1,0) + ucy, uconnx - u_bar(2*index,0) + ucx);
		}
		thetad = atan2(uy,ux);

		double etheta = vicon[robot_name].theta() - thetad;

		double v = k*1000*cos(etheta)*sqrt(pow(ux,2) + pow(uy,2));;
		double w = -ktheta*etheta;

		// std::cerr << "v = " << v << std::endl;
		// std::cerr << "w = " << w << std::endl;
		// std::cerr << "ux = " << ux << std::endl;
		// std::cerr << "uconnx = " << uconnx << std::endl;
		// std::cerr << "ufx = " << ufx << std::endl;
		// std::cerr << "ucx = " << ucx << std::endl;
		// std::cerr << "(((pow(alphaOverBar,2)-pow(alphaUnderBar,2))*(pow(detM, 2)-pow(alphaOverBar,2)))/(pow(pow(detM, 2)-pow(alphaUnderBar,2),3)))  = " << (((pow(alphaOverBar,2)-pow(alphaUnderBar,2))
		// 		     *(pow(detM, 2)-pow(alphaOverBar,2)))
		// 		     /(pow(pow(detM, 2)-pow(alphaUnderBar,2),3)))  << std::endl;
		// std::cerr << "pow(detM,2)*trace_Mx[index] = " << pow(detM,2)*trace_Mx[index] << std::endl;
		// std::cerr << "((pow(alphaOverBar,2)-pow(alphaUnderBar,2)) *(pow(detM, 2)-pow(alphaOverBar,2))) = " << ((pow(alphaOverBar,2)-pow(alphaUnderBar,2))
		// 		     *(pow(detM, 2)-pow(alphaOverBar,2))) << std::endl;
		// std::cerr << "1 /(pow(pow(detM, 2)-pow(alphaUnderBar,2),3)) = " << 1 /(pow(pow(detM, 2)-pow(alphaUnderBar,2),3)) << std::endl;

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

		connectivity_controller::ControlMsgs dmsg; 
		dmsg.ux = ux;
		dmsg.uy = uy;
		dmsg.etheta = etheta;
		dmsg.thetad = thetad;
		dmsg.uconny = kconn*uconny;
		dmsg.ufy = kfc*ufy;
		dmsg.ucy = kca*ucy;
		dmsg.uconny = kconn*uconnx;
		dmsg.ufy = kfc*ufx;
		dmsg.ucy = kca*ucx;
		dmsg.psi = psi;


		dia.publish(dmsg);
		

		loop_rate.sleep();
	}
	
	return 0;
}