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
 * Filename: connectivity.cpp
 * 
 * Description: This file contains the ROS node that calculates the control variables
 * for a collection of iRobot Creates.
 * 
 * Log
 * ----
 * 2013-09-15 File created by Hazen Eckert
 *
 */
 
// ROS includes
#include "ros/ros.h"
#include "ros/assert.h"
#include "dynamic_reconfigure/server.h"
#include "create_driver/vicon_driver.h"

// Library includes
#include <string>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/LU>

// Local Package includes
#include "connectivity_controller/ConnectivityMsgs.h"
#include "connectivity_controller/ConnectivityVariablesConfig.h"

using namespace std;
using namespace Eigen;

#define rho1 0.7
#define rho2 2.3

// Collision avoidance variables
double R = 0.7;	  
double r = 0.33;

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

void reconfigureCallback(connectivity_controller::ConnectivityVariablesConfig &config, uint32_t level) {
	R = config.r_max;
	r = config.r_min;

}

int main(int argc, char **argv)
{
	// ROS Initalization
	ros::init(argc, argv, "connectivity");
	
	ros::NodeHandle n;
	ros::NodeHandle private_n("~");

	dynamic_reconfigure::Server<connectivity_controller::ConnectivityVariablesConfig> server;
  	dynamic_reconfigure::Server<connectivity_controller::ConnectivityVariablesConfig>::CallbackType f;
  	f = boost::bind(&reconfigureCallback, _1, _2);
 	server.setCallback(f);

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
	
	// List of formation x positions
	vector<double> formation_x;
	XmlRpc::XmlRpcValue x_list;
	private_n.getParam("formation_x_list", x_list);
	ROS_ASSERT(x_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for (int i = 0; i < x_list.size(); i++) 
	{
		ROS_ASSERT(x_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		formation_x.push_back(static_cast<double>(x_list[i]));
	}
	
	// List of formation y positions
	vector<double> formation_y;
	XmlRpc::XmlRpcValue y_list;
	private_n.getParam("formation_y_list", y_list);
	ROS_ASSERT(y_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for (int i = 0; i < y_list.size(); i++) 
	{
		ROS_ASSERT(y_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		formation_y.push_back(static_cast<double>(y_list[i]));
	}
	
	// Number of robots
	const int num_robots = robot_names.size();
	
	// ROS Subscribers
	map<string, create_driver::ViconStream> vicon;
	vector<string>::iterator name_it;
	vector<ros::Subscriber> sub;

	for (name_it = robot_names.begin(); name_it != robot_names.end(); name_it++)
	{
		vicon.insert(pair<string, create_driver::ViconStream>(*name_it, create_driver::ViconStream()));
		sub.push_back(n.subscribe(*name_it + "/tf", 10, &create_driver::ViconStream::callback, &vicon[*name_it]));
	}
	
	// ROS Publishers
	ros::Publisher pub = n.advertise<connectivity_controller::ConnectivityMsgs>("/connectivity", 10);
	
	// ROS loop
	ros::Rate loop_rate(250); // 250 Hz
	
	while (ros::ok())
	{
		// Retrieve Vicon Data
		ros::spinOnce();
		
		// Calculations
		MatrixXd adjacency_matrix = MatrixXd::Zero(num_robots, num_robots);
		MatrixXd P = MatrixXd::Zero(num_robots, num_robots - 1);
		MatrixXd L = MatrixXd::Zero(num_robots, num_robots);
		
		// Adjacency Matrix	
		for (int i = 0; i < num_robots; i++) 
		{
			for (int j = 0; j < num_robots; j++)
			{
				adjacency_matrix(i,j) = psi(sqrt(
				pow(vicon[robot_names[j]].x() - vicon[robot_names[i]].x(), 2) + 
				pow(vicon[robot_names[j]].y() - vicon[robot_names[i]].y(), 2)
				));
			}
		}
		
		
		// L Matrix
		for (int i = 0; i < num_robots; i++) 
		{
			for (int j = 0; j < num_robots; j++)
			{
				if ( i == j ) {
					double rowSum = 0;
					for (int k = 0; k < num_robots; k++) {
						rowSum += adjacency_matrix(i,k);
					}
					L(i,j) = rowSum - adjacency_matrix(i,j);
				} else {
					L(i,j) = -adjacency_matrix(i,j);
				}
			}
		}
		
		// P Matrix
		if (num_robots == 6) 
		{
			P << 	1/sqrt(2), 		-1/sqrt(6.0),		-1/2.0/sqrt(3), 	-1/2.0/sqrt(5), 	-1/sqrt(30),
					0.0,       		sqrt(2.0/3), 		-1/2.0/sqrt(3), 	-1/2.0/sqrt(5), 	-1/sqrt(30),
					0.0, 		 	0.0, 				sqrt(3.0)/2,    	-1/(2*sqrt(5)),		-1/sqrt(30),
					0.0, 			0.0, 				0.0, 				2/sqrt(5), 			-1/sqrt(30),
					0.0, 			0.0, 				0.0,				0.0, 				sqrt(5.0/6),
					-1/sqrt(2), 	-1/sqrt(6),			-1/2.0/sqrt(3), 	-1/2.0/sqrt(5), 	-1/sqrt(30);
		} else if (num_robots == 4) {
			P << 	-0.5,        		-0.5,              	-0.5, 
					 0.833333333333333, -0.166666666666667, -0.166666666666667, 
					-0.166666666666667,  0.833333333333333, -0.166666666666667, 
					-0.166666666666667, -0.166666666666667,  0.833333333333333;
		}
		// M Matrix
		MatrixXd M = P.transpose() * L * P;
		//std::cerr << "adjacency_matrix" << std::endl << adjacency_matrix << std::endl << std::endl;
		// dLdx and dLdy for each robot
		MatrixXd dLdx[num_robots];
		MatrixXd dLdy[num_robots];
		
		for (int i = 0; i < num_robots; i++)
		{
			dLdx[i] = MatrixXd::Zero(num_robots, num_robots);
			dLdy[i] = MatrixXd::Zero(num_robots, num_robots);
		}
		
		for(int k = 0; k < num_robots; k++)
		{
			for(int i = 0; i < num_robots; i++)
			{
				for (int j = i + 1; j < num_robots; j++) 
				{
					double D = sqrt(
					pow(vicon[robot_names[i]].x() - vicon[robot_names[j]].x(), 2) + 
					pow(vicon[robot_names[i]].y() - vicon[robot_names[j]].y(), 2)
					);
					
					double dAdD = psiPrime(D);
			
					if( k == i ) {
						double dDdx = (vicon[robot_names[i]].x()-vicon[robot_names[j]].x()) / D;							
						double dDdy = (vicon[robot_names[i]].y()-vicon[robot_names[j]].y()) / D;
						
						dLdx[k](i,j) = dAdD*dDdx;
						dLdy[k](i,j) = dAdD*dDdy;        		
						
					}
			
					if( k == j ) {
						double dDdx = -(vicon[robot_names[i]].x()-vicon[robot_names[j]].x()) /D;							
						double dDdy = -(vicon[robot_names[i]].y()-vicon[robot_names[j]].y()) / D;
						
						dLdx[k](i,j) = dAdD*dDdx;
						dLdy[k](i,j) = dAdD*dDdy;
				      		
					}
			
			   	
					dLdx[k](j,i) = dLdx[k](i,j);
					dLdy[k](j,i) = dLdy[k](i,j);
				
				}
			}

			for(int i = 0; i < num_robots; i++)
			{
				double sumx = 0;
				double sumy = 0;
				for(int j = 0; j<num_robots; j++)
				{
					sumx += dLdx[k](i,j);
					sumy += dLdy[k](i,j);
				} 
				dLdx[k](i,i) = -sumx;
	            dLdy[k](i,i) = -sumy;
			}
		}
		
		// dMdx and dMdy for each robot
		MatrixXd dMdx[num_robots];
		MatrixXd dMdy[num_robots];
		
		for (int i = 0; i < num_robots; i++)
		{
			dMdx[i] = P.transpose() * dLdx[i] * P;
			dMdy[i] = P.transpose() * dLdy[i] * P;
			//std:cerr << dMdx[i] << std::endl << std::endl;
		}
		
		// Collision Avoidance for each robot starts here:
		MatrixXd CAMatrix_x = MatrixXd::Zero(num_robots,num_robots);		
		MatrixXd CAMatrix_y = MatrixXd::Zero(num_robots,num_robots);
		
		for(int i = 0; i < num_robots; i++)
		{
			for (int j = i + 1; j < num_robots; j++) 
			{
				double D = sqrt( 
				pow(vicon[robot_names[i]].x() - vicon[robot_names[j]].x(), 2) + 
				pow(vicon[robot_names[i]].y() - vicon[robot_names[j]].y(), 2)
				);
				if( D < R && D > r){
					CAMatrix_x(i,j) = -4*(pow(R,2)-pow(r,2))
										*(pow(D,2)-pow(R,2))
										/pow((pow(D,2)-pow(r,2)),3)
										*(vicon[robot_names[i]].x() 
										- vicon[robot_names[j]].x());
										
		        	CAMatrix_y(i,j) = -4*(pow(R,2)-pow(r,2))
		        						*(pow(D,2)-pow(R,2))
		        						/pow((pow(D,2)-pow(r,2)),3)
		        						*(vicon[robot_names[i]].y() 
		        						- vicon[robot_names[j]].y());
            	} else {
            		CAMatrix_x(i,j) = 0;
            		CAMatrix_y(i,j) = 0;
            	}
            	
            	CAMatrix_x(j,i) = -CAMatrix_x(i,j);
	       		CAMatrix_y(j,i) = -CAMatrix_y(i,j);
			}
		}
		
		double control_input_CA_x[num_robots];				 		 
		double control_input_CA_y[num_robots];				 		 
		
		for(int i = 0; i < num_robots; i++)
		{
			double sumx = 0;
			double sumy = 0;
			for (int j = 0; j < num_robots; j++) 
			{
				sumx = sumx + CAMatrix_x(i,j);
				sumy = sumy + CAMatrix_y(i,j);
			}
			control_input_CA_x[i] = sumx;
			control_input_CA_y[i] = sumy;
		}
			
		double control_input_FC_x[num_robots];				 		 
		double control_input_FC_y[num_robots];

		for (int i = 0; i < num_robots; i++)
		{
			control_input_FC_x[i] = formation_x[i] - vicon[robot_names[i]].x();
			control_input_FC_y[i] = formation_y[i] - vicon[robot_names[i]].y();
		}

		// Trace of Mx and My for each robot
		double trace_Mx[num_robots];
		double trace_My[num_robots];
		
		for (int i = 0; i < num_robots; i++)
		{
			//std::cerr << (M.inverse()*dMdx[i]) << std::endl << std::endl;
			trace_Mx[i] = (M.inverse()*dMdx[i]).trace();
			trace_My[i] = (M.inverse()*dMdy[i]).trace();
		}
		
		double determinant_M = M.determinant();

		// Send msg
		
		connectivity_controller::ConnectivityMsgs msg; 
		
		msg.trMx.assign(trace_Mx, trace_Mx + num_robots);
		msg.trMy.assign(trace_My, trace_My + num_robots);
		msg.u_ca_x.assign(control_input_CA_x, control_input_CA_x + num_robots);
		msg.u_ca_y.assign(control_input_CA_y, control_input_CA_y + num_robots);
		msg.u_fc_x.assign(control_input_FC_x, control_input_FC_x + num_robots);
		msg.u_fc_y.assign(control_input_FC_y, control_input_FC_y + num_robots);
		msg.detM = determinant_M;

		pub.publish(msg);
		
		// Sleep
		loop_rate.sleep();
	}
	
	return 0;
}
