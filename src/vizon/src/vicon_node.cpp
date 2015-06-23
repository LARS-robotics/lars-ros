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
 *     * Neither the name of the The University of Texas at Dallas nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "vicon/Vicon.h"
#include <stdio.h>
#include <stdlib.h>
#include <string>
//#include <LinearMath/btQuaternion.h>
#include "tf/transform_datatypes.h"
#include <math.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

std::string viconIP;

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "vicon");

	ros::NodeHandle n;
	ros::NodeHandle private_n("~");

	private_n.param<std::string>("vicon_IP", viconIP, "192.168.1.50:801");
	
	Vicon *vicon;
	vicon = new Vicon( viconIP );
	vicon->getFrame();
	int x = vicon->getSubjectCount();
	std::vector<ros::Publisher> odom_pub;
	
	
	for (int i = 0; i < x; i++) {
		std::string str = "/";
		str.append(vicon->getSubjectName(i));
		str.append("/odom");
		odom_pub.push_back(n.advertise<nav_msgs::Odometry>(str, 1));
	}
	
	
	unsigned int sequence = 0;
	
	while (ros::ok())
	{
		sequence = vicon->getFrame();
		ros::Time scan_time = ros::Time::now();
		
		for (int i = 0; i < x; i++) {
			nav_msgs::Odometry msg;
			msg.header.seq = sequence;
			msg.header.stamp = scan_time;
			msg.header.frame_id = 1;
	
			Eigen::Vector3d origin = vicon->getMarkerTranslation( vicon->getSubjectName(i), "origin" );
			Eigen::Vector3d front = vicon->getMarkerTranslation( vicon->getSubjectName(i), "front" );
			geometry_msgs::Quaternion Q;
			geometry_msgs::Point P;
			
			if ( vicon->hasMarkerWithName(vicon->getSubjectName(i), "backleft"))
			{
				// crazyflie
				ROS_INFO("cf");
				// get markers
				Eigen::Vector3d back = vicon->getMarkerTranslation( vicon->getSubjectName(i), "back" );
				Eigen::Vector3d backleft = vicon->getMarkerTranslation( vicon->getSubjectName(i), "backleft" );
				Eigen::Vector3d frontright = vicon->getMarkerTranslation( vicon->getSubjectName(i), "frontright" );
				
				P.x = origin[0];
				P.y = origin[1];
				P.z = origin[2] - 20.0;
				
				Eigen::Vector3d v1 = front - back;			// vector pointing from the back marker to the front
				Eigen::Vector3d v2 = backleft - back;		// vector pointing from the back marker to the backleft
				Eigen::Vector3d v3 = back - frontright;		// vector pointing from the front marker to the front right 
				Eigen::Vector3d u1 = v1;					
				Eigen::Vector3d e1 = u1.normalized();			// unit vector along u1
				Eigen::Vector3d u2 = v2 - (v2.dot(u1)/u1.dot(u1))*u1;	// the orthogonal component of the vector v2 to the vector u1
				Eigen::Vector3d e2 = u2.normalized();			// unit vector along u2
				Eigen::Vector3d u3 = v3 - (v3.dot(u1)/u1.dot(u1))*u1 - (v3.dot(u2)/u2.dot(u2))*u2; // the orthogonal component of the vector v3 to the vectors u1 and u2
				Eigen::Vector3d e3 = u3.normalized();
				
			    double zeta = M_PI/4.0;
				Eigen::Matrix3d R0;
				if (vicon->getSubjectName(i).compare("apollo") == 0)
				{
					R0 << 	0.731890,	-0.423853,	-0.533560, 
							0.681012, 	0.427771, 	0.594335,
							-0.023669, 	-0.798348, 	0.601730;
				} else if (vicon->getSubjectName(i).compare("sputnik") == 0) {
					R0 << 	0.61107991, 0.275000, 0.744796, 
							0.623794, 0.414865, -0.662395,
							0.491149, -0.867330, -0.080691;
				}
				Eigen::Matrix3d R1;
				R1 << e1, e2, e3; 
				Eigen::Matrix3d R  = R1*R0.transpose();
				//ROS_INFO("R:\n");
				//ROS_INFO("%f, %f, %f\n",  R(0,0), R(0,1), R(0,2) );
				//ROS_INFO("%f, %f, %f\n",  R(1,0), R(1,1), R(1,2) );
				//ROS_INFO("%f, %f, %f\n",  R(2,0), R(2,1), R(2,2) );
				
				//ROS_INFO("e1: %f, %f, %f\n",  e1(0), e1(1), e1(2) );
				//ROS_INFO("e2: %f, %f, %f\n",  e2(0), e2(1), e2(2) );
				//ROS_INFO("e3: %f, %f, %f\n",  e3(0), e3(1), e3(2) );
		
				Eigen::Quaterniond q(R);
				
				Q.x = q.x();
				Q.y = q.y();
				Q.z = q.z();
				Q.w = q.w();
				
			} else {
				// irobot create
				ROS_INFO("irc");
				if (vicon->getSubjectName(i).compare("caesar") == 0)
				{
					origin = (origin + front)/2.0;
				}
				Eigen::Vector3d vector = front - origin;;
			
				float YPR[3] = {atan2( vector[1], vector[0] ), 0.0, 0.0};
			
				tf::Quaternion btQ;// = tf::Quaternion(YPR[0], YPR[1], YPR[2]);
				btQ.setEulerZYX(YPR[0], YPR[1], YPR[2]);
				P.x = origin[0];
				P.y = origin[1];
				P.z = origin[2] - 20.0;
				
				Q.x = (double) btQ.x();
				Q.y = (double) btQ.y();
				Q.z = (double) btQ.z();
				Q.w = (double) btQ.w();
			}
			 
			msg.pose.pose.position = P;
			msg.pose.pose.orientation = Q;
			odom_pub[i].publish(msg);
		}
	
		ros::spinOnce();

	}
  return 0;
}
