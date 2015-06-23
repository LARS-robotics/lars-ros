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
 * Filename: vicon_node.cpp
 *
 * Description: This file contains the ROS node that publishes robot world transforms
 * from a Vicon Motion Capture system. These transforms are found by reading the Vicon
 * subjects' named markers (origin and front for iRobot Creates) (origin, front, back,
 * left, and right for Bitcraze Crazyflie) and calculating its position and orientation.
 * These transforms are published as a geometry_msgs/TransformStamped.msg (Header,
 * Vector3 and Quaternion) to the topic /<Subject Name>/tf
 *
 * Log
 * ----
 * 2013-09-14 File created by Hazen Eckert
 *
 */

// ROS includes
#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include "eigen_conversions/eigen_msg.h"

// Library includes
#include <string>
#include <map>
#include <vector>
#include <iterator>
#include <Eigen/Core>

// Local Package includes
#include "vicon/Vicon.h"
#include "vicon/ViconRobot.h"
#include "vicon/ViconCrazyflie.h"
#include "vicon/ViconCreate.h"


int main (int argc, char **argv)
{
	// ROS Initalization
	ros::init(argc, argv, "vicon");
	ros::NodeHandle n;
	ros::NodeHandle private_n("~");

	// ROS Parameters
	std::string vicon_IP;
	
	private_n.param<std::string>("vicon_IP", vicon_IP, "192.168.1.50:801");

	// Vicon Initalization
	Vicon *vicon;
	vicon = new Vicon(vicon_IP);

	// Vicon Connect
	ros::Duration vicon_sleep(0, 100000);
	while (vicon->connect() == false && ros::ok())
	{
		vicon_sleep.sleep();
	}	
	
	// ROS Publisher Map
	std::map<std::string, ros::Publisher> robot_pub;

	// ROS Loop
	while (ros::ok() && vicon->isConnected())
	{
		// Pull New Vicon Data
		while (vicon->update() == false && ros::ok())
		{
			vicon_sleep.sleep();
		}

		// Message Header
		std_msgs::Header header;
		header.seq = vicon->getFrameNumber();
		header.stamp = ros::Time::now();
		header.frame_id = 1;
		
		// Map of robots and their markers
		std::map<std::string, std::map<std::string, Eigen::Vector3d> > robot_data = vicon->getRobotData();

		// Loop over all robots
		std::vector<std::string> robot_names = vicon->getRobotNames();
		std::vector<std::string>::iterator it;

		for (it = robot_names.begin(); it != robot_names.end(); it++)
		{
			// Message Data
			geometry_msgs::TransformStamped msg;
			msg.header = header;
			
			// Calculate robot transforms
			ViconRobot *robot;
			
			// If the robot has a "left" marker it is a Crazyflie
			if (robot_data[*it].find("crazy_left") != robot_data[*it].end())
			{
				robot = new ViconCrazyflie(robot_data[*it]);
			} else { // If the robot is a Create
				robot = new ViconCreate(robot_data[*it]);
			}

			// Get Rotation and translation
			geometry_msgs::Vector3 t;
			tf::vectorEigenToMsg(robot->getTranslation(), t);
			msg.transform.translation = t;
			tf::quaternionEigenToMsg(robot->getOrientation(), msg.transform.rotation);

			// If we do not yet have a publisher for this robot, create one
			if (robot_pub.find(*it) == robot_pub.end())
			{
				std::string topic_name = "/";
				topic_name.append(*it);
				topic_name.append("/tf");
				robot_pub.insert(std::pair<std::string, ros::Publisher>(*it, n.advertise<geometry_msgs::TransformStamped>(topic_name, 1)));
			}
			
			// Publish transform
			robot_pub[*it].publish(msg);
			delete robot;
		}

		ros::spinOnce();
	}
	
	// If vicon is already disconnected there has been an error
	if (vicon->isConnected())
	{
		vicon->disconnect();
	} else {
		ROS_ERROR("Vicon has become disconnected");
	}
	
	return 0;
}
