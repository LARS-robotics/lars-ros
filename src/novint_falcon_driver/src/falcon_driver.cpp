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
 * Filename: falcon_driver.cpp
 * 
 * Description: This file contains the ROS node that interfaces with the 
 * Novint Falcon 3D Controller
 * 
 * Log
 * ----
 * 2015-02-09 File created by Hazen Eckert
 *
 */
 
// ROS includes
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"

#include "novint_falcon_driver/NovintFalcon.h"

#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

/*
Process:
- get configuration parameters
- setup flacon class
- 
*/
using namespace Eigen;
Vector3d force;

void force_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	force(0) = msg->x;
	force(1) = msg->y;
	force(2) = msg->z;
}


 int main(int argc, char **argv)
{
	// ROS Initalization
	ros::init(argc, argv, "falcon_driver");
	ros::NodeHandle n;
	ros::NodeHandle private_n("~");

	// ROS Parameters
	std::string firmware;
	bool force_firmware, skip_checksum;
	int device_index;

	private_n.param<std::string>("firmware", firmware, "nvent_firmware");

	private_n.param<bool>("force_firmware", force_firmware, false);

	private_n.param<bool>("skip_checksum", skip_checksum, true);

	private_n.param<int>("device_index", device_index, 0);

	// ROS Subscribers

	ros::Subscriber force_sub = n.subscribe("force", 10, force_callback);

	// ROS Publishers

	ros::Publisher position_pub = n.advertise<geometry_msgs::Vector3>("position", 10);

	// Falcon Initialization
	NovintFalcon falcon;
	if(!falcon.initialize( firmware, force_firmware, skip_checksum, device_index ))
	{
		ROS_FATAL("Unable to initialize the falcon device. Exiting...");
		return 1;
	}

	while(!falcon.calibrate() && ros::ok());

	// ROS loop

	while(ros::ok())
	{
		if (!falcon.update())
			continue;

		ros::spinOnce();

		Vector3d position = falcon.getPosition();

		geometry_msgs::Vector3 msg;
		msg.x = position(0);
		msg.y = position(1);
		msg.z = position(2);

		position_pub.publish(msg);

		if(force_sub.getNumPublishers() != 1)
			falcon.setForce(Vector3d(0,0,0));
		else
			falcon.setForce(force);

	}

}