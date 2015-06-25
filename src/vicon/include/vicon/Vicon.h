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
 * Filename: Vicon.h
 *
 * Description: ROS Wrapper for the Vicon Motion Capture System
 *
 * Log
 * ----
 * 2013-09-14 File created by Hazen Eckert
 *
 */

#ifndef _Vicon_H
#define _Vicon_H

// Library includes
#include <string>
#include <map>
#include <vector>
#include <Eigen/Core>

// Local Package includes
#include "vicon/Client.h"

//using namespace std;

/*
 * Class: Vicon
 *
 * Description: This class is a ROS wrapper for the Vicon Motion Capture system.
 *
 * Attributes
 * ----------
 * vicon_client_ - Client object of the Vicon SDK
 * server_name_ - address:port of the Vicon server
 * frame_number_ - current frame number
 * robot_names_ - vector of the robot names for the current frame
 * robot_data_ - map of the robot names to a map of marker names to marker positions
 *				 for the current frame
 *
 */

class Vicon
{
public:

	/*
	 * Method: Vicon(string vicon_IP)
	 *
	 * Description: Vicon constructor
	 *
	 * Arguments: string: "<ip address>:<port number>"
	 *
	 */
	Vicon(std::string vicon_IP);

	/*
	 * Method: connect()
	 *
	 * Description: Attempts to connect to the Vicon server
	 *
	 * Return Value: bool: true if connection is successful else false
	 *
	 */
	bool connect();

	/*
	 * Method: isConnected()
	 *
	 * Description: Checks if we are still connected to the server
	 *
	 * Return Value: bool: true if connected else false
	 *
	 */
	bool isConnected();

	/*
	 * Method: update()
	 *
	 * Description: Pulls new data from the Vicon server
	 *
	 * Return Value: bool: true if successful else false
	 *
	 */
	bool update();

	/*
	 * Method: getFrameNumber()
	 *
	 * Description: Gets the frame number for the current dataset
	 *
	 * Return Value: unsigned int: current frame number
	 *
	 */
	unsigned int getFrameNumber();

	/*
	 * Method: getRobotData()
	 *
	 * Description: Gets the marker data for all the robots
	 *
	 * Return Value: map<string, map<string, Eigen::Vector3d>>: map of the robot
	 * names to a map of marker names to marker positions for the current frame
	 *
	 */
	std::map<std::string, std::map<std::string, Eigen::Vector3d> > getRobotData();

	/*
	 * Method: getRobotNames()
	 *
	 * Description: Gets the names of all the robots in the current dataset
	 *
	 * Return Value: vector<string>: vector of robot names
	 *
	 */
	std::vector<std::string> getRobotNames();

	/*
	 * Method: disconnect()
	 *
	 * Description: Disconnects from the vicon server
	 *
	 */
	void disconnect();

private:
	ViconDataStreamSDK::CPP::Client vicon_client_;
	std::string server_name_;
	unsigned int frame_number_;
	std::vector<std::string> robot_names_;
	std::map<std::string, std::map<std::string, Eigen::Vector3d> > robot_data_;
};

#endif
