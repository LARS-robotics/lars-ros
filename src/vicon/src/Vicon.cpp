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
 * Filename: Vicon.cpp
 * 
 * Description: ROS Wrapper for the Vicon Motion Capture System
 * 
 * Log
 * ----
 * 2013-09-14 File created by Hazen Eckert
 *
 */
 
// Interface Header
#include "vicon/Vicon.h"

using namespace ViconDataStreamSDK::CPP;
using namespace std;

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

Vicon::Vicon(string vicon_IP):vicon_client_(),server_name_(vicon_IP),
								frame_number_(0),robot_names_(), robot_data_()
{
}

bool Vicon::connect()
{
    Output_Connect Output = vicon_client_.Connect(server_name_);
    if (Output.Result == Result::Success || Output.Result == Result::ClientAlreadyConnected)
    {
    	// Enable Data
		vicon_client_.EnableSegmentData();
		vicon_client_.EnableMarkerData();
		vicon_client_.EnableUnlabeledMarkerData();
		vicon_client_.EnableDeviceData();

		// Set Stream Mode
		vicon_client_.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);

		// Set the global up axis
		vicon_client_.SetAxisMapping(Direction::Forward, 
										Direction::Left, 
					 					Direction::Up); // Z-up
		
        return true;
    }
    return false;
}

bool Vicon::isConnected()
{
	Output_IsConnected Output = vicon_client_.IsConnected();
	if (Output.Connected == true)
	{
		return true;
	}
	return false;
}

bool Vicon::update()
{
	 if (vicon_client_.GetFrame().Result == Result::Success)
	 {
	 	// Get frame number
	 	frame_number_ = vicon_client_.GetFrameNumber().FrameNumber;
	 	
	 	// Temp name vector and robot data map
	 	vector<string> temp_robot_names;
	 	
	 	map<string, map<string, Eigen::Vector3d> > temp_robot_data;
	 	
	 	// Fill out temp_robot_names_ and temp_robot_data_
	 	unsigned int num_robots = vicon_client_.GetSubjectCount().SubjectCount;
	 	for (int i = 0; i < num_robots; i++)
	 	{
	 		// Add robot name to temp name vector
	 		temp_robot_names.push_back(vicon_client_.GetSubjectName(i).SubjectName);
	 		
	 		// Get the markers for this robot
	 		unsigned int num_markers = vicon_client_.GetMarkerCount(temp_robot_names.back()).MarkerCount;
	 		map<string, Eigen::Vector3d> temp_robot_markers;
	 		for (int j = 0; j < num_markers; j++)
	 		{
	 			// Get marker name
	 			string marker_name = vicon_client_.GetMarkerName(temp_robot_names.back(), j).MarkerName;
	 			
	 			// Get marker translation and convert to meters Eigen Vector 
	 			double *translation = vicon_client_.GetMarkerGlobalTranslation(temp_robot_names.back(), marker_name).Translation;	
	 			Eigen::Vector3d position(translation[0]/1000.0, translation[1]/1000.0, translation[2]/1000.0);
	 			
	 			// Add Eigen::Vector3d to map
	 			temp_robot_markers.insert(pair<string, Eigen::Vector3d>(marker_name, position));
	 		}
	 		
	 		// Add marker map to robot map
	 		temp_robot_data.insert(pair<string, map<string, Eigen::Vector3d> >(temp_robot_names.back(), temp_robot_markers));
	 	}
	 	
	 	// Copy map and vector
	 	robot_names_ = temp_robot_names;
	 	robot_data_ = temp_robot_data;
	 	
	 	return true;
	 }
	 
	 return false;
}

unsigned int Vicon::getFrameNumber()
{
	return frame_number_;
}

map<string, map<string, Eigen::Vector3d> > Vicon::getRobotData()
{
	return robot_data_;
}

vector<string> Vicon::getRobotNames()
{
	return robot_names_;
}

void Vicon::disconnect()
{
	vicon_client_.Disconnect();
}
