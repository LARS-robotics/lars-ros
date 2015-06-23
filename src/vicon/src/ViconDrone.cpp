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
 * Filename: ViconDrone.cpp
 * 
 * Description: This file contains the implementation of the ViconDrone class
 * 
 * Log
 * ----
 * 2013-09-14 File created by Hazen Eckert
 *
 */
 
// Interface Header
#include "vicon/ViconDrone.h"
 
/*
 * Class: ViconDrone
 * 
 * Description: Class for converting marker maps of Drone robots to translation 
 * and orientation
 * 
 * Attributes
 * ----------
 * translation - postion of the robot in the global frame
 * orientation - orientation of the robot in the global frame
 * 
 */
 
// TO DO: The orientation is not correct
ViconDrone::ViconDrone(std::map<std::string, Eigen::Vector3d> markers):ViconRobot(markers)
{
	Eigen::Vector3d v1 = markers["drone_front"] - markers["drone_back"];
	Eigen::Vector3d v2 = markers["drone_left"] - markers["drone_right"];
	Eigen::Vector3d v3 = markers["drone_front"] - markers["drone_back"];
	
	Eigen::Vector3d u1 = v1;
	Eigen::Vector3d e1 = u1.normalized(); // unit vector along u1
	
	Eigen::Vector3d u2 = v2 - (v2.dot(u1)/u1.dot(u1))*u1; // the orthogonal component of the vector v2 to the vector u1
	Eigen::Vector3d e2 = u2.normalized(); // unit vector along u2
	
	Eigen::Vector3d u3 = v3 - (v3.dot(u1)/u1.dot(u1))*u1 - (v3.dot(u2)/u2.dot(u2))*u2; // the orthogonal component of the vector v3 to the vectors u1 and u2
	Eigen::Vector3d e3 = u3.normalized(); // unit vector along u3
	
	double zeta = M_PI/4.0;
	
	Eigen::Matrix3d R0;
	R0 <<   0.731890,       -0.423853,      -0.533560, 
			0.681012,       0.427771,       0.594335,
			-0.023669,      -0.798348,      0.601730;
			
	Eigen::Matrix3d R1;
	R1 << e1, e2, e3; 
	Eigen::Matrix3d R  = R1*R0.transpose();
	
	Eigen::Quaterniond q(R);
	
	translation = markers["drone_front"];
	orientation = q;
}

Eigen::Vector3d ViconDrone::getTranslation()
{
	return translation;
}

Eigen::Quaterniond ViconDrone::getOrientation()
{
	return orientation;
}
