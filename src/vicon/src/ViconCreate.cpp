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
 * Filename: ViconCreate.cpp
 * 
 * Description: This file contains the implementation of the ViconCreate class
 * 
 * Log
 * ----
 * 2013-09-14 File created by Hazen Eckert
 *
 */
 
// Interface Header
#include "vicon/ViconCreate.h"

// ROS includes
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"

/*
 * Class: ViconCreate
 * 
 * Description: Class for converting marker maps of Create robots to translation 
 * and orientation
 * 
 * Attributes
 * ----------
 * translation - postion of the robot in the global frame
 * orientation - orientation of the robot in the global frame
 * 
 */
 
ViconCreate::ViconCreate(std::map<std::string, Eigen::Vector3d> markers):ViconRobot(markers)
{
	translation = markers["origin"];
	Eigen::Vector3d vector = markers["front"] - markers["origin"];
	double YPR[3] = {atan2(vector[1], vector[0]), 0.0, 0.0};
	tf::Quaternion tf_q = tf::createQuaternionFromRPY(YPR[2], YPR[1], YPR[0]);
	Eigen::Quaterniond q;
	quaternionTFToEigen(tf_q, q);
	orientation = q;
}

Eigen::Vector3d ViconCreate::getTranslation()
{
	return translation;
}

Eigen::Quaterniond ViconCreate::getOrientation()
{
	return orientation;
}
