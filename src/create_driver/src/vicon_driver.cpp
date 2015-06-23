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
 * Filename: vicon_driver.cpp
 * 
 * Description: This file contains the implementation of the ViconStream class
 * 
 * Log
 * ----
 * 2013-09-15 File created by Hazen Eckert
 *
 */

// Interface Header
#include "create_driver/vicon_driver.h"

// ROS includes
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include "eigen_conversions/eigen_msg.h"

using namespace create_driver;

ViconStream::ViconStream()
{
}

void ViconStream::callback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	x_ = msg->transform.translation.x; 
	y_ = msg->transform.translation.y;  

	Eigen::Quaterniond eigen_q;
	tf::quaternionMsgToEigen(msg->transform.rotation, eigen_q);
	tf::Quaternion q;
	tf::quaternionEigenToTF(eigen_q, q);
	double pitch, roll;											
	tf::Matrix3x3(q).getEulerYPR(theta_, pitch, roll); // convert to radians
}

double ViconStream::x()
{
	return x_;
}

double ViconStream::y()
{
	return y_;
}

double ViconStream::theta()
{
	return theta_;
}
