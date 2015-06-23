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
 * Filename: ViconCrazyflie.h
 * 
 * Description: This file contains the class for converting marker maps of Crazyflies
 * to translation and orientation
 * 
 * Log
 * ----
 * 2013-09-14 File created by Hazen Eckert
 *
 */
 
#ifndef __ViconCrazyflie_H__
#define __ViconCrazyflie_H__

// Library includes
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Local Package includes
#include "vicon/ViconRobot.h"

/*
 * Class: ViconCrazyflie
 * 
 * Description: Class for converting marker maps of crazyflie robots to translation 
 * and orientation
 * 
 * Attributes
 * ----------
 * translation - postion of the robot in the global frame
 * orientation - orientation of the robot in the global frame
 * 
 */

class ViconCrazyflie: public ViconRobot
{
public:

	/*
	 * Method: ViconCrazyflie(map<string, Eigen::Vector3d> markers)
	 * 
	 * Description: Constructor, converts the map to translation and orientation
	 * 
	 * Arguments: map<string, Eigen::Vector3d>: markers map
	 *
	 * Return Value: ViconCrazyflie instance
	 * 
	 */
	ViconCrazyflie(std::map<std::string, Eigen::Vector3d> markers);
	
	/*
	 * Method: getTranslation()
	 * 
	 * Description: Get the translation of the robot
	 *
	 * Return Value: Eigen::Vector3d: translation of the robot
	 * 
	 */
	Eigen::Vector3d getTranslation();
	
	/*
	 * Method: getOrientation()
	 * 
	 * Description: Get the orientation of the robot
	 *
	 * Return Value: Eigen::Quaterniond: orientation of the robot
	 * 
	 */
	Eigen::Quaterniond getOrientation();
};

#endif
