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
 * Filename: NovintFalcon.h
 * 
 * Description: This file contains the class for interfacing with the Novint
 * Falcon 3D Controller
 *
 * Log
 * ----
 * 2015-02-09 File created by Hazen Eckert
 *
 */

 /* TODO
- parse configuration options
- calibrate
- run loop in thread over force and position
 */
 
#ifndef __NovintFalcon_H__
#define __NovintFalcon_H__


// Library includes
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "falcon/core/FalconDevice.h"

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

class NovintFalcon
{
public:

	/*
	 * Method: ViconCreate(map<string, Eigen::Vector3d> markers)
	 * 
	 * Description: Constructor, converts the map to translation and orientation
	 * 
	 * Arguments: map<string, Eigen::Vector3d>: markers map
	 *
	 * Return Value: ViconCreate instance
	 * 
	 */
	NovintFalcon();
	~NovintFalcon();
	bool initialize(std::string firmware, bool force_firmware, bool skip_checksum, int device_index);
	bool calibrate();
	bool update();
	Eigen::Vector3d getPosition();
	void setForce(Eigen::Vector3d force);
	
	/*
	 * Method: getTranslation()
	 * 
	 * Description: Get the translation of the robot
	 *
	 * Return Value: Eigen::Vector3d: translation of the robot
	 * 
	 */

	
	/*
	 * Method: getOrientation()
	 * 
	 * Description: Get the orientation of the robot
	 * 
	 * Return Value: Eigen::Quaterniond: orientation of the robot
	 * 
	 */
protected:
	libnifalcon::FalconDevice *falcon;
	bool display_calibration_message;
};


#endif
