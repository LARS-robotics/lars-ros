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
#include "ros/console.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconFirmwareBinaryTest.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/core/FalconDevice.h"

// Interface Header
#include "novint_falcon_driver/NovintFalcon.h"
 

NovintFalcon::NovintFalcon()
{
	falcon = new libnifalcon::FalconDevice();
	falcon->setFalconKinematic<libnifalcon::FalconKinematicStamper>();
	display_calibration_message = true;
}

NovintFalcon::~NovintFalcon()
{
	delete falcon;
}

bool NovintFalcon::initialize(std::string firmware, bool force_firmware, bool skip_checksum, int device_index)
{
	falcon->setFalconFirmware<libnifalcon::FalconFirmwareNovintSDK>();

	if(!falcon->open(device_index))
	{
		ROS_ERROR_STREAM( "Cannot open falcon device index " << device_index << " - Lib Error Code: " << falcon->getErrorCode() << " Device Error Code: " << falcon->getFalconComm()->getDeviceErrorCode());
		return false;
	}

	falcon->setFalconFirmware<libnifalcon::FalconFirmwareNovintSDK>();

	// If force_firmware is not set, check if firmware is already loaded
	bool firmware_loaded = false;
	if(!force_firmware)
		firmware_loaded = falcon->isFirmwareLoaded();

	if(!firmware_loaded)
	{
		ROS_INFO("Loading firmware");

		uint8_t* firmware_block;
		long firmware_size;

		if(firmware.compare("nvent_firmware"))
		{
			firmware_block = const_cast<uint8_t*>(libnifalcon::NOVINT_FALCON_NVENT_FIRMWARE);
			firmware_size = libnifalcon::NOVINT_FALCON_NVENT_FIRMWARE_SIZE;


			for(int i = 0; i < 10; ++i)
			{
				if(!falcon->getFalconFirmware()->loadFirmware(skip_checksum, libnifalcon::NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(libnifalcon::NOVINT_FALCON_NVENT_FIRMWARE)))
				{
					ROS_WARN("Firmware loading try failed");
					//Completely close and reopen
					falcon->close();
					if(!falcon->open(device_index))
					{
						ROS_ERROR_STREAM( "Cannot open falcon device index " << device_index << " - Lib Error Code: " << falcon->getErrorCode() << " Device Error Code: " << falcon->getFalconComm()->getDeviceErrorCode() );
						return false;
					}
				}
				else
				{
					firmware_loaded = true;
					break;
				}
			}

		}
		else if(firmware.compare("test_firmware"))
		{
			firmware_block = const_cast<uint8_t*>(libnifalcon::NOVINT_FALCON_TEST_FIRMWARE);
			firmware_size = libnifalcon::NOVINT_FALCON_TEST_FIRMWARE_SIZE;

			for(int i = 0; i < 10; ++i)
			{
				if(!falcon->getFalconFirmware()->loadFirmware(skip_checksum, libnifalcon::NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(libnifalcon::NOVINT_FALCON_NVENT_FIRMWARE)))
				{
					ROS_WARN("Firmware loading try failed");
					//Completely close and reopen
					falcon->close();
					if(!falcon->open(device_index))
					{
						ROS_ERROR_STREAM( "Cannot open falcon device index " << device_index << " - Lib Error Code: " << falcon->getErrorCode() << " Device Error Code: " << falcon->getFalconComm()->getDeviceErrorCode() );
						return false;
					}
				}
				else
				{
					firmware_loaded = true;
					break;
				}
			}
		} 
		else 
		{
			//Check for existence of firmware file

			if(!falcon->setFirmwareFile(firmware))
			{
				ROS_ERROR_STREAM("Cannot find firmware file - " << firmware );
				return false;
			}
			for(int i = 0; i < 10; ++i)
			{
				if(!falcon->loadFirmware(skip_checksum))
				{
					ROS_WARN( "Cannot load firmware to device" );
					ROS_WARN_STREAM( "Error Code: " << falcon->getErrorCode() );
					if(falcon->getErrorCode() == 2000)
					{
						ROS_WARN_STREAM( "Device Error Code: " << falcon->getFalconComm()->getDeviceErrorCode() );
					}
					falcon->close();
					if(!falcon->open(device_index))
					{
						ROS_ERROR_STREAM( "Cannot open falcon device index " << device_index << " - Lib Error Code: " << falcon->getErrorCode() << " Device Error Code: " << falcon->getFalconComm()->getDeviceErrorCode());
						return false;
					}
				}
				else
				{
					firmware_loaded = true;
					break;
				}
			}
		}
	}
	else
	{
		return true;
	}

	if(!firmware_loaded || !falcon->isFirmwareLoaded())
	{
		ROS_ERROR("No firmware loaded to device, cannot continue");
		return false;
	}
	ROS_INFO("Firmware loaded");
	return true;
}

bool NovintFalcon::calibrate()
{
	falcon->getFalconFirmware()->setHomingMode(true);
	falcon->runIOLoop();
	if(!falcon->getFalconFirmware()->isHomed())
	{
		falcon->getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::RED_LED);
		if(display_calibration_message)
		{
			ROS_INFO( "Falcon not currently calibrated. Move control all the way out then push straight all the way in." );
			display_calibration_message = false;
		}
		return false;
	}
	ROS_INFO( "Falcon calibrated successfully." );
	falcon->getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::GREEN_LED);
	return true;
}

bool NovintFalcon::update()
{
	return falcon->runIOLoop();
}

Eigen::Vector3d NovintFalcon::getPosition()
{
	boost::array<double, 3> d = falcon->getPosition();
	return Eigen::Vector3d(d[0], d[1], d[2]);
}

void NovintFalcon::setForce(Eigen::Vector3d force)
{
	boost::array<double, 3> f;
	f[0] = force(0);
	f[1] = force(1);
	f[2] = force(2);
	falcon->setForce(f);
}
	