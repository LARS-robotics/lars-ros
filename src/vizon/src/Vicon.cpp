/*
 * Vicon.cpp
 * iRobot Server
 * 
 * Created by Hazen Eckert on 10/29/11.
 * The Laboratory for Autonomous Robotics and Systems. 
 * The University of Texas at Dallas.
 *
 */

#include "vicon/Vicon.h"
#include "vicon/Client.h"
#include "ros/ros.h"
#include <stdio.h>

using namespace ViconDataStreamSDK::CPP;

namespace
{
  std::string Adapt( const bool i_Value )
  {
    return i_Value ? "True" : "False";
  }

  std::string Adapt( const Direction::Enum i_Direction )
  {
    switch( i_Direction )
    {
      case Direction::Forward:
        return "Forward";
      case Direction::Backward:
        return "Backward";
      case Direction::Left:
        return "Left";
      case Direction::Right:
        return "Right";
      case Direction::Up:
        return "Up";
      case Direction::Down:
        return "Down";
      default:
        return "Unknown";
    }
  }

  std::string Adapt( const DeviceType::Enum i_DeviceType )
  {
    switch( i_DeviceType )
    {
      case DeviceType::ForcePlate:
        return "ForcePlate";
      case DeviceType::Unknown:
      default:
        return "Unknown";
    }
  }

  std::string Adapt( const Unit::Enum i_Unit )
  {
    switch( i_Unit )
    {
      case Unit::Meter:
        return "Meter";
      case Unit::Volt:
        return "Volt";
      case Unit::NewtonMeter:
        return "NewtonMeter";
      case Unit::Newton:
        return "Newton";
      case Unit::Unknown:
      default:
        return "Unknown";
    }
  }
}
 
Vicon::Vicon( std::string viconIP ) // Vicon constructor
{
	std::string HostName = viconIP;

	while( !MyClient.IsConnected().Connected && ros::ok() ) // poll until connected
	{
		MyClient.Connect( HostName );
		usleep(100);
	}
	
	if ( ros::ok() )
	{
	 
		// enable data
		MyClient.EnableSegmentData();
		MyClient.EnableMarkerData();
		MyClient.EnableUnlabeledMarkerData();
		MyClient.EnableDeviceData();

		MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );
		//MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch );

		// Set the global up axis
		MyClient.SetAxisMapping( Direction::Forward, 
					 				Direction::Left, 
		                          	 Direction::Up ); // Z-up
	}
}

Vicon::~Vicon() // Vicon deconstructor
{
	MyClient.Disconnect();
}

unsigned int Vicon::getFrame()
{
	while( MyClient.GetFrame().Result != Result::Success && ros::ok() )
      		{
     			// Sleep a little so that we don't lumber the CPU with a busy poll
          		usleep(50);
      		}
    Output_GetFrameNumber Output = MyClient.GetFrameNumber();
    return Output.FrameNumber;
  	
}

unsigned int Vicon::getSubjectCount() 
{
	Output_GetSubjectCount Output;
	Output = MyClient.GetSubjectCount();
	printf("%d",Output.SubjectCount);
	return Output.SubjectCount;
}

std::string Vicon::getSubjectName( unsigned int SubjectIndex )
{
	std::string SubjectName = MyClient.GetSubjectName( SubjectIndex ).SubjectName;
	return SubjectName;
}

std::string Vicon::getSubjectRootSegmentName( std::string SubjectName )
{
	std::string RootSegment = MyClient.GetSubjectRootSegmentName( SubjectName ).SegmentName;
	return RootSegment;
}

unsigned int Vicon::getSegmentCount( std::string SubjectName ) 
{
	Output_GetSegmentCount Output;
	Output = MyClient.GetSegmentCount( SubjectName );
	return Output.SegmentCount;
}

std::string Vicon::getSegmentName( std::string SubjectName, unsigned int SegmentIndex )
{
	Output_GetSegmentName OutputGSN;
	OutputGSN = MyClient.GetSegmentName( SubjectName, SegmentIndex );
	return OutputGSN.SegmentName;
}

Eigen::Vector3d Vicon::getSegmentGlobalTranslation( std::string SubjectName, std::string SegmentName )
{
	Output_GetSegmentGlobalTranslation GlobalTranslation = 
				MyClient.GetSegmentGlobalTranslation( SubjectName, SegmentName );
	Eigen::Vector3d point;
	point << GlobalTranslation.Translation[0], GlobalTranslation.Translation[1], GlobalTranslation.Translation[2];
	return point;
}

Eigen::Quaterniond Vicon::getSegmentGlobalRotationQuaternion( std::string SubjectName, std::string SegmentName )
{
	Output_GetSegmentGlobalRotationQuaternion Output = MyClient.GetSegmentGlobalRotationQuaternion( SubjectName, SegmentName );

	Eigen::Quaterniond q (Output.Rotation[3], Output.Rotation[0], Output.Rotation[1], Output.Rotation[2]);

	return q;
}

unsigned int Vicon::getMarkerCount( std::string SubjectName ) 
{
	Output_GetMarkerCount Output;
	Output = MyClient.GetMarkerCount(SubjectName);
	return Output.MarkerCount;
}

std::string Vicon::getMarkerName( std::string SubjectName, unsigned int MarkerIndex )
{
	Output_GetMarkerName OutputGMN;
	OutputGMN = MyClient.GetMarkerName( SubjectName, MarkerIndex );
	return OutputGMN.MarkerName;
}

Eigen::Vector3d Vicon::getMarkerTranslation( std::string SubjectName, std::string MarkerName )
{
	Output_GetMarkerGlobalTranslation GlobalTranslation = 
				MyClient.GetMarkerGlobalTranslation( SubjectName, MarkerName );
	Eigen::Vector3d point;
	point << GlobalTranslation.Translation[0], GlobalTranslation.Translation[1], GlobalTranslation.Translation[2];
	return point;	
}

bool Vicon::hasMarkerWithName( std::string SubjectName, std::string MarkerName )
{
	Output_GetMarkerGlobalTranslation GlobalTranslation = 
				MyClient.GetMarkerGlobalTranslation( SubjectName, MarkerName );

	return (GlobalTranslation.Result != Result::InvalidMarkerName);	
}
