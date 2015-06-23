/*
 * Tcpip.cpp
 * iRobot Server
 * 
 * Created by Hazen Eckert on 10/29/11.
 * The Laboratory for Autonomous Robotics and Systems. 
 * The University of Texas at Dallas.
 *
 */
 
#ifndef _Vicon_H
#define _Vicon_H

#include "vicon/Client.h"
#include "ros/ros.h"
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>


#define DEFAULT_IP "192.168.2.53:801"
#define DEFAULT_PORT 801

class Vicon {

public:
	Vicon( std::string viconIP );
	~Vicon();
	void getFrame();
	unsigned int getSubjectCount();
	std::string getSubjectName( unsigned int SubjectIndex );
	std::string getSubjectRootSegmentName( std::string SubjectName );
	unsigned int getSegmentCount( std::string SubjectName );
	std::string getSegmentName( std::string SubjectName, unsigned int SegmentIndex );
	Eigen::Vector3d getSegmentGlobalTranslation( std::string SubjectName, std::string SegmentName );
	Eigen::Quaterniond getSegmentGlobalRotationQuaternion( std::string SubjectName, std::string SegmentName );
	unsigned int getMarkerCount( std::string SubjectName );
	std::string getMarkerName( std::string SubjectName, unsigned int MarkerIndex );
	Eigen::Vector3d getMarkerTranslation( std::string SubjectName, std::string MarkerName );
	bool hasMarkerWithName( std::string SubjectName, std::string MarkerName );
	
private:
	ViconDataStreamSDK::CPP::Client MyClient;

};

#endif 
