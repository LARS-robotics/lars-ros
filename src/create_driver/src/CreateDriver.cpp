/*
 * CreateDriver.cpp
 * create_driver
 *
 * Created by Hazen Eckert on 04/14/12.
 * The Laboratory for Autonomous Robotics and Systems. 
 * The University of Texas at Dallas.
 *
 */ 
 
#include "create_driver/CreateDriver.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <algorithm> 

#include "create_driver/OpenInterface.h"

 #define MAX_VELOCITY 500

CreateDriver::CreateDriver( const char *hostname )
{
	this->rstate = OFF;
	this->state = CLOSED;
	this->hostname = hostname;
}

CreateDriver::~CreateDriver()
{
	this->doClose();
}

void CreateDriver::doOpen()
{
	if( this->state != CLOSED )
		return;
		
	// Setup TCP Connection
	int portNumber = 2852;
	
	struct sockaddr_in serv_addr;
	struct hostent *server;
	
	this->sockfd = socket(AF_INET, SOCK_STREAM, 0);
	
	if (this->sockfd < 0) {
		fprintf(stderr,"ERROR: Unable to open socket\n");
		exit(1);
	}
		
	server = gethostbyname(this->hostname);
	
	if (server == NULL) {
		fprintf(stderr,"ERROR: No such host\n");
		exit(1);
	}
	
	bzero((char *) &serv_addr, sizeof(serv_addr));
	
	serv_addr.sin_family = AF_INET;
	
	bcopy((char *)server->h_addr, 
		(char *)&serv_addr.sin_addr.s_addr,
			server->h_length);
			
	serv_addr.sin_port = htons(portNumber);
	
	if (connect(this->sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) {
		fprintf(stderr,"ERROR:Cannot connect\n");
		exit(1);
	}
	this->state = OPEN;
}

void CreateDriver::doClose()
{
	if( this->state != OPEN )
		return;
	
	char buffer[255];
	bzero(buffer, 255);
	buffer[0] = 0xFF;
	write(this->sockfd, buffer, 1);
	close(this->sockfd);
	this->state = CLOSED;
}

void CreateDriver::doStart()
{
	if( this->state != OPEN )
		return;
		
	char buffer[255];
	bzero(buffer, 255);
	buffer[0] = 0x01;
	buffer[1] = CmdStart;
	write(this->sockfd, buffer, 2);
	read(this->sockfd, buffer, 255);
	this->rstate = PASSIVE;
	this->state = RUNNING;
	
}

void CreateDriver::doStop()
{
	if( this->state != RUNNING )
		return;
		
	this->directDrive( 0, 0 );
	this->state = OPEN;
}

State CreateDriver::getState()
{
	return this->state;
}

void CreateDriver::setRobotState( RobotState rstate )
{
	if ( this->rstate == rstate || rstate == OFF || rstate == PASSIVE )
		return;
		
	char buffer[255];
	bzero(buffer, 255);
	buffer[0] = 0x01;
	if ( rstate == SAFE ) {
		buffer[1] = CmdSafe;
		this->rstate = SAFE;
	} else {
		buffer[1] = CmdFull;
		this->rstate = FULL;
	}
	write(this->sockfd, buffer, 2);
	read(this->sockfd, buffer, 255);
}

RobotState CreateDriver::getRobotState()
{
	return this->rstate;
}

void CreateDriver::directDrive( int rightVelocity, int leftVelocity )
{
	if( this->state != RUNNING )
		return;

	int diffV = rightVelocity - leftVelocity;
	int avgV = (int) ((rightVelocity + leftVelocity)/2.0);

	if (std::max(abs(rightVelocity), abs(leftVelocity)) > MAX_VELOCITY) // If one of the wheel velocities exceeds the limit
	{
		if (diffV <= 2*MAX_VELOCITY) // If the difference in wheel velocities does not exceed the max difference
		{
			if (avgV > 0) // If the linear velocity component is positive 
			{
				int decrease = std::max(rightVelocity, leftVelocity) - MAX_VELOCITY;
				rightVelocity -= decrease;
				leftVelocity -= decrease;
			} else { // If the linear velocity component is negative 
				int increase = -(MAX_VELOCITY + std::min(rightVelocity, leftVelocity));
				rightVelocity += increase;
				leftVelocity += increase;
			}
		} else { // If the difference in wheel velocities exceeds the max difference
			if (rightVelocity > leftVelocity) {
				rightVelocity = MAX_VELOCITY;
				leftVelocity = -MAX_VELOCITY;
			} else {
				rightVelocity = -MAX_VELOCITY;
				leftVelocity = MAX_VELOCITY;
			}
		}
	}
		
	uint16_t rv = rightVelocity & 0xFFFF;
	
	uint16_t lv = leftVelocity & 0xFFFF;
	
	char buffer[255];
	bzero(buffer, 255);
	
	buffer[0] = 0x01;
	buffer[1] = CmdDriveWheels;		
	buffer[2] = (uint8_t)((rv >> 8) & 0x00FF);
	buffer[3] = (uint8_t)(rv & 0x00FF);
	buffer[4] = (uint8_t)((lv >> 8) & 0x00FF);
	buffer[5] = (uint8_t)(lv & 0x00FF);	
	
	write(this->sockfd, buffer, 6);
	read(this->sockfd, buffer, 255);
}

void CreateDriver::drive( int velocity, int radius )
{
	if( this->state != RUNNING )
		return;
		
	if ( velocity > 500 )
		velocity = 500;
	
	if ( velocity < -500 )
		velocity = -500;
		
	if ( radius > 2000 )
		radius = 2000;
		
	if ( radius < -2000 )
		radius = -2000;
		
	uint16_t v = velocity & 0xFFFF;
	
	uint16_t r = radius & 0xFFFF;
	
	char buffer[255];
	bzero(buffer, 255);
	
	buffer[0] = 0x01;
	buffer[1] = CmdDrive;		
	buffer[2] = (uint8_t)((v >> 8) & 0x00FF);
	buffer[3] = (uint8_t)(v & 0x00FF);
	buffer[4] = (uint8_t)((r >> 8) & 0x00FF);
	buffer[5] = (uint8_t)(r & 0x00FF);	
	
	write(this->sockfd, buffer, 6);
	read(this->sockfd, buffer, 255);
}





















