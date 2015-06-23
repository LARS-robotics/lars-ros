/*
 * CreateDriver.h
 * create_driver
 *
 * Created by Hazen Eckert on 04/14/12.
 * The Laboratory for Autonomous Robotics and Systems. 
 * The University of Texas at Dallas.
 *
 */ 
 
#ifndef _CreateDriver_H
#define _CreateDriver_H

enum RobotState { OFF, PASSIVE, SAFE, FULL };
enum State { OPEN, CLOSED, RUNNING };

class CreateDriver {

public:
	CreateDriver( const char *hostname );
	~CreateDriver();
	void doOpen();
	void doClose();
	void doStart();
	void doStop();
	State getState();
	void setRobotState( RobotState state );
	RobotState getRobotState();
	void directDrive( int rightVelocity, int leftVelocity );
	void drive( int velocity, int radius );
	
protected:
	int sockfd;
	const char *hostname;
	State state;
	RobotState rstate;
	
};

#endif 
