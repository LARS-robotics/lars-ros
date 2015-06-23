// ROS includes
#include "ros/ros.h"
#include "ros/assert.h"

// Library includes
#include <xbee.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// Local Package includes
#include "robotchess/PWM.h"
#include "robotchess/Encoder.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control");
	
	ros::NodeHandle n;
	ros::NodeHandle private_n("~");



	ros::Publisher torque_pub = n.advertise<robotchess::PWM>("torque", 10);

	// ROS loop
	ros::Rate loop_rate(75); // 75 Hz

	ros::Time begin = ros::Time::now();
	ros::Duration d(1.5);

	robotchess::PWM emsg;
	emsg.leftPWM = 0;
	emsg.rightPWM = 0;
	torque_pub.publish(emsg);
	int speed = 255;

	while (ros::ok())
	{
		ros::spinOnce();

		if ( ros::Time::now() > begin + d*4 ) {
			emsg.leftPWM = 0;
			emsg.rightPWM = 0;
			torque_pub.publish(emsg);

		} else if ( ros::Time::now() > begin + d*3 ) {
			emsg.leftPWM = speed;
			emsg.rightPWM = -speed;
			torque_pub.publish(emsg);

		} else if ( ros::Time::now() > begin + d*2 ) {
			emsg.leftPWM = speed;
			emsg.rightPWM = speed;
			torque_pub.publish(emsg);

		} else if ( ros::Time::now() > begin + d ) {
			emsg.leftPWM = -speed;
			emsg.rightPWM = speed;
			torque_pub.publish(emsg);

		}  

		
		loop_rate.sleep();
	}
	return 0;
}