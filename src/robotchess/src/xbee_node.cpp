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



int16_t leftPWM = 0;
int16_t rightPWM = 0;
int32_t leftEncoder = 0;
int32_t rightEncoder = 0;

// Callbacks
void TorqueCallback(const robotchess::PWM::ConstPtr& msg) {
	if ( msg->leftPWM > 255 ) {
		leftPWM = 255;
	} else if ( msg->leftPWM < -255 ) {
		leftPWM = -255;
	} else {
		leftPWM = msg->leftPWM;
	}


	if ( msg->rightPWM > 255 ) {
		rightPWM = 255;
	} else if ( msg->rightPWM < -255 ) {
		rightPWM = -255;
	} else {
		rightPWM = msg->rightPWM;
	}

}

void RxCallback(struct xbee *xbee, struct xbee_con *con, struct xbee_pkt **pkt, void **data) 
{
	leftEncoder = ((int32_t *) ((*pkt)->data))[0];
	rightEncoder = ((int32_t *) ((*pkt)->data))[1];
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "chessbot");
	
	ros::NodeHandle n;
	ros::NodeHandle private_n("~");

	ros::Subscriber sub = n.subscribe( "torque", 10, TorqueCallback);

	ros::Publisher encoder_pub = n.advertise<robotchess::Encoder>("encoder", 10);

	struct xbee *xbee;
	struct xbee_con *con;
	struct xbee_conAddress address;
	xbee_err ret;

	if ((ret = xbee_setup(&xbee, "xbeeZB", "/dev/ttyUSB0", 57600)) != XBEE_ENONE) {
		printf("ret: %d (%s)\n", ret, xbee_errorToStr(ret));
		return ret;
	}

	memset(&address, 0, sizeof(address));
	address.addr64_enabled = 1;
	address.addr64[0] = 0x00;
	address.addr64[1] = 0x13;
	address.addr64[2] = 0xA2;
	address.addr64[3] = 0x00;
	address.addr64[4] = 0x40;
	address.addr64[5] = 0xDA;
	address.addr64[6] = 0xDE;
	address.addr64[7] = 0xD9;

	if ((ret = xbee_conNew(xbee, &con, "Data", &address)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conNew() returned: %d (%s)", ret, xbee_errorToStr(ret));
		return ret;
	}

	if ((ret = xbee_conDataSet(con, xbee, NULL)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conDataSet() returned: %d", ret);
		return ret;
	}

	if ((ret = xbee_conCallbackSet(con, RxCallback, NULL)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conCallbackSet() returned: %d", ret);
		return ret;
	}

	// ROS loop
	ros::Rate loop_rate(75); // 75 Hz
	
	while (ros::ok())
	{
		ros::spinOnce();

		uint8_t msg[3];
		msg[0] = 8;
		
		msg[1] = (leftPWM < 0) ? 1 : 0;
		msg[2] = (uint8_t) abs(leftPWM);

		msg[3] = (rightPWM < 0) ? 1 : 0;
		msg[4] = (uint8_t) abs(rightPWM);

		unsigned char retVal;
		xbee_connTx(con, &retVal, msg, 5);

		// struct xbee_pkt *pkt;
		// if ((ret = xbee_conRx(con, &pkt, NULL)) != XBEE_ENONE) {
		// 	printf("ret: %d (%s)\n", ret, xbee_errorToStr(ret));
		// 	return ret;
		// }
		// leftEncoder = pkt->dataLen;
		// if ((ret = xbee_pktFree(pkt)) != XBEE_ENONE) {
		// 	printf("ret: %d (%s)\n", ret, xbee_errorToStr(ret));
		// 	return ret;
		// }


		robotchess::Encoder emsg;
		emsg.leftEncoder = leftEncoder;
		emsg.rightEncoder = rightEncoder;
		encoder_pub.publish(emsg);

		loop_rate.sleep();

	}
	return 0;
}