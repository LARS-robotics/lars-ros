#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Joy.h"

#include "create_controller/ControlMsgs.h"

#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

#define PI 3.141592
using namespace Eigen;
Vector3d position(-5,-5,-5);
double etheta[4]={0.0,0.0,0.0,0.0};

std::string robot1_name;
std::string robot2_name;
std::string robot3_name;
std::string robot4_name;

void Callbackzero(const create_controller::ControlMsgs::ConstPtr& msg)
{
	etheta[0] = fmod((msg->etheta), 2*PI);
	if (etheta[0] > PI) {
		etheta[0] = 2*PI-etheta[0];
	}
}
void Callbackone(const create_controller::ControlMsgs::ConstPtr& msg)
{
	etheta[1] = fmod((msg->etheta), 2*PI);
	if (etheta[1] > PI) {
		etheta[1] = 2*PI-etheta[1];
	}
}
void Callbacktwo(const create_controller::ControlMsgs::ConstPtr& msg)
{
	etheta[2] = fmod((msg->etheta), 2*PI);
	if (etheta[2] > PI) {
		etheta[2] = 2*PI-etheta[2];
	}
}
void Callbackthree(const create_controller::ControlMsgs::ConstPtr& msg)
{
	etheta[3] = fmod((msg->etheta), 2*PI);
	if (etheta[3] > PI) {
		etheta[3] = 2*PI-etheta[3];
	}
}

void position_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	position(0) = msg->x;
	position(1) = msg->y;
	position(2) = msg->z;
}

int main(int argc, char **argv)
{
	// ROS Initalization
	ros::init(argc, argv, "falcon_to_joy");
	
	ros::NodeHandle n;
	ros::NodeHandle private_n("~");

	private_n.param<std::string>("robot1name", robot1_name, "gandhi");
	private_n.param<std::string>("robot2name", robot2_name, "alexander");
	private_n.param<std::string>("robot3name", robot3_name, "suleyman");
	private_n.param<std::string>("robot4name", robot4_name, "hiawatha");

	ros::Subscriber position_sub = n.subscribe("position", 10, position_callback);
	ros::Subscriber subzero = n.subscribe(  robot1_name + "/dia", 10, Callbackzero);
	ros::Subscriber subone =  n.subscribe(  robot2_name + "/dia", 10, Callbackone);
	ros::Subscriber subtwo =  n.subscribe(  robot3_name + "/dia", 10, Callbacktwo);
	ros::Subscriber subthree =n.subscribe(  robot4_name + "/dia", 10, Callbackthree);
	ros::Publisher joy_pub = n.advertise<sensor_msgs::Joy>("joy", 10);
	ros::Publisher force_pub = n.advertise<geometry_msgs::Vector3>("force", 10);

	// ROS loop
	ros::Rate loop_rate(1000); // 1 kHz
	bool uninitialized = true;
	while (ros::ok())
	{
		ros::spinOnce();

		Vector3d joy(position(0)/0.06,  position(1)/0.06, (2.0*(position(2)-0.073))/0.1035 - 1.0);

		Vector3d force;
		double resistance;

		if (joy.norm()<0.25) {
			resistance =1;
		} else {
			double radius = joy.norm() - 0.25;
			double sum_etheta = fabs(etheta[0])+ fabs(etheta[1])+fabs(etheta[2])+fabs(etheta[3]);
			resistance = 1 ;//+6*(radius*sum_etheta);
			printf("%f\n", sum_etheta);
		}
		
		for (int i = 0; i < 3; i++)
		{
			force(i) = -5*joy(i)*resistance;
		}

		geometry_msgs::Vector3 fmsg;
		fmsg.x = force(0);
		fmsg.y = force(1);
		fmsg.z = force(2);
		force_pub.publish(fmsg);

		if (joy.norm() < 0.25) {
			for (int i = 0; i < 2; ++i)
			{
				joy(i) =0;
			}
		}
		
		
		sensor_msgs::Joy jmsg;
		jmsg.axes.resize(3);
		jmsg.axes[0] = -joy(0);//position(0)/0.06; // -1.0 to 1.0
		jmsg.axes[1] = joy(1);//position(1)/0.06;
		jmsg.axes[2] = joy(2);//(2.0*(position(2)-0.073))/0.1035 - 1.0;//32767
		joy_pub.publish(jmsg);

		
		

		loop_rate.sleep();
	}
	
	return 0;
}