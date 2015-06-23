#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"


#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;
Vector3d position(-5,-5,-5);

void position_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	position(0) = msg->x;
	position(1) = msg->y;
	position(2) = msg->z;
}

int main(int argc, char **argv)
{
	// ROS Initalization
	ros::init(argc, argv, "control");
	
	ros::NodeHandle n;
	ros::NodeHandle private_n("~");

	ros::Subscriber position_sub = n.subscribe("position", 10, position_callback);
	ros::Publisher force_pub = n.advertise<geometry_msgs::Vector3>("force", 10);
	// ROS loop
	ros::Rate loop_rate(1000); // 1 kHz
	bool uninitialized = true;
	while (ros::ok())
	{
		ros::spinOnce();

		if (uninitialized)
		{
			if (position(0) < 0.015 && position(0) > -0.015 && position(1) < 0.015 && position(1) > -0.015) 
			{
				uninitialized = false;
				//std::cout << "initialized" << std::endl;
			}

		} else {
			double dist = 10000, m_stiffness = 1000;
			int closest = -1, outside=2, axis;

			Vector3d force;
			for (axis=0; axis<2; axis++)
			{
				force(axis) = 0;
				dist = 10000;
				if (position(axis) > 0.015 || position(axis) < -0.015)
				{
					double dA = position(axis)-(0.015);
					double dB = position(axis)-(-0.015);
					if (fabs(dA) > fabs(dB)) { dist = dB; } else { dist = dA; }
					//if (fabs(dA) < fabs(dist)) { dist = dA; closest = axis; }
					//if (fabs(dB) < fabs(dist)) { dist = dB; closest = axis; }
					//outside--;
					force(axis) = -m_stiffness*dist;
				}
			}
			force(2) = 0;

			geometry_msgs::Vector3 msg;
			msg.x = force(0);
			msg.y = force(1);
			msg.z = force(2);
			force_pub.publish(msg);
		}

		loop_rate.sleep();
	}
	
	return 0;
}