#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Joy.h"
#include "create_driver/vicon_driver.h"
#include "geometry_msgs/Twist.h"

#include "create_controller/ControlMsgs.h"

#include <string>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

#define PI 3.141592
using namespace Eigen;
using namespace std;
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
	ros::init(argc, argv, "falcon_ellipse_feedback");
	
	ros::NodeHandle n;
	ros::NodeHandle private_n("~");

	// List of robot names
	vector<string> robot_names;
	XmlRpc::XmlRpcValue robot_list;
	private_n.getParam("robot_list", robot_list);
	ROS_ASSERT(robot_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for (int i = 0; i < robot_list.size(); i++) 
	{
		ROS_ASSERT(robot_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
		robot_names.push_back(static_cast<string>(robot_list[i]));
	}

	// Number of robots
	const int num_robots = robot_names.size();

	
	// ROS Subscribers
	map<string, create_driver::ViconStream> vicon;
	vector<string>::iterator name_it;
	vector<ros::Subscriber> sub;
	for (name_it = robot_names.begin(); name_it != robot_names.end(); name_it++)
	{
		vicon.insert(pair<string, create_driver::ViconStream>(*name_it, create_driver::ViconStream()));
		sub.push_back(n.subscribe("/" + *name_it + "/tf", 10, &create_driver::ViconStream::callback, &vicon[*name_it]));
	}

	sub.push_back(n.subscribe("position", 10, position_callback));

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

		double p[num_robots];
		double q[num_robots];

		for (int i = 0; i < num_robots; i++) {
			p[i] = cos(vicon[robot_names[i]].theta());
			q[i] = sin(vicon[robot_names[i]].theta());
		}

		double p_bar = 0.0;
		double q_bar = 0.0;

		for (int i = 0; i < num_robots; i++) {
			p_bar += p[i]/num_robots;
			q_bar += q[i]/num_robots;
		}

		double r = sqrt(pow(p_bar, 2) + pow(q_bar, 2))/2.0;

		double phi = atan2(q_bar, p_bar);

		double b = 0.2;

		double a = sqrt(pow(r, 2) + pow(b, 2));

		double x = joy(0) - r*cos(phi);
		double y = joy(1) - r*sin(phi);

		double x_barE = x * cos(phi) + y*sin(phi);
		double y_barE = -x * sin(phi) + y*cos(phi);

		double c = pow(x_barE,2) / pow(a,2) + pow(y_barE,2) / pow(b,2);

		double lowerbound = 1.0;
		double upperbound = 25.0;
		double gain = 0.0;
		if(lowerbound < c && c < upperbound){
			gain = (-500)*(pow(upperbound,2) - pow(lowerbound,2) )*(pow(c,2) - pow(lowerbound,2) ) / pow((pow(c,2) - pow(upperbound,2) ),3);

		} 
		
		double v_bar1 = -2 * x_barE/pow(a,2);
		double v_bar2 = -2 * y_barE/pow(b,2);

		double v_bar_norm = sqrt(pow(v_bar1, 2) + pow(v_bar2, 2));

		Vector3d v_bar(v_bar1*cos(phi)-v_bar2*sin(phi), v_bar1*sin(phi)+v_bar2*cos(phi), 0);
		force = gain*v_bar_norm*v_bar;
		// if (joy.norm()<0.25) {
		// 	resistance =1;
		// } else {
		// 	double radius = joy.norm() - 0.25;

		// 	double sum_etheta = fabs(etheta[0])+ fabs(etheta[1])+fabs(etheta[2])+fabs(etheta[3]);
		// 	resistance = 1; // + 6 * (radius * sum_etheta);
		// 	printf("%f\n", sum_etheta);
		// }
		
		// for (int i = 0; i < 3; i++)
		// {
		// 	force(i) = -5*joy(i)*resistance;
		// }

		geometry_msgs::Vector3 fmsg;
		fmsg.x = force(0);
		fmsg.y = force(1);
		fmsg.z = -joy(2)*5;
		force_pub.publish(fmsg);

		// Deadband
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