// ROS includes
#include "ros/ros.h"
#include "ros/assert.h"
#include "dynamic_reconfigure/server.h"
#include "create_driver/vicon_driver.h"
#include "geometry_msgs/Twist.h"

// Library includes
#include <string>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/LU>

// Local Package includes
#include "to_velocity_tracking/ControlVariablesConfig.h"
#include "to_velocity_tracking/ControlMsgs.h"

using namespace std;
using namespace Eigen;

// Constants
const double PI = 3.1415926535;
const double wmax = 8;

// Dynamic reconfigure variables
double k = 0.5; // linear velocity gain
double ktheta = 5.0; // angular velocity gain
bool power = true;
double epsilon = 0.25;
bool u_switch = true;

void reconfigureCallback(to_velocity_tracking::ControlVariablesConfig &config, uint32_t level) {
	k = config.k;
	ktheta = config.ktheta;
	power = config.power;
	epsilon = config.epsilon;
	u_switch = config.u_switch;
}

int main(int argc, char **argv)
{
	// ROS Initalization
	ros::init(argc, argv, "control");
	
	ros::NodeHandle n;
	ros::NodeHandle private_n("~");
	
	// ROS Parameters

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

	// Robot name
	string robot_name;
	private_n.param<std::string>("robotname", robot_name, "defaultname");

	int index = find(robot_names.begin(), robot_names.end(), robot_name) - robot_names.begin();

	// ROS Dynamic Reconfigure

	dynamic_reconfigure::Server<to_velocity_tracking::ControlVariablesConfig> server;
  	dynamic_reconfigure::Server<to_velocity_tracking::ControlVariablesConfig>::CallbackType f;
  	f = boost::bind(&reconfigureCallback, _1, _2);
 	server.setCallback(f);

	// ROS Subscribers
	map<string, create_driver::ViconStream> vicon;
	vector<string>::iterator name_it;
	vector<ros::Subscriber> sub;
	for (name_it = robot_names.begin(); name_it != robot_names.end(); name_it++)
	{
		vicon.insert(pair<string, create_driver::ViconStream>(*name_it, create_driver::ViconStream()));
		sub.push_back(n.subscribe("/" + *name_it + "/tf", 10, &create_driver::ViconStream::callback, &vicon[*name_it]));
	}


	// ROS Publishers
	ros::Publisher vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	ros::Publisher dia = n.advertise<to_velocity_tracking::ControlMsgs>("dia", 10);

	// ROS loop
	ros::Rate loop_rate(250); // 250 Hz
	
	while (ros::ok())
	{
		// Retrieve Vicon Data
		ros::spinOnce();

		/* An instance of this code is run on each robot in the formation.
		 *
		 * You can access each robot's position and oreintation through the following statements:
		 * vicon[robot_names[i]].x()
		 * vicon[robot_names[i]].y()
		 * vicon[robot_names[i]].theta()
		 * 
		 * The index of the robot running the instance of the code is contain in the "index" 
		 * variable. For example, if you wanted to get the distance of this robot from all of its
		 * neighbors you could run the following:
		 * double dist[num_robots];
		 * for (int i = 0; i < num_robots; i++) {
		 * 		dist[i] = sqrt(pow(vicon[robot_names[i]].x() - vicon[robot_names[index]].x(),2) 
		 * 				     + pow(vicon[robot_names[i]].y() - vicon[robot_names[index]].y(),2));
		 * }
		 */
		double ux = 0;
		double uy = 0;

		if (u_switch) {
			ux = 1;
			uy = 1;
		} else {
			ux = -1;
			uy = -1;
		}

		double thetad = atan2(uy, ux);
		double theta = vicon[robot_name].theta();
		double etheta = theta - thetad;

		double v = k*1000*cos(etheta)*sqrt(pow(ux,2) + pow(uy,2)); // linear velocity output
		double w = -ktheta*etheta; // angular velocity output



		if (fabs(etheta) > epsilon){
			v = 0;
			w = (etheta > 0) ? -wmax : wmax;			
		}
		
		if ( power ) {
		// Send wheel velocities to driver
			geometry_msgs::Twist msg;
			msg.linear.x = v;
			msg.linear.y = 0;
			msg.linear.z = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;
			msg.angular.z = w;
			vel.publish(msg);
		} else {
			geometry_msgs::Twist msg;
			msg.linear.x = 0;
			msg.linear.y = 0;
			msg.linear.z = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;
			msg.angular.z = 0;
			vel.publish(msg);
		}		

		to_velocity_tracking::ControlMsgs dmsg; 
		dmsg.theta = theta;

		dia.publish(dmsg);

		loop_rate.sleep();
	}
	
	return 0;
}