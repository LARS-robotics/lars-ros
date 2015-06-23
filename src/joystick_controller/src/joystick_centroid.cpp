#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include <sensor_msgs/Joy.h>


class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int linear_, angular_, radius_;
  double l_scale_, a_scale_, r_scale_;
  ros::Publisher centroid_pub_;
  ros::Subscriber joy_sub_;
  
};


TeleopTurtle::TeleopTurtle():
  linear_(1),
  angular_(2),
  radius_(3)
  
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  
  nh_.param("axis_radius", radius_, radius_);
  nh_.param("scale_radius", r_scale_, r_scale_);


  centroid_pub_ = nh_.advertise<geometry_msgs::Point>("centroid", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);

}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Point centroid;
  centroid.x = a_scale_*joy->axes[angular_];
  centroid.y = l_scale_*joy->axes[linear_];
  centroid.z = r_scale_*joy->axes[radius_];
  centroid_pub_.publish(centroid);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_centroid");
  TeleopTurtle teleop_turtle;

  ros::spin();
}
