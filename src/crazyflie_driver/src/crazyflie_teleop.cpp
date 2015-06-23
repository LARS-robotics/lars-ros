#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include <sensor_msgs/Joy.h>
#include "crazyflie_driver/RPYT.h"


class TeleopCrazyflie
{
public:
  TeleopCrazyflie();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int roll_, pitch_, yaw_, thrust_;
  ros::Publisher pub;
  ros::Subscriber joy_sub_;
  
};


TeleopCrazyflie::TeleopCrazyflie():
  roll_(0),
  pitch_(1),
  yaw_(2),
  thrust_(3)
  
{

  //nh_.param("axis_linear", linear_, linear_);
  //nh_.param("axis_angular", angular_, angular_);
  //nh_.param("scale_angular", a_scale_, a_scale_);
  //nh_.param("scale_linear", l_scale_, l_scale_);
  
  //nh_.param("axis_radius", radius_, radius_);
  //nh_.param("scale_radius", r_scale_, r_scale_);


  pub = nh_.advertise<crazyflie_driver::RPYT>("rotation_desired", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopCrazyflie::joyCallback, this);

}

void TeleopCrazyflie::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  
  double roll = (joy->axes[roll_])*-30.0;
  
  double pitch = (joy->axes[pitch_])*30.0;
  double yaw = (joy->axes[yaw_]);
  if ((yaw < -0.2) || (yaw > 0.2)) {
  	if (yaw < 0.0) {
      yaw = (yaw + 0.2) * 200 * 1.25;
    } else {
      yaw = (yaw - 0.2) * 200 * 1.25;
    }
  } else {
    yaw = 0.0;
  }
  double percent_to_thrust = (1.0/100.0)*65365.0;
  double thrust = (joy->axes[thrust_]);
  if (thrust < 0.05) {
    thrust = 0.0;
  } else {
    thrust = 25.0 + thrust*(100.0 - 25.0) ;
  }
  //ROS_ERROR("%f, %f, %f, %f\n",  roll, pitch, yaw, thrust );
  crazyflie_driver::RPYT msg;
  msg.roll = (float) roll;
  msg.pitch = (float) pitch;
  msg.yaw = (float) yaw;
  msg.thrust = (float) thrust;
  pub.publish(msg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_control");
  TeleopCrazyflie teleop_crazy;

  ros::spin();
}
