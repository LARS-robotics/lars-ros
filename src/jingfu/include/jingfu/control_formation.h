//#include "stdafx.h"
#include <stdio.h>
#include <cmath>
//#include "mobileRobotSystem.h"
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace Eigen;
using Eigen::MatrixXd;
using namespace std;
#ifndef PI
#define PI 3.1415926
#endif

class pose2D
{
public:
	double x;
	double y;
	double h; // heading angle theta
};


class point2D
{
public:
	double x;
	double y;
};

class Wheel
{
public:
	double l_v; // linear velocity
	double a_v; // angular velocity
};


class control_formation
{
public:
	control_formation();
	~control_formation();
	bool init_formation();
	double switchingSignal(double d_ro, double h_oc);
	double between2PI(const double tAngle);
	//void desired_shape(char shape,int number,int k);
	Wheel nonswitching_w_obstacle(pose2D qv, pose2D qv_neighbor, pose2D q, point2D po, Wheel vd);
	Wheel switching_w_obstacle(pose2D qv, pose2D qv_neighbor, pose2D q, point2D po, Wheel vd); 
	Wheel singleRobot(pose2D qd, pose2D q);
	Wheel single_w_obstacle(pose2D qd, pose2D q, point2D po);
	double zeta;
private:

	Wheel v_input;			// control input or target velocity

	/////////error posture/////////
	double e_x;
	double e_y;
	double e_theta; 
	///////////////////////////////


	double d_ro;	// distance between the robot and obstacle
	double d_rg;	// distance to the goal

	//point2D p_ro; // repulsive vector
	Vector2d p_ro;	// repulsive vector
	//point2D p_rg; // attractive vector
	Vector2d p_rg;	// attractive vector
	Vector2d pu; 
	double hd;
	
	/////////////////////
	//double zeta;
	double h_o;	// theta_o
	double h_ro;	// theta_ro
	////////////////////

	/// control gain///
	double kk;
	double kx;
	double ky;
	double kt;

};
