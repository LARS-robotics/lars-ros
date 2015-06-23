//%% Author : Jingfu Jin @ UT at Dallas%%//
//%% Time : April. 9, 2015%%//
//#include "stdafx.h"
#include <iostream>
#include "jingfu/control_formation.h"
#include <cmath>
#include <algorithm>    // std::min
using namespace std;
#include<fstream> // file stream





control_formation::control_formation()
{
	


	d_ro = 0;					// distance between the robot and obstacle
	d_rg = 0;					// distance to the goal

	p_ro(0,0);					// repulsive vector
	p_rg(0,0);					// attractive vector
	pu(0,0);
	hd = 0;

	v_input.l_v = 0;
	v_input.a_v = 0;			// set the initial control input to zero


	/// control gain///
	kk = 1;
	kx = kk*1;
	ky = kk*1.5;
	kt = kk*1;
	zeta = 0;

}

control_formation::~control_formation()
{

}

bool control_formation::init_formation()
{

	//////////////////
	return true;
}

double control_formation::switchingSignal(double d_ro, double h_ro)
{
	
	double R = 0.95;
	double r = 0.4;
	if (abs(h_ro) >= PI/2)
		zeta = 0;
	else
	{
		if (d_ro > min(R,r/sin(abs(h_ro))))
		//if (p_ro > min(R,r/sin(abs(h_ro)))&&(zeta_in==0))
			zeta = 0;
		else
			zeta = 1;
	}
	return zeta;
}

// convert the angle in [ -PI ~ PI]
double control_formation::between2PI(const double tAngle)
{		
	double angle;
	angle=tAngle;
	while(fabs(angle)>PI)
	{
		if(angle>PI)
		{
			angle-=2.0*PI;
		}
		else
		{
			angle+=2.0*PI;
		}
	}
	return angle;
}
/*
void control_formation::desired_shape(char shape,int number,int k)
{

}
*/

Wheel control_formation::nonswitching_w_obstacle(pose2D q1v, pose2D q1v_neighbor, pose2D q, point2D po, Wheel vd)
{
	
	MatrixXd J_pseudoInv(2,2);     //  pseudo inverse
	MatrixXd I2(2,2);              //  2 by 2 identity matrix
	I2 << 1,0,
		0,1;
	MatrixXd N_ro(2,2);            //  NULL space projection
	N_ro = I2;

	Vector2d pd;

	
	// which is same as pd = (q2(1:2)+q3(1:2))-2*q1(1:2);
	pd(0) = q1v_neighbor.x - q1v.x;
	pd(1) = q1v_neighbor.y - q1v.y;
	hd =    q1v_neighbor.h - q1v.h;

	// repulsive vector
	p_ro(0) = q.x - po.x; // in R^1 
	p_ro(1) = q.y - po.y; // in R^1
	
	
	//double d_ro1 = sqrt((q.x - po.x)*(q.x - po.x) + (q.y - po.y)*(q.y - po.y)); // distance between robot and obstacle
	d_ro = p_ro.norm();	                    // distance between robot and obstacle

	h_o  = atan2(-p_ro(1), -p_ro(0));		// The direction of the robot-obstacle w/t World coordinate frame

	h_ro = q.h-h_o;						    // relative angle between the robot and obstacle
	h_ro = between2PI(h_ro);                // turn the angle into -pi ~ +pi

	zeta = switchingSignal(d_ro, h_ro);     // switching signal

	
	// Calculating Moore–Penrose pseudoinverse of J_rep using SVD method
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(p_ro, Eigen::ComputeThinU |Eigen::ComputeThinV);
	J_pseudoInv = svd.matrixV()*svd.singularValues().inverse()*svd.matrixU().transpose();
	//std::cout <<J_pseudoInv <<endl;
	N_ro = (I2 - p_ro*J_pseudoInv);							                // null space of repulsive vector

	//	cout << "n_ro =>" << N_ro << endl;
	pu = (1-zeta)*pd + 2*zeta*N_ro*p_ro; //
	
	double h_err = zeta*(atan2(pu(1), pu(0)) - q1v.h) + (1-zeta)*hd;		// Desired heading error


	/// transform the error into robot's local coordinate frame ///
	e_x = cos(q.h)*pu(0)+sin(q.h)*pu(1);
	e_y = -sin(q.h)*pu(0)+cos(q.h)*pu(1);
	e_theta = h_err;
	//////////////////////////////////////////////////////////////
    /// beginning of control law ///
	v_input.l_v = kx*e_x + vd.l_v*cos(e_theta);
	v_input.a_v = vd.a_v + vd.l_v*ky*e_y + kt*sin(e_theta);
	/// end of control ///

	return v_input;

}


Wheel control_formation::switching_w_obstacle(pose2D q1v, pose2D q1v_neighbor, pose2D q, point2D po, Wheel vd)
{
	
	MatrixXd J_pseudoInv(2,2);     //  pseudo inverse
	MatrixXd I2(2,2);              //  2 by 2 identity matrix
	I2 << 1,0,
		0,1;
	MatrixXd N_ro(2,2);            //  NULL space projection
	N_ro = I2;

	Vector2d pd;


	// which is same as pd = (q2(1:2)+q3(1:2))-2*q1(1:2);
	pd(0) = q1v_neighbor.x - q1v.x;
	pd(1) = q1v_neighbor.y - q1v.y;
	hd = q1v_neighbor.h - q1v.h;
	//hd = Pi/3 - q1v.h;

	// repulsive vector
	p_ro(0) = q.x - po.x; // in R^1 
	p_ro(1) = q.y - po.y; // in R^1
	
	
	//double d_ro1 = sqrt((q.x - po.x)*(q.x - po.x) + (q.y - po.y)*(q.y - po.y)); // distance between robot and obstacle
	d_ro = p_ro.norm();	                // distance between robot and obstacle

	h_o  = atan2(-p_ro(1), -p_ro(0));	// The direction of the robot-obstacle w/t World coordinate frame

	h_ro = q.h-h_o;				// relative angle between the robot and obstacle
	h_ro = between2PI(h_ro);                // turn the angle into -pi ~ +pi

	zeta = switchingSignal(d_ro, h_ro);     // switching signal

	
	// Calculating Moore–Penrose pseudoinverse of J_rep using SVD method
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(p_ro, Eigen::ComputeThinU |Eigen::ComputeThinV);
	J_pseudoInv = svd.matrixV()*svd.singularValues().inverse()*svd.matrixU().transpose();
	//std::cout <<J_pseudoInv <<endl;
	N_ro = (I2 - p_ro*J_pseudoInv);							                // null space of repulsive vector

//	cout << "n_ro =>" << N_ro << endl;
	pu = (1-zeta)*pd + 2*zeta*N_ro*p_ro; //	
	v_input.l_v = cos(q1v.h)*tanh(pu(0)) + sin(q1v.h)*tanh(pu(1));  //  linear control input
	double gamma;
	if (abs(v_input.l_v) > 0.02)
	{
		gamma =  1;
	}
	else
	{
		gamma = 0;
	}
	

	double h_err = gamma*(atan2(pu(1), pu(0)) - q1v.h) + (1-gamma)*hd;		// Desired heading error


	v_input.a_v = 2*sin(h_err); // angular control input
	/// end of control ///

	return v_input;

}

Wheel control_formation::singleRobot(pose2D qd, pose2D q)
{
	Vector2d pd;

	// which is same as pd = (q2(1:2)+q3(1:2))-2*q1(1:2);
	pd(0) = qd.x - q.x;
	pd(1) = qd.y - q.y;
	
	int k1 = 2; int k2 = 2;
	v_input.l_v = k1*tanh(cos(q.h)*pd(0) + sin(q.h)*pd(1));
	double h_err = atan2(pd(1), pd(0)) - q.h;
	v_input.a_v = k2*sin(h_err);
	zeta = 0;
	return v_input;
}


Wheel control_formation::single_w_obstacle(pose2D q1v_neighbor, pose2D q, point2D po)
{
	
	MatrixXd J_pseudoInv(2,2);     //  pseudo inverse
	MatrixXd I2(2,2);              //  2 by 2 identity matrix
	I2 << 1,0,
		0,1;
	MatrixXd N_ro(2,2);            //  NULL space projection
	N_ro = I2;

	Vector2d pd;


	// which is same as pd = (q2(1:2)+q3(1:2))-2*q1(1:2);
	pd(0) = q1v_neighbor.x - q.x;
	pd(1) = q1v_neighbor.y - q.y;
	
	// repulsive vector
	p_ro(0) = q.x - po.x; // in R^1 
	p_ro(1) = q.y - po.y; // in R^1
	
	
	//double d_ro1 = sqrt((q.x - po.x)*(q.x - po.x) + (q.y - po.y)*(q.y - po.y)); // distance between robot and obstacle
	d_ro = p_ro.norm();	                // distance between robot and obstacle

	h_o  = atan2(-p_ro(1), -p_ro(0));	// The direction of the robot-obstacle w/t World coordinate frame

	h_ro = q.h-h_o;				// relative angle between the robot and obstacle
	h_ro = between2PI(h_ro);                // turn the angle into -pi ~ +pi

	zeta = switchingSignal(d_ro, h_ro);     // switching signal

	
	// Calculating Moore–Penrose pseudoinverse of J_rep using SVD method
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(p_ro, Eigen::ComputeThinU |Eigen::ComputeThinV);
	J_pseudoInv = svd.matrixV()*svd.singularValues().inverse()*svd.matrixU().transpose();
	//std::cout <<J_pseudoInv <<endl;
	N_ro = (I2 - p_ro*J_pseudoInv);							                // null space of repulsive vector

//	cout << "n_ro =>" << N_ro << endl;
	pu = (1-zeta)*pd + 2*zeta*N_ro*p_ro; //	
	v_input.l_v = tanh(cos(q.h)*pu(0) + sin(q.h)*pu(1));  //  linear control input
	
	double h_err = atan2(pu(1), pu(0)) - q.h;		// Desired heading error


	v_input.a_v = 2*sin(h_err); // angular control input
	/// end of control ///

	return v_input;

}
