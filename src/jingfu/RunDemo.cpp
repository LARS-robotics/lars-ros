// RunDemo.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include "control_formation.h"
//#include "mobileRobotSystem.h"
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace Eigen;
using Eigen::MatrixXd;
using namespace std;
#include <iomanip>
using std::setw;
#include <cmath>
int _tmain(int argc, _TCHAR* argv[])
{

	/// initial poses from Vicon///

	pose2D q10; q10.x = -2.0423; q10.y = -3.5116; q10.h = 0;
	pose2D q20; q20.x = -0.1945; q20.y = -9.3454; q20.h =  1.5708;
	pose2D q30; q30.x = 2.4694;  q30.y = 0.9643;  q30.h = -1.5708;
	pose2D q40; q40.x = 6.6571;  q40.y = -4.5076; q40.h =  3.1416;


	
	/// initlal offests /// 
	double k = 2; // radius

	pose2D q1off; q1off.x = k*cos(1*PI/4); q1off.y = k*sin(1*PI/4); q1off.h = 0; 
	pose2D q2off; q2off.x = k*cos(3*PI/4); q2off.y = k*sin(3*PI/4); q2off.h = 0; 
	pose2D q3off; q3off.x = k*cos(5*PI/4); q3off.y = k*sin(5*PI/4); q3off.h = 0; 
	pose2D q4off; q4off.x = k*cos(7*PI/4); q4off.y = k*sin(7*PI/4); q4off.h = 0; 
    /// end of initial offsets ///

	
	double df12 = sqrt((q1off.x - q2off.x)*(q1off.x - q2off.x) + (q1off.y - q2off.y)*(q1off.y - q2off.y));
	double df13 = sqrt((q1off.x - q3off.x)*(q1off.x - q3off.x) + (q1off.y - q3off.y)*(q1off.y - q3off.y));
	double df14 = sqrt((q1off.x - q4off.x)*(q1off.x - q4off.x) + (q1off.y - q4off.y)*(q1off.y - q4off.y));

	
	double df23 = sqrt((q2off.x - q3off.x)*(q2off.x - q3off.x) + (q2off.y - q3off.y)*(q2off.y - q3off.y));
	double df24 = sqrt((q2off.x - q4off.x)*(q2off.x - q4off.x) + (q2off.y - q4off.y)*(q2off.y - q4off.y));
	
	double df34 = sqrt((q3off.x - q4off.x)*(q3off.x - q4off.x) + (q3off.y - q4off.y)*(q3off.y - q4off.y));

	pose2D q10_v; pose2D q20_v; pose2D q30_v; pose2D q40_v; 

	Vector2d p12; Vector2d p13; Vector2d p14; //Vector2d p15; 
	Vector2d p23; Vector2d p24; //Vector2d p25; 
	Vector2d p34; //Vector2d p35;
	//Vector2d p45;

	double d12=0; double d13=0; double d14=0; 
	double d23=0;  double d24=0;
	double d34=0;

	Wheel vd;
	vd.l_v = 0.8;
	vd.a_v = 0;

	/// initial control inputs ///
	Wheel MyVel_1;
	MyVel_1.l_v = 0;
	MyVel_1.a_v = 0;

	Wheel MyVel_2;
	MyVel_2.l_v = 0;
	MyVel_2.a_v = 0;

	Wheel MyVel_3;
	MyVel_3.l_v = 0;
	MyVel_3.a_v = 0;

	Wheel MyVel_4;
	MyVel_4.l_v = 0;
	MyVel_4.a_v = 0;
	/////////////////////////////

	control_formation r1;
	mobileRobotSystem TestSys1;
	control_formation r2;
	mobileRobotSystem TestSys2;
	control_formation r3;
	mobileRobotSystem TestSys3;
	control_formation r4;
	mobileRobotSystem TestSys4;
	double dt = 0.1;

	MatrixXd A(4,4); // adjacency matrix
	A <<0,1,1,1,
		1,0,1,1,
		1,1,0,1,
		1,1,1,0;


	pose2D virtualGoal_1; pose2D virtualGoal_2; pose2D virtualGoal_3; pose2D virtualGoal_4;// virtual goal
	
	point2D po1; point2D po2; point2D po3;  point2D po4; // position of obstacle
	
	for(int i=0;i<250;i++) 
	{
		
		
		cout << "iteration i = " << i <<endl;
		/// virtual poses    
		q10_v.x = q10.x + q1off.x; q10_v.y = q10.y + q1off.y; q10_v.h = q10.h + q1off.h;
		q20_v.x = q20.x + q2off.x; q20_v.y = q20.y + q2off.y; q20_v.h = q20.h + q2off.h;
		q30_v.x = q30.x + q3off.x; q30_v.y = q30.y + q3off.y; q30_v.h = q30.h + q3off.h;
		q40_v.x = q40.x + q4off.x; q40_v.y = q40.y + q4off.y; q40_v.h = q40.h + q4off.h;
		//q50_v.x = q50.x + q5off.x; q50_v.y = q50.y + q5off.y; q50_v.h = q50.h + q5off.h;
		///
		/// virtual goals ///
		/// virtual goals ///
		virtualGoal_1.x = (q20_v.x + q30_v.x + q40_v.x)/3;// + q50_v.x;
		virtualGoal_1.y = (q20_v.y + q30_v.y + q40_v.y)/3;// + q50_v.y;
		virtualGoal_1.h = (q20_v.h + q30_v.h + q40_v.h)/3;// + q50_v.h;

		virtualGoal_2.x = (q10_v.x + q30_v.x + q40_v.x)/3;// + q50_v.x;
		virtualGoal_2.y = (q10_v.y + q30_v.y + q40_v.y)/3;// + q50_v.y;
		virtualGoal_2.h = (q10_v.h + q30_v.h + q40_v.h)/3;// + q50_v.h;

		virtualGoal_3.x = (q20_v.x + q10_v.x + q40_v.x)/3;// + q50_v.x;
		virtualGoal_3.y = (q20_v.y + q10_v.y + q40_v.y)/3;// + q50_v.y;
		virtualGoal_3.h = (q20_v.h + q10_v.h + q40_v.h)/3;// + q50_v.h;

		virtualGoal_4.x = (q20_v.x + q30_v.x + q10_v.x)/3; //+ q50_v.x;
		virtualGoal_4.y = (q20_v.y + q30_v.y + q10_v.y)/3;//+ q50_v.y;
		virtualGoal_4.h = (q20_v.h + q30_v.h + q10_v.h)/3;// + q50_v.h;
		//////////////////////////////////
	
		//p12(0) = q10.x - q20.x; p12(1) = q10.y - q20.y;
		////p13(0) = q10.x - q30.x; p13(1) = q10.y - q30.y;
		//p23(0) = q20.x - q30.x; p23(1) = q20.y - q30.y;


		p12(0) = q10.x - q20.x; p12(1) = q10.y - q20.y;
		p13(0) = q10.x - q30.x; p13(1) = q10.y - q30.y;
		p14(0) = q10.x - q40.x; p14(1) = q10.y - q40.y;
		//p15(0) = q10.x - q50.x; p15(1) = q10.y - q50.y;
		
		p23(0) = q20.x - q30.x; p23(1) = q20.y - q30.y;
		p24(0) = q20.x - q40.x; p24(1) = q20.y - q40.y;
		//p25(0) = q20.x - q50.x; p25(1) = q20.y - q50.y;

		p34(0) = q30.x - q40.x; p34(1) = q30.y - q40.y;

		/// Distance between robots ///
		d12 = p12.norm();
		d13 = p13.norm();
		d14 = p14.norm();

		d23 = p23.norm();
		d24 = p24.norm();
		//d25 = p25.norm();

		d34 = p34.norm();

		cout << "e12 = " << d12 - df12 << endl;
		cout << "e13 = " << d13 - df13 << endl;
		cout << "e23 = " << d23 - df23 << endl;
 

		double d1[3] = {d12,d13,d14};
		cout <<"d1 "<<d1<<endl;
 		int nneighbor = std::min_element(d1,d1+3) - d1;
		cout <<"std::min_element(d1,d1+3) "<<std::min_element(d1,d1+3)-d1<<endl;
 		switch (nneighbor) {
 			case 0:
 				po1.x = q20.x;
				po1.y = q20.y;
 				break;
 			case 1:
 				po1.x = q30.x;
				po1.y = q30.y;
 				break;
 			case 2:
 				po1.x = q40.x;
				po1.y = q40.y;
 				break;
 			// case 4:
 			// 	po1.x = q50.x;
				// po1.y = q50.y;
 			// 	break;
 		}

 		double d2[3] = {d12,d23,d24};
 		nneighbor = std::min_element(d2,d2+3) - d2;

 		switch (nneighbor) {
 			case 0:
 				po2.x = q10.x;
				po2.y = q10.y;
 				break;
 			case 1:
 				po2.x = q30.x;
				po2.y = q30.y;
 				break;
 			case 2:
 				po2.x = q40.x;
				po2.y = q40.y;
 				break;
 			// case 4:
 			// 	po2.x = q50.x;
				// po2.y = q50.y;
 			// 	break;
 		}

 		double d3[3] = {d13,d23,d34};
 		nneighbor = std::min_element(d3,d3+3) - d3;

 		switch (nneighbor) {
 			case 0:
 				po3.x = q10.x;
				po3.y = q10.y;
 				break;
 			case 1:
 				po3.x = q20.x;
				po3.y = q20.y;
 				break;
 			case 2:
 				po3.x = q40.x;
				po3.y = q40.y;
 				break;
 			// case 4:
 			// 	po3.x = q50.x;
				// po3.y = q50.y;
 			// 	break;
 		}
		double d4[3] = {d14,d24,d34};
 		nneighbor = std::min_element(d4,d4+3) - d4;

 		switch (nneighbor) {
 			case 0:
 				po4.x = q10.x;
				po4.y = q10.y;
 				break;
 			case 1:
 				po4.x = q20.x;
				po4.y = q20.y;
 				break;
 			case 2:
 				po4.x = q30.x;
				po4.y = q30.y;
 				break;
 			// case 4:
 			// 	po4.x = q50.x;
				// po4.y = q50.y;
 			// 	break;
 		}
		

		MyVel_1 = r1.switching_w_obstacle(q10_v, virtualGoal_1, q10, po1, vd);
		q10 = TestSys1.DifftypeSys(dt, q10, MyVel_1);
		q10.h = r1.between2PI(q10.h);

		MyVel_2 = r2.switching_w_obstacle(q20_v, virtualGoal_2, q20, po2, vd);
		q20 = TestSys2.DifftypeSys(dt, q20, MyVel_2);
		q20.h = r2.between2PI(q20.h);

		MyVel_3 = r3.switching_w_obstacle(q30_v, virtualGoal_3, q30, po3, vd);
		q30 = TestSys3.DifftypeSys(dt, q30, MyVel_3);
		q30.h = r3.between2PI(q30.h);

		MyVel_4 = r4.switching_w_obstacle(q40_v, virtualGoal_4, q40, po4, vd);
		q40 = TestSys4.DifftypeSys(dt, q40, MyVel_4);
		q40.h = r4.between2PI(q40.h);

		if ((MyVel_1.l_v-MyVel_2.l_v)<=0.008 && (MyVel_1.l_v-MyVel_3.l_v)<=0.008 && (MyVel_1.l_v-MyVel_4.l_v)<=0.008)
		{
			vd.l_v = vd.l_v - 0.01;
		}

		if (vd.l_v<=0)
		{
			vd.l_v = 0;
		}


		cout<<"Current State x, y, theta"<<endl;
 		cout<<q10.x<<setw(20)<<q10.y<<setw(20)<<q10.h<<endl;
		cout<<q20.x<<setw(20)<<q20.y<<setw(20)<<q20.h<<endl;
		cout<<q30.x<<setw(20)<<q30.y<<setw(20)<<q30.h<<endl;
		cout<<q40.x<<setw(20)<<q40.y<<setw(20)<<q40.h<<endl;
 		cout<<endl;
		/*
		cout<<"Current velocities" <<endl;
		cout<<MyVel_1.l_v<<setw(20)<<endl;
		cout<<MyVel_2.l_v<<setw(20)<<endl;
		cout<<MyVel_3.l_v<<setw(20)<<endl;
 		cout<<endl;
		*/

	}
	return 0;
}

