#!/usr/bin/env python

import roslib
roslib.load_manifest('crazyflie_control')
import rospy
import sys

from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from crazyflie_driver.msg import RPYT

import dynamic_reconfigure.server
from crazyflie_control.cfg import CrazyflieControlConfig

from math import *

import numpy as np

class CrazyflieControlNode(object):
	mass = 1.0
	gravity = 9.801
	kpz = 1.0 
	kdz = 1.0
	kpx = 1.0 
	kpy = 1.0 
	kdx = 1.0 
	kdy = 1.0
	
	xd = 0.0
	yd = 0.0
	zd = 0.0
	
	xp = 0.0
	yp = 0.0
	zp = 0.0
	
	x = 0.0
	y = 0.0
	z = 0.0
	
	q0 = 1.0
	q1 = 0.0
	q2 = 0.0
	q3 = 0.0
	last_odometry_update = rospy.Time()
	
	def __init__(self, default_name='apollo', default_update_rate=100):
		self.default_name = default_name
		self.default_update_rate = default_update_rate
		
		rospy.init_node('crazyflie_control')
		self._init_params()
		self._init_pubsub()
		
		dynamic_reconfigure.server.Server(CrazyflieControlConfig, self.reconfigure)
		
		self.last_odometry_update = rospy.get_rostime()
		
	def _init_params(self):
		self.name = rospy.get_param('~name', self.default_name)
		self.update_rate = rospy.get_param('~update_rate', self.default_update_rate) 
	
	def _init_pubsub(self):
		self.vicon_sub = rospy.Subscriber('/' + self.name + '/odom', Odometry, self.set_odometry)
		self.rotation_desired_pub = rospy.Publisher('/' + self.name + '/rotation_desired', RPYT)
		self.rotation_actual_pub = rospy.Publisher('/' + self.name + '/rotation_actual', Vector3)
		
	def set_odometry(self, msg):
	
		now = rospy.get_rostime()
		dt = self.last_odometry_update - now
		
		x_old = self.x
		y_old = self.y
		z_old = self.z

		self.x = msg.pose.pose.position.x * 0.001
		self.y = msg.pose.pose.position.y * 0.001
		self.z = msg.pose.pose.position.z * 0.001
		self.q1 = msg.pose.pose.orientation.x
		self.q2 = msg.pose.pose.orientation.y
		self.q3 = msg.pose.pose.orientation.z
		self.q0 = msg.pose.pose.orientation.w
		
		self.xd = (2.0/dt.to_sec())*(self.x - x_old) - self.xd
		self.yd = (2.0/dt.to_sec())*(self.y - y_old) - self.yd
		self.zd = (2.0/dt.to_sec())*(self.z - z_old) - self.zd
	
		self.last_odometry_update = now
		
	def reconfigure(self, config, level):
		self.kpx = config['kpx']
		self.kpy = config['kpy']
		self.kpz = config['kpz']
		
		self.kdx = config['kdx']
		self.kdy = config['kdy']
		self.kdz = config['kdz']
		
		self.xd = config['xd']
		self.yd = config['yd']
		self.zd = config['zd']
		
		self.power = config['power']
		
		return config
		
	def spin(self):
		rospy.loginfo("Spinning")
		r = rospy.Rate(self.update_rate)
		
		while not rospy.is_shutdown():
			gx = 2 * (self.q1*self.q3 - self.q0*self.q2);
			gy = 2 * (self.q0*self.q1 + self.q2*self.q3);
			gz = self.q0*self.q0 - self.q1*self.q1 - self.q2*self.q2 + self.q3*self.q3;
			
			yaw = atan2(2*self.q1*self.q2 - 2*self.q0*self.q3, 2*self.q0*self.q0 + 2*self.q1*self.q1 - 1) * 180 /pi;
			pitch = atan(gx / sqrt(gy*gy + gz*gz)) * 180 / pi;
			roll = atan(gy / sqrt(gx*gx + gz*gz)) * 180 / pi;
			
			msg_actual = Vector3()
			
			msg_actual.x = roll
			msg_actual.y = pitch
			msg_actual.z = yaw
			
			self.rotation_actual_pub.publish(msg_actual)
			
			R = [ [0]*3 ]*3
			
			R[0][0] = pow(self.q0,2) + pow(self.q1,2) - pow(self.q2,2) - pow(self.q3,2)
			R[0][1] = 2*self.q0*self.q1 - 2*self.q0*self.q3
			R[0][2] = 2*self.q1*self.q3 + 2*self.q0*self.q2
			
			R[1][0] = 2*self.q0*self.q1 + 2*self.q0*self.q3
			R[1][1] = pow(self.q0,2) - pow(self.q1,2) + pow(self.q2,2) - pow(self.q3,2)
			R[1][2] = 2*self.q2*self.q3 - 2*self.q0*self.q1
			
			R[2][0] = 2*self.q1*self.q3 - 2*self.q0*self.q2
			R[2][1] = 2*self.q2*self.q3 + 2*self.q0*self.q1
			R[2][2] = pow(self.q0,2) - pow(self.q1,2) - pow(self.q2,2) + pow(self.q3,2)
			
			r_matrix = np.matrix(R)
			
			# This is the thrust, should be also placed in the function below...
			f = self.mass / R[2][2] * ( self.gravity - self.kpz*(self.z-self.zd) - self.kdz*self.zp ) 
			
			r13d = self.mass / f * ( -self.kpx*(self.x-self.xd) - self.kdx*self.xp )
			r23d = self.mass / f * ( -self.kpy*(self.y-self.yd) - self.kdy*self.yp )
			r33d = sqrt(1-pow(r13d,2)-pow(r23d,2))
			
			v = [0]*3
			v[0] = -r23d
			v[1] = r13d
			v[2] = 0.0
			
			angle = acos(r33d)
			ca = cos(angle)
			sa = sin(angle)

			A = [ [0]*3 ]*3

			A[0][0] = ca + pow(v[0],2)*(1-ca)
			A[0][1] = v[0]*v[1]*(1-ca) - v[2]*sa
			A[0][2] = v[0]*v[2]*(1-ca) + v[1]*sa
			A[1][0] = v[0]*v[1]*(1-ca) + v[2]*sa
			A[1][1] = ca + pow(v[1],2)*(1-ca)
			A[1][2] = v[1]*v[2]*(1-ca) - v[0]*sa
			A[2][0] = v[0]*v[2]*(1-ca) + v[1]*sa
			A[2][1] = v[1]*v[2]*(1-ca) + v[0]*sa
			A[2][2] = ca + pow(v[2],2)*(1-ca)
			
			a_matrix = np.matrix(A)

			rd = [0]*3
			
			rd[0] = r13d
			rd[1] = r23d
			rd[2] = r33d
			
			rd_matrix = np.matrix(rd)

			gd = np.transpose(r_matrix)*a_matrix*np.transpose(rd_matrix)

			eulerRollDesired  = atan2(gd[1],sqrt(pow(gd[1],2)+pow(gd[2],2))) * 180 / pi
			eulerPitchDesired = -atan(gd[0]/sqrt(pow(gd[1],2)+pow(gd[2],2))) * 180 / pi
			eulerYawDesired   = 0.0;
			
			msg_desired = RPYT()
			msg_desired.roll = eulerRollDesired
			msg_desired.pitch = eulerPitchDesired
			msg_desired.yaw = eulerYawDesired
			
			if self.power:
				msg_desired.thrust = f
				
			else:
				msg_desired.thrust = 0.0
				
			
			self.rotation_desired_pub.publish(msg_desired)
		
			r.sleep()
	
def crazyflie_control_main(argv):
	c = CrazyflieControlNode()
	c.spin()

if __name__ == '__main__':
	crazyflie_control_main(sys.argv) 
