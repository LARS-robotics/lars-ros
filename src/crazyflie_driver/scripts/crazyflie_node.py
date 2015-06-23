#!/usr/bin/env python

import roslib
roslib.load_manifest('crazyflie_driver')
import rospy
import sys

from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from crazyflie_driver.msg import RPYT

import cflib.crtp

from cflib.crazyflie import Crazyflie
from cfclient.utils.logconfigreader import LogConfig
from cfclient.utils.logconfigreader import LogVariable


class CrazyflieNode(object):
	roll_desired = 0.0
	pitch_desired = 0.0
	yaw_desired = 0.0
	thrust_desired = 0.0
	roll_actual = 0.0
	pitch_actual = 0.0
	yaw_actual = 0.0
	TIMEOUT = rospy.Duration.from_sec(0.5)

	def __init__(self, default_radio='radio://0/15/2M',  default_update_rate=100.0):
	
		self.default_radio = default_radio
		self.default_update_rate = default_update_rate
		
		self.crazyflie = Crazyflie()
		cflib.crtp.init_drivers()
		
		rospy.init_node('crazyflie_driver')
		self._init_params()
		self._init_pubsub()
		
		self.crazyflie.connectionFailed.add_callback(self.connectionFailed)
		self.crazyflie.connectSetupFinished.add_callback(self.connectSetupFinished)
		
		self.opened = False
		
		self.last_command_time = rospy.get_rostime()
		
	def start(self):
		rospy.loginfo("Starting connection")
		self.crazyflie.open_link(self.radio)
	
	def connectionFailed(self, linkURI, exception_text):
		rospy.logerr("Failed to connect to radio: %s" %(self.radio))
		if not rospy.is_shutdown():
			rospy.sleep(0.5)
			self.crazyflie.open_link(self.radio)
		
	def isOpened(self):
		return self.opened
		
	def connectSetupFinished(self, linkURI):
		rospy.loginfo("connection established with radio: %s" %(self.radio))
		self.opened = True
		self._init_logs()
		
	def _init_logs(self):
		accel_log_conf = LogConfig("Accel", 10)
		accel_log_conf.addVariable(LogVariable("acc.x", "float"))
		accel_log_conf.addVariable(LogVariable("acc.y", "float"))
		accel_log_conf.addVariable(LogVariable("acc.z", "float"))
		
		self.accel_log = self.crazyflie.log.create_log_packet(accel_log_conf)
		
		if self.accel_log is not None:
			self.accel_log.dataReceived.add_callback(self.log_accel_data)
			self.accel_log.start()
		else:
			print("acc.x/y/z not found in log TOC")	
			
	def log_accel_data(self, data):
		if not rospy.is_shutdown():
			rospy.loginfo("Accelerometer: x=%.2f, y=%.2f, z=%.2f" %
							(data["acc.x"], data["acc.y"], data["acc.z"]))
			acc_msg = Vector3()
			acc_msg.x = data["acc.x"]
			acc_msg.y = data["acc.y"]
			acc_msg.z = data["acc.z"]
			
			self.accelerometer_pub.publish(acc_msg)	
		
	def _init_params(self):
		self.radio = rospy.get_param('~radio', self.default_radio)
		self.update_rate = rospy.get_param('~update_rate', self.default_update_rate) 
		rospy.loginfo("radio: %s"%(self.radio))
	
	def _init_pubsub(self):
		self.rotation_desired_sub = rospy.Subscriber('rotation_desired', RPYT, self.set_rotation_desired)
		self.rotation_actual_sub = rospy.Subscriber('rotation_actual', Vector3, self.set_rotation_actual)
		self.accelerometer_pub = rospy.Publisher('accelerometer', Vector3)
		
		#testing code
		self.vicon_sub = rospy.Subscriber('odom', Odometry, self.set_vicon)
	
	def set_rotation_desired(self, msg):
		self.roll_desired = msg.roll
		self.pitch_desired = msg.pitch
		self.yaw_desired = msg.yaw # yaw is a rate
		
		if msg.thrust > 100.0:
			msg.thrust = 100.0
		elif msg.thrust < 0.0:
			msg.thrust = 0.0
			
		self.thrust_desired = msg.thrust*55365*0.01+10000 # thrust is a percentage (0.0 - 100.0)
		
		self.last_command_time = rospy.get_rostime()
		
	def set_rotation_actual(self, msg):
		self.roll_actual = msg.x
		self.pitch_actual = msg.y
		self.yaw_actual = msg.z
		
	def set_vicon(self, msg):
		self.z = msg.pose.pose.position.z;

	def spin(self):
		rospy.loginfo("Spinning")
		r = rospy.Rate(self.update_rate)
		#turnOff = False;
		while not rospy.is_shutdown():
			#if self.z > 1700.0 or self.z == -20.0:
			#	turnOff = True
			if (rospy.get_rostime() - self.last_command_time) > self.TIMEOUT:# or turnOff:
				self.roll_desired = 0.0 
				self.pitch_desired = 0.0
				self.yaw_desired = 0.0
				self.thrust_desired = 0.0
			
			#self.crazyflie.commander.send_setpoint(self.roll_desired, self.pitch_desired, self.yaw_desired, self.thrust_desired, self.roll_actual, self.pitch_actual, self.yaw_actual)
			self.crazyflie.commander.send_setpoint(self.roll_desired, self.pitch_desired, self.yaw_desired, self.thrust_desired)
			
			r.sleep()
		
		rospy.loginfo("Cleaning up")
		self.crazyflie.close_link()
	
	
def crazyflie_main(argv):
	c = CrazyflieNode()
	if not rospy.is_shutdown():
		c.start()
		while not c.isOpened() and not rospy.is_shutdown():
			rospy.sleep(0.25)
		c.spin()

if __name__ == '__main__':
	crazyflie_main(sys.argv) 

	
	
	
	
	
	
		
