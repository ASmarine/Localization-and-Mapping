#!/usr/bin/env python

#simplePingExample.py
import rospy
import time
import argparse
from brping import Ping1D
from sensor_msgs.msg import LaserScan
from builtins import input

pi = 3.14159265359

class SonarPinger(object):
	def __init__(self):

		# Sonar Data Publisher	

		self.pub = rospy.Publisher('/Sensor', LaserScan, queue_size=10)
		rospy.init_node('sonar_interface', anonymous=True)
		self.pinger=self.init_sonar()
		while not rospy.is_shutdown():
			msg1 = self.sonar_data()
			rospy.loginfo(msg1)
			self.pub.publish(msg1)
		    	time.sleep(0.1)

	def sonar_data(self):
		sonar = LaserScan()
		measurements = []
		for i in range(5):
			measurements.append(self.pinger.get_distance()['distance'])
			time.sleep(0.1)
		sonar.angle_min=0 #angle in rad
		sonar.angle_max= pi #angle in rad
		sonar.angle_increment= pi/180 
		sonar.range_min=0.5 #distance in meters
		sonar.range_max=30 #distance in meters
		sonar.ranges= measurements
		return sonar

	def init_sonar(self):
		#Parse Command line options
		device="/dev/ttyTHS2"
		baudrate=9600		
		#Make a new Ping
		myPing = Ping1D(device, baudrate)
		return myPing
		print("Pinger initialized!")
		if myPing.initialize() is False:
    			print("Failed to initialize Pinger!")
			return None

if __name__ == '__main__':
    try:
        SonarPinger()
    except rospy.ROSInterruptException:
        pass
