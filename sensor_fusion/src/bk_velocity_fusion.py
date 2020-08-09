#!/usr/bin/env python

import rospy
#from std_msgs.msg import String
#from std_msgs.msg import Float32
from sensor_msgs.msg import Imu 
from std_msgs.msg import ByteMultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from filterpy.kalman import KalmanFilter
import numpy as np

class VelocityFusion(object):
	def __init__(self):

		#subscribe to pixhawk acceleration data 
		rospy.Subscriber('/mavros/imu/data', Imu , self.kalman)
		self.twist_pub = rospy.Publisher('/fused_twist', Twist, queue_size=1) # For testing purposes	
		#self.twiststamped_pub = rospy.Publisher('/fused_twiststamped', TwistStamped, queue_size=1)		
		# state variables matrix dim is dim_x
		# measurement/observation matrix dim is dim_z 
		self.f = KalmanFilter (dim_x=6, dim_z=3)
		#Time stamp
		self.dt = 1/4.0
		#Standard Deviation
		self.sigmax= 0.08
		self.sigmay= 0.08
		self.sigmaz= 0.08

		# state variables initialization
		self.f.x = np.array( [ 0,		#vx
				  0,		#vy
				  0,		#vz
				  0,			#ax
				  0, 			#ay 
                  0])    		#az

		#PREDICTION STEP MATRICES 

		#State transition matrix F 6x6 (sometimes called A)
		self.f.F = np.array([[1,0,0,self.dt,0,0],
		                [0,1,0,0,self.dt,0],
		                [0,0,1,0,0,self.dt],
		                [0,0,0,1,0,0],
		                [0,0,0,0,1,0],
		                [0,0,0,0,0,1]])
		
		#Process Noise W 6x1
		
		#6x3
		self.G = np.array([[self.dt,0,0],
					 [0,self.dt,0],
					 [0,0,self.dt],
					 [1,0,0],
					 [0,1,0],
					 [0,0,1]])


		#Sensor Covariance Matrix (also called Expected value of a.aT (E(aaT))) 3x3
		#R= np.array([[sigmax**2, 0, 0], [0, sigmay**2,0], [0, 0, sigmaz**2]])
		#R = np.array([[8.999999999999999e-08, 0.0, 0.0],[ 0.0, 8.999999999999999e-08, 0.0], [0.0, 0.0, 8.999999999999999e-08]])
		#R = np.array([[1000, 0.0, 0.0],[ 0.0, 1000, 0.0], [0.0, 0.0, 1000]])
		#Process Noise Covariance Matrix Q 6x6
		#Q = np.dot(np.dot (self.G , R), self.G.T)

		self.f.Q = np.array([ [0.001,0,0,0,0,0],
			             [0,0.001,0,0,0,0],
			             [0,0,0.001,0,0,0],
			             [0,0,0,0.001,0,0],
			             [0,0,0,0,0.001,0],
			             [0,0,0,0,0,0.001]])

		#State Covariance Matrix P 6x6 INITIALIZATION
		self.f.P = np.array([ [1000,0,0,0,0,0],
			             [0,1000,0,0,0,0],
			             [0,0,1000,0,0,0],
			             [0,0,0,1000,0,0],
			             [0,0,0,0,1000,0],
			             [0,0,0,0,0,1000]])
		
		
		#MEASUREMENT STEP MATRICES
		
		#the measurement function H 3x6 (sometimes called C)
		self.f.H = np.array([[0, 0 ,0 ,self.dt ,0  , 0 ],
						[0, 0 ,0 ,0  ,self.dt ,0  ],
						[0, 0 ,0 ,0  ,0  ,self.dt]])

		#Measurement Noise (Uncertainity) Z 3x1
		#We will assume it's a gaussian white noise where Z~N(0,R) 
		#Z =  np.random.normal(0, 4  , size=3) * 0.04
		self.Z=0 


		#subscribe to imu acceleration data 
		#sub2 = rospy.Subscriber('/mavros/imu/data', Float32, self.go_to_depth)

		rospy.init_node('velocity_fusion')
		rospy.spin()



	def kalman (self, msg):
		#print(msg.linear_acceleration)
		#print("received")
		a_reading = msg.linear_acceleration
		a_reading.y +=9.9
		print(a_reading)
		print("****************")
		#a = [a_reading.x, a_reading.y, a_reading.z]
		#W = np.dot (self.G, a)
		#self.f.W = W 
		self.f.predict()
		Y= np.dot(self.f.H, np.array([0,0,0, a_reading.x, a_reading.y, a_reading.z])) + self.Z
		self.f.update(Y)

		testData = Twist()
		#stampedData = TwistStamped()

		#linear velocity
		testData.linear.x = self.f.x[0]
		testData.linear.y = self.f.x[1]		
		testData.linear.z = self.f.x[2]
		
		#angular velocity
		testData.angular.x = self.f.x[3]
		testData.angular.y = self.f.x[4]		
		testData.angular.z = self.f.x[5]
		
		self.twist_pub.publish(testData) # For testing purposes

		#linear stamped velocity		
		#stampedData.twist.linear.x = self.f.x[0]
		#stampedData.twist.linear.y = self.f.x[1]		
		#stampedData.twist.linear.z = self.f.x[2]
		#self.twiststamped_pub.publish(stampedData)
		print(testData)
		print("-------------------------------")


if __name__ == '__main__':
	try:
		VelocityFusion()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start velocity_fusion node.')
