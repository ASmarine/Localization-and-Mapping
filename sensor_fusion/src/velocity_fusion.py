#!/usr/bin/env python

import rospy
import numpy as np
from math import sin, cos, tan
from sensor_msgs.msg import Imu 
from mavros_msgs.msg import VFR_HUD
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import ByteMultiArray
from filterpy.kalman import KalmanFilter
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class VelocityFusion(object):
	def __init__(self):

		rospy.init_node('velocity_fusion')
		
		self.counter = 0 

		self.linear_acc_x = 0
		self.avg_linear_acc_x = 0
		
		self.linear_acc_y = 0
		self.avg_linear_acc_y = 0
		
		self.linear_acc_z = 0
		self.avg_linear_acc_z = 0 

		self.angular_vel_x = 0
		self.avg_angular_vel_x = 0
		
		self.angular_vel_y = 0
		self.avg_angular_vel_y = 0
		
		self.angular_vel_z = 0
		self.avg_angular_vel_z = 0

		self.orientation_x = 0
		self.avg_orientation_x = 0
		
		self.orientation_y = 0
		self.avg_orientation_y = 0
		
		self.orientation_z = 0
		self.avg_orientation_z = 0
		
		self.orientation_w = 0
		self.avg_orientation_w = 0
		
		self.depth_measurement = 0
		
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		
		self.linear_vel_x = 0
		self.linear_vel_y = 0
		self.linear_vel_z  = 0
		
		self.prev_msg = None

		#subscribe to pixhawk acceleration data 
		rospy.Subscriber('/mavros/imu/data', Imu , self.update_cb)
		rospy.Subscriber('/mavros/vfr_hud', VFR_HUD, self.pressure_cb)
		rospy.Subscriber('/motion_model/A', numpy_msg(Floats), self.motion_model_A_cb)
		rospy.Subscriber('/motion_model/B', numpy_msg(Floats), self.motion_model_B_cb)
		rospy.Subscriber('/motion_model/thrusts', numpy_msg(Floats), self.thrusts_cb)

		self.vel_pub = rospy.Publisher('/fused_velocity', ByteMultiArray, queue_size=1)

		# state variables matrix dim is dim_x
		# measurement/observation matrix dim is dim_z 
		self.f = KalmanFilter (dim_x=10, dim_z=10)

		#Timestamp
		self.dt = 1/4.0

		#Standard Deviation
		self.sigmax= 0.08
		self.sigmay= 0.08
		self.sigmaz= 0.08

		# state variables initialization
		self.f.x = np.array( [ 
				  0,
				  0,
				  0,
				  0,
				  0,		#vx
				  0,		#vy
				  0,		#vz
				  0,		#ax
				  0, 		#ay 
                  0])    	#az

		#PREDICTION STEP MATRICES 

		#State transition matrix F 6x6 (sometimes called A)
		self.f.F = np.array([[0,0,0,0,0,0,0,0,0,0],
							[0,0,0,0,0,0,0,0,0,0],
							[0,0,0,0,0,0,0,0,0,0],
							[0,0,0,0,0,0,0,0,0,0],
							[0,0,0,0,0,0,0,0,0,0],
							[0,0,0,0,0,0,0,0,0,0],
							[0,0,0,0,0,0,0,0,0,0],
							[0,0,0,0,0,0,0,0,0,0],
							[0,0,0,0,0,0,0,0,0,0],
							[0,0,0,0,0,0,0,0,0,0]])

		#State transition matrix B
		self.f.B = np.array([[0,0,0,0,0,0,0,0],
							[0,0,0,0,0,0,0,0],
							[0,0,0,0,0,0,0,0],
							[0,0,0,0,0,0,0,0],
							[0,0,0,0,0,0,0,0],
							[0,0,0,0,0,0,0,0],
							[0,0,0,0,0,0,0,0],
							[0,0,0,0,0,0,0,0],
							[0,0,0,0,0,0,0,0],
							[0,0,0,0,0,0,0,0]])
		#Process Noise W 6x1
		#6x3
		# self.G = np.array([[self.dt,0,0],
		# 			 [0,self.dt,0],
		# 			 [0,0,self.dt],
		# 			 [1,0,0],
		# 			 [0,1,0],
		# 			 [0,0,1]])


		#Sensor Covariance Matrix (also called Expected value of a.aT (E(aaT))) 3x3
		# R = np.array([[1000, 0.0, 0.0],[ 0.0, 1000, 0.0], [0.0, 0.0, 1000]])


		#Process Noise Covariance Matrix Q 6x6
		# self.f.Q = np.array([ [0.001,0,0,0,0,0],
		# 	             [0,0.001,0,0,0,0],
		# 	             [0,0,0.001,0,0,0],
		# 	             [0,0,0,0.001,0,0],
		# 	             [0,0,0,0,0.001,0],
		# 	             [0,0,0,0,0,0.001]])

		#State Covariance Matrix P 6x6 INITIALIZATION
		self.f.P = np.array([ [1000,0,0,0,0,0,0,0,0,0],
			             [0,1000,0,0,0,0,0,0,0,0],
			             [0,0,1000,0,0,0,0,0,0,0],
			             [0,0,0,1000,0,0,0,0,0,0],
			             [0,0,0,0,1000,0,0,0,0,0],
			             [0,0,0,0,0,1000,0,0,0,0],
			             [0,0,0,0,0,0,1000,0,0,0],
			             [0,0,0,0,0,0,0,1000,0,0],
			             [0,0,0,0,0,0,0,0,1000,0],
			             [0,0,0,0,0,0,0,0,0,1000]])
		
		#MEASUREMENT STEP MATRICES
		
		#the measurement function H 3x6 (sometimes called C)
		self.f.H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
							 [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
							 [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
							 [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
							 [0, 0, 0, 0, self.dt, 0, 0, 0, 0, 0],
							 [0, 0, 0, 0, 0, self.dt, 0, 0, 0, 0],
							 [0, 0, 0, 0, 0, 0, self.dt, 0, 0, 0],
							 [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
							 [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
							 [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])

		#Measurement Noise (Uncertainity) Z 3x1
		#We will assume it's a gaussian white noise where Z~N(0,R) 
		#Z =  np.random.normal(0, 4  , size=3) * 0.04
		self.Z = 0 

		#subscribe to imu acceleration data 
		#sub2 = rospy.Subscriber('/mavros/imu/data', Float32, self.go_to_depth)

		rate = rospy.Rate(1) # 10hz
		while not rospy.is_shutdown():
			self.kalman()
			rate.sleep()

	def update_cb(self, msg):
		if (self.counter < 100):
			self.avg_linear_acc_x += msg.linear_acceleration.x
			self.avg_linear_acc_y += msg.linear_acceleration.y
			self.avg_linear_acc_z += msg.linear_acceleration.z

			self.avg_angular_vel_x += msg.angular_velocity.x
			self.avg_angular_vel_y += msg.angular_velocity.y
			self.avg_angular_vel_z += msg.angular_velocity.z

			self.avg_orientation_x += msg.orientation.x
			self.avg_orientation_y += msg.orientation.y
			self.avg_orientation_z += msg.orientation.z
			self.avg_orientation_w += msg.orientation.w

			self.counter += 1 
			return

		elif self.counter == 100:
			self.avg_linear_acc_x = self.avg_linear_acc_x/100.0
			self.avg_linear_acc_y = self.avg_linear_acc_y/100.0
			self.avg_linear_acc_z = self.avg_linear_acc_z/100.0

			self.avg_angular_vel_x = self.avg_angular_vel_x/100.0
			self.avg_angular_vel_y = self.avg_angular_vel_y/100.0
			self.avg_angular_vel_z = self.avg_angular_vel_z/100.0

			self.avg_orientation_x = self.avg_orientation_x/100.0
			self.avg_orientation_y = self.avg_orientation_y/100.0
			self.avg_orientation_z = self.avg_orientation_z/100.0
			self.avg_orientation_w = self.avg_orientation_w/100.0

		self.linear_acc_x = msg.linear_acceleration.x - self.avg_linear_acc_x
		self.linear_acc_y = msg.linear_acceleration.y - self.avg_linear_acc_y
		self.linear_acc_z = msg.linear_acceleration.z - self.avg_linear_acc_z

		# if self.prev_msg:
		# 	print(type(msg.header.stamp))
		# 	print(type(self.prev_msg.header.stamp))
		# 	timesample = msg.header.stamp - self.prev_msg.header.stamp
		# 	print(timesample.secs)
		# 	self.linear_vel_x += (self.linear_acc_x_prev * timesample.secs) + (0.5*(self.linear_acc_x - self.linear_acc_x_prev)*timesample.secs)
		# 	self.linear_vel_y += (self.linear_acc_y_prev * timesample.secs) + (0.5*(self.linear_acc_y - self.linear_acc_y_prev)*timesample.secs)
		# 	self.linear_vel_z += (self.linear_acc_z_prev * timesample.secs) + (0.5*(self.linear_acc_z - self.linear_acc_z_prev)*timesample.secs)

		self.prev_msg = msg
		self.linear_acc_x_prev = self.linear_acc_x
		self.linear_acc_y_prev = self.linear_acc_y
		self.linear_acc_z_prev = self.linear_acc_z

		self.angular_vel_x = msg.angular_velocity.z - self.avg_angular_vel_x
		self.angular_vel_y = msg.angular_velocity.z - self.avg_angular_vel_y
		self.angular_vel_z = msg.angular_velocity.z - self.avg_angular_vel_z

		self.orientation_x = msg.orientation.x - self.avg_orientation_x
		self.orientation_y = msg.orientation.y - self.avg_orientation_y
		self.orientation_z = msg.orientation.z - self.avg_orientation_z
		self.orientation_w = msg.orientation.w - self.avg_orientation_w

		self.roll, self.yaw, self.pitch = euler_from_quaternion([self.orientation_x, self.orientation_y, self.orientation_z, self.orientation_w])
		# print self.roll, self.yaw, self.pitch
		# print([a_x_reading, a_y_reading, a_z_reading])
		# print("****************")


	def pressure_cb(self, msg):
		self.depth_measurement = msg.altitude - 0.0828

	def thrusts_cb(self):
		pass

	def kalman (self):

		# self.f.predict()

		Y = np.dot(self.f.H, np.array([self.depth_measurement, self.roll,  self.pitch, self.yaw, self.linear_vel_x, self.linear_vel_y, self.linear_vel_z, self.angular_vel_x, self.angular_vel_y, self.angular_vel_z])) + self.Z
			
		jacobian = np.linalg.inv(np.array([
			[cos(self.yaw)*cos(self.pitch), (-sin(self.yaw)*cos(self.roll))+(cos(self.yaw)*sin(self.pitch)*sin(self.roll)), (sin(self.yaw)*sin(self.roll) )+(cos(self.yaw)*cos(self.roll)*sin(self.pitch)), 0, 0, 0],
			[sin(self.yaw)*cos(self.pitch), (cos(self.yaw)*cos(self.pitch))+(sin(self.roll)*sin(self.pitch)*sin(self.yaw)), (-cos(self.yaw)*sin(self.roll))+(sin(self.pitch)*sin(self.yaw)*cos(self.roll)), 0, 0, 0],
			[-sin(self.pitch), cos(self.pitch)*sin(self.roll), cos(self.pitch)*cos(self.roll), 0, 0, 0],
			[0, 0, 0, 1, sin(self.roll)*tan(self.pitch), cos(self.roll)*tan(self.pitch)],
			[0, 0, 0, 0, cos(self.roll), -sin(self.roll)],
			[0, 0, 0, 0, sin(self.roll)/cos(self.pitch), cos(self.roll)/cos(self.pitch)]]))
					
		mat = np.array([
			[1, 0, 0, 0, 0, 0, 0, 0 ,0 ,0],
			[0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
			[0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
			[0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
			[0, 0, 0, 0, jacobian[0][0], jacobian[0][1], jacobian[0][2], jacobian[0][3], jacobian[0][4], jacobian[0][5]],
			[0, 0, 0, 0, jacobian[1][0], jacobian[1][1], jacobian[1][2], jacobian[1][3], jacobian[1][4], jacobian[1][5]], 
			[0, 0, 0, 0, jacobian[2][0], jacobian[2][1], jacobian[2][2], jacobian[2][3], jacobian[2][4], jacobian[2][5]], 
			[0, 0, 0, 0, jacobian[3][0], jacobian[3][1], jacobian[3][2], jacobian[3][3], jacobian[3][4], jacobian[3][5]], 
			[0, 0, 0, 0, jacobian[4][0], jacobian[4][1], jacobian[4][2], jacobian[4][3], jacobian[4][4], jacobian[4][5]], 
			[0, 0, 0, 0, jacobian[5][0], jacobian[5][1], jacobian[5][2], jacobian[5][3], jacobian[5][4], jacobian[5][5]]])

		# print(mat.shape)
		
		# Y = np.dot(Y, mat)
		
		# print(Y.shape)

		print(Y)
		print(Y[4])
		print(Y[5])
		print(Y[6])

		self.f.update(Y)


		print 'Depth:', self.f.x[0]
		print 'Roll:', self.f.x[1]
		print 'Pitch:', self.f.x[2]
		print 'Yaw:', self.f.x[3]
		print 'Linear Velocity x:', self.f.x[4]
		print 'Linear Velocity y:', self.f.x[5]
		print 'Linear Velocity z:', self.f.x[6]
		print 'Angular Velocity x:', self.f.x[7]
		print 'Angular Velocity y:', self.f.x[8]
		print 'Angular Velocity z:', self.f.x[9]
		print '-------------------------------------------'
		#testData = ByteMultiArray()
		#testData.layout.dim = 3
		#testData.data=self.f.x
		#self.vel_pub.publish(testData) # For testing purposes
		#print(testData)

	def motion_model_A_cb(self, msg):
		self.f.F = msg

	def motion_model_B_cb(self, msg):
		self.f.B = msg

if __name__ == '__main__':
	try:
		VelocityFusion()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start velocity_fusion node.')
