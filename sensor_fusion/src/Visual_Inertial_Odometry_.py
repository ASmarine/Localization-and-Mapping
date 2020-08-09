#!/usr/bin/env python

import rospy
import tf.transformations
import math
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import datetime

class VisualInertialOdometry():

	def __init__(self):

		self.filtered_odom_pub = rospy.Publisher("/odometry/filtered/self",Odometry,queue_size=10)

		self.transform_broadcaster = tf.TransformBroadcaster()
		
		self.odom = Odometry()
		self.imu = Imu()

		self.vYawViso=0.0
		self.vX=0.0

		self.yawViso=0.0
		self.yawImu=0.0
		self.yawImu1=0.0


		self.roll=0.0
		self.pitch=0.0

		self.yawCov=10
		self.vYawCov=0.09
		self.ImuYawCov=0.15707963267948966

		self.first_time = False

		self.cmd = np.array ([(0.0,0.0),(0.0,0.0)])
		self.yaw = np.array ([0.0,0.0])
		self.meas = np.array ([(0.0,0.0),(0.0,0.0)])

		self.inp = np.array ([0.0,0.0])
		self.odomEKF = np.array ([0.0,0.0,0.0])
		self.pred = np.array ([0.0,0.0,0.0])
		
		self.frequency = 5.0 # 5Hz
		self.loop_duration = 1/self.frequency

		self.prev_timestamp = None
		self.prev_x = None
		self.first_odom = True

		rospy.init_node('visual_inertial_odometry')

		rospy.Subscriber("/zed/zed_node/odom", Odometry, self.odom_callback)
		rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
		rospy.Subscriber("/fused_twist", Twist, self.twist_callback) # Velocity Subscriber
		#rospy.Subscriber("/fused_velocity", TwistStamped, self.twist_stamped_callback) # Stamped Velocity Subscriber

		rospy.Timer(rospy.Duration(self.loop_duration), self.loop)
		rospy.spin()

	def odom_callback(self, msg): 
   
		self.odom = msg

		x = self.odom.pose.pose.position.x
		timestamp = self.odom.header.stamp.secs

		quaternion = (
			msg.pose.pose.orientation.x,
			msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z,
			msg.pose.pose.orientation.w
		)

		euler = tf.transformations.euler_from_quaternion(quaternion)

		self.roll = euler[0]
		self.pitch = euler[1]
		self.yawViso = euler[2]

		if self.first_odom:
			self.prev_x = x
			self.prev_yawViso = self.yawViso 
			self.prev_timestamp = timestamp
			self.first_odom = False
		
		time_diff = timestamp - self.prev_timestamp

		if time_diff < 0:
			self.first_odom = True
			return

		if (time_diff >= 0.5):
			self.vX = (x - self.prev_x) / time_diff
			self.vYawViso = (self.yawViso - self.prev_yawViso) / time_diff
			self.prev_x = x
			self.prev_yawViso = self.yawViso
			self.prev_timestamp = timestamp    


	def imu_callback(self, msg):

		self.imu = msg

		quaternion = (
			msg.orientation.x,
			msg.orientation.y,
			msg.orientation.z,
			msg.orientation.w
		)

		euler = tf.transformations.euler_from_quaternion(quaternion)
		
		self.roll = euler[0]
		self.pitch = euler[1]
		self.yawImu = euler[2]
	    

	def twist_callback(self, msg):
		self.cmd[1][0] = msg.linear.x
		#self.cmd[2][0] = msg.twist.linear.y
		#self.cmd[3][0] = msg.twist.linear.z
		#self.cmd[1][1] = msg.twist.angular.x
		#self.cmd[1][2] = msg.twist.angular.y
		self.cmd[1][1] = msg.angular.z

	def stamped_twist_callback(self, msg):
		self.cmd[1][0] = msg.linear.x
		self.cmd[1][1] = msg.angular.z

	def send(self):
		odomOut = Odometry()

		odomOut.header.stamp=rospy.Time.now()

		odomOut.header.frame_id = "map"
		odomOut.child_frame_id = "base_link"

		odomOut.pose.pose.position.x = self.odomEKF[0]
		odomOut.pose.pose.position.y = self.odomEKF[1]
		odomOut.pose.pose.position.z = self.odomEKF[2]

		quaternion = tf.transformations.quaternion_from_euler(self.odomEKF[0], self.odomEKF[1], self.odomEKF[2])

		odomOut.pose.pose.orientation.x = quaternion[0]
		odomOut.pose.pose.orientation.y = quaternion[1]
		odomOut.pose.pose.orientation.z = quaternion[2]
		odomOut.pose.pose.orientation.w = quaternion[3]
	    
		self.transform_broadcaster.sendTransform(
			(self.odomEKF[0], self.odomEKF[1], 0),
			quaternion,
			rospy.Time.now(),
			"base_link",
			"odom"
		)
	    
		self.filtered_odom_pub.publish(odomOut)


	def loop(self, event):
		#print 'Timer called at ' + str(event.current_real)

		if self.first_time:
			self.yawImu1=self.yawImu
			self.first_time = False
	    
		#linear KF
		predYaw=self.yaw[0]+self.vYawViso*self.loop_duration
		#print('predYaw=', predYaw)

		predCov=self.yawCov + self.vYawCov
		#print('predCov=', predCov)

		inn=(self.yawImu-self.yawImu1)-predYaw
		#print('inn=', inn)

		innCov=predCov+self.ImuYawCov
		#print('innCov=', innCov)

		K=predCov*(1/innCov)
		#print ('k=',K)

		self.yaw[1]=predYaw+K*inn
		#print ('yaw[1]',self.yaw[1])

		self.yawCov=(1-K)*predCov
		#print ('yawCov=',self.yawCov)

		vYaw=(self.yaw[1]-self.yaw[0])*self.frequency
		#print ('vYaw=',vYaw)

		self.yaw[0]=self.yaw[1]
		#print ('yaw=',self.yaw[1])

		self.meas[1][0]=self.vX
		self.meas[1][1]=vYaw
		#print ('meas[1][0]=',self.meas[1][0])
		#print ('meas[1][1]=',self.meas[1][1])

		if self.meas[1][0]>1.0:
			self.meas[1][0]=self.meas[0][0]
			self.meas[1][1]=self.meas[0][1]

		#print ('meas[1][0]=',self.meas[1][0])
		#print ('meas[1][1]=',self.meas[1][1])

		self.meas[0][0]=self.meas[1][0]
		self.meas[0][1]=self.meas[1][1]

		vConCov=0.001
		wConCov=0.01

		vMeasCov=0.002
		wMeasCov=self.yawCov*self.frequency

		self.inp[0]=(((vConCov)**2)*self.meas[1][0]+((vMeasCov)**2)*self.cmd[0][0])/(((vConCov)**2)+((vMeasCov)**2))
		self.inp[1]=(((wConCov)**2)*self.meas[1][1]+((wMeasCov)**2)*self.cmd[0][1])/(((wConCov)**2)+((wMeasCov)**2))

		#print(self.inp[0], self.inp[1])
		print(self.cmd[0][0], self.cmd[0][1])

		if self.cmd[0][0]>0.000001 or self.cmd[0][1]>0.000001:
		
			if self.cmd[0][1]>0.000001:
				self.pred[0]=-(self.inp[0]/self.inp[1])*np.sin(self.odomEKF[2])+(self.inp[0]/self.inp[1])*np.sin(self.odomEKF[2]+self.inp[1]*self.loop_duration)
				self.pred[1]=(self.inp[0]/self.inp[1])*np.cos(self.odomEKF[2])-(self.inp[0]/self.inp[1])*np.cos(self.odomEKF[2]+self.inp[1]*self.loop_duration)
				self.pred[2]=self.inp[1]*self.loop_duration
			else:
				self.pred[0]=self.inp[0]*self.loop_duration*np.cos(self.yaw[0])
				self.pred[1]=self.inp[0]*self.loop_duration*np.sin(self.yaw[1])
				#pred[1]=(inp[0]/inp[1])*np.cos(self.odomEKF[2])-(inp[0]/inp[1])*np.cos(self.odomEKF[2]+inp[1]*t)
				#pred[1]=(meas[0]/meas[1])*np.cos(self.odomEKF[2])-(meas[0]/meas[1])*np.cos(self.odomEKF[2]+meas[1]*t)
				self.pred[2]=self.meas[1][1]*self.loop_duration

		self.odomEKF=self.odomEKF+self.pred
		#print(self.pred)
            
		#print('odom ekf:', self.odomEKF)
		#print(self.inp[0], self.inp[1])

		self.cmd[0][0]=self.cmd[1][0]
		self.cmd[0][1]=self.cmd[1][1]

		#print self.odomEKF  
		self.send()

if __name__ == '__main__':
	VisualInertialOdometry()
