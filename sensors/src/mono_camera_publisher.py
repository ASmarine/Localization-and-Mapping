#!/usr/bin/env python

import tf
import cv2
import yaml
import rospy
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

class MonoCameraPublisher(object):
    def __init__(self):

        self.bridge = CvBridge()

        self.mono_pub = rospy.Publisher('/mono_image_color', Image, queue_size=1)

        rospy.init_node('mono_camera_publisher')

        self.test_image = cv2.imread('naturo-monkey-selfie.jpg')

        self.mono_pub.publish(self.bridge.cv2_to_imgmsg(self.test_image, "bgr8"))
        # print('Mono published')

        rospy.spin()

        # rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     try:
        #         self.mono_pub.publish(self.bridge.cv2_to_imgmsg(self.test_image, "bgr8"))
        #         print('Mono published..')
        #     except CvBridgeError as e:
        #         print(e)
        #         print('Error publishing mono image.')
        #     rate.sleep()


if __name__ == '__main__':
    try:
        MonoCameraPublisher()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start object detector tester node.')
