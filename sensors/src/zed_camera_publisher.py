#!/usr/bin/env python

import tf
import cv2
import yaml
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray

class ZEDCameraPublisher(object):
    def __init__(self):

        self.bridge = CvBridge()

        self.left_pub = rospy.Publisher('/zed/left/image_rect_color', Image, queue_size=1)
        self.right_pub = rospy.Publisher('/zed/right/image_rect_color', Image, queue_size=1)
        self.depth_pub = rospy.Publisher('/zed/depth/depth_registered', Image, queue_size=1)
        self.confidence_pub = rospy.Publisher('/zed/confidence/confidence_map', Image, queue_size=1)
        self.cloud_pub = rospy.Publisher('/zed/point_cloud/cloud_registered', PointCloud2, queue_size=1)

        rospy.init_node('zed_camera_publisher')

        self.test_image = cv2.imread('naturo-monkey-selfie.jpg') # For testing purposes only

        # img = self.bridge.cv2_to_imgmsg(self.test_image, "bgr8")

        # self.left_pub.publish(img)
        # self.right_pub.publish(img)
        # self.depth_pub.publish(img)
        # self.confidence_pub.publish(img)
        # self.cloud_pub.publish(PointCloud2())
        
        # print('ZED data published')

        # rospy.spin()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self.publish_left_rgb()
                self.publish_right_rgb()
                self.publish_depth_image()
                self.publish_point_cloud()
                self.publish_confidence_map()
            except CvBridgeError as e:
                print(e)
                print('Error publishing mono image.')
            rate.sleep()

    def publish_left_rgb(self):
        # read left channel from ZED camera hera
        left_rgb = self.test_image
        
        # publish
        self.left_pub.publish(self.bridge.cv2_to_imgmsg(left_rgb, "bgr8"))

    def publish_right_rgb(self):
        # read right channel from ZED camera hera
        right_rgb = self.test_image

        # publish
        self.right_pub.publish(self.bridge.cv2_to_imgmsg(right_rgb, "bgr8"))

    def publish_depth_image(self):
        # read depth image from ZED camera hera
        depth_image = self.test_image

        # publish
        self.depth_pub.publish(self.bridge.cv2_to_imgmsg(depth_image, "bgr8")) # TODO: Make sure the conversion here is correct

    def publish_confidence_map(self):
        # read confidence map from ZED camera hera
        confidence_map = self.test_image

        # publish
        self.confidence_pub.publish(self.bridge.cv2_to_imgmsg(confidence_map, "bgr8")) # TODO: Make sure the conversion here is correct

    def publish_point_cloud(self):
        # read point cloud from ZED camera hera
        point_cloud = PointCloud2()

        # publish
        self.cloud_pub.publish(point_cloud)


if __name__ == '__main__':
    try:
        ZEDCameraPublisher()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start zed_camera_publisher node.')
