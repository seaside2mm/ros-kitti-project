#!/usr/bin/env python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy

import cv2
import numpy as np

class Image_Subscriber():
	def __init__(self):
		self.bridge = CvBridge()
		#self.sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.callback)
		self.rgb_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_callback)

	def rgb_callback(self, data):
		rospy.loginfo("showing rgb image")
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		cv2.namedWindow("rgb image", cv2.WINDOW_NORMAL)
		cv2.resizeWindow("rgb image", 400, 400)
		cv2.imshow("rgb image", cv_image)
		cv2.waitKey(1)

if __name__ == '__main__':
	rospy.init_node('rgb_image_listener', anonymous=True)
	image_subscriber = Image_Subscriber()
	rospy.spin()
	cv2.destroyAllWindows()
