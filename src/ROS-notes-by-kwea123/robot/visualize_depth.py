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
		self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)

	def depth_callback(self, data):
		rospy.loginfo("showing depth image")
		cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
		cv_image = np.where(np.isnan(cv_image), np.nanmax(cv_image), cv_image)
		cv_image = cv2.normalize(cv_image, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8UC1)
		cv2.namedWindow("depth image", cv2.WINDOW_NORMAL)
		cv2.resizeWindow("depth image", 400, 400)
		cv2.imshow("depth image", cv_image)
		cv2.waitKey(1)

if __name__ == '__main__':
	rospy.init_node('depth_image_listener', anonymous=True)
	image_subscriber = Image_Subscriber()
	rospy.spin()
	cv2.destroyAllWindows()
