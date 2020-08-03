#!/usr/bin/env python
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import rospy

import cv2
import numpy as np

class Image_Subscriber():
	def __init__(self, trajectory_lifetime=10):
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
		self.path_pub = rospy.Publisher('path', Path, queue_size=10)
		self.path = Path()
		self.max_path_length = trajectory_lifetime * 100

	def odom_callback(self, data):
		rospy.loginfo("showing odom location")
		self.path.header = data.header
		pose = PoseStamped()
		pose.header = data.header
		pose.pose = data.pose.pose
		self.path.poses.append(pose)
		if len(self.path.poses) > self.max_path_length:
			self.path.poses.pop(0)
		self.path_pub.publish(self.path)

if __name__ == '__main__':
	rospy.init_node('odom_listener', anonymous=True)
	image_subscriber = Image_Subscriber()
	rospy.spin()
