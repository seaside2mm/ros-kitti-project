#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

rospy.init_node("marker_lifetime_node", anonymous=True)

pub = rospy.Publisher("marker_test", Marker, queue_size=10)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
	for i in range(10):
		marker = Marker()
		marker.header.frame_id = "map"
		marker.header.stamp = rospy.Time.now()

		marker.id = i
		marker.action = Marker.ADD
		marker.lifetime = rospy.Duration(0.5)
		marker.type = Marker.LINE_LIST

		marker.color.r = 1.0
		marker.color.g = 0.0
		marker.color.b = 0.0
		marker.color.a = 1.0

		marker.scale.x = 0.1

		marker.points = []
		# first point
		first_line_point = Point()
		first_line_point.x = i
		first_line_point.y = 0
		first_line_point.z = 0.0
		marker.points.append(first_line_point)
		# second point
		second_line_point = Point()
		second_line_point.x = i+0.5
		second_line_point.y = 0
		second_line_point.z = 0.0
		marker.points.append(second_line_point)

		pub.publish(marker)
		rospy.loginfo("marker %d published"%i)
		rate.sleep()
