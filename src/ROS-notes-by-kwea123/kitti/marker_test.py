#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

rospy.init_node("marker_node", anonymous=True)

pub = rospy.Publisher("marker_test", Marker, queue_size=10)
rate = rospy.Rate(25)

while not rospy.is_shutdown():
	marker = Marker()
	marker.header.frame_id = "map"
	marker.header.stamp = rospy.Time.now()

	#marker.ns = "basic_shapes"
	marker.id = 0
	marker.action = Marker.ADD
	marker.lifetime = rospy.Duration()
	marker.type = Marker.LINE_LIST

	#marker.pose.position.x = 0.0
	#marker.pose.position.y = 0.0
	#marker.pose.position.z = 0.0

	#marker.pose.orientation.x=0.0
	#marker.pose.orientation.y=0.0
	#marker.pose.orientation.z=1.0
	#marker.pose.orientation.w=1.0

	marker.color.r = 1.0
	marker.color.g = 0.0
	marker.color.b = 0.0
	marker.color.a = 1.0

	marker.scale.x = 0.01
	#marker.scale.y = 0.1
	#marker.scale.z = 0.1

	# marker line points
	marker.points = []
	# first point
	first_line_point = Point()
	first_line_point.x = 1.0
	first_line_point.y = 0.0
	first_line_point.z = 0.0
	marker.points.append(first_line_point)
	# second point
	second_line_point = Point()
	second_line_point.x = 0.0
	second_line_point.y = 0.0
	second_line_point.z = 0.0
	marker.points.append(second_line_point)

	pub.publish(marker)
	rospy.loginfo("marker published")
	rate.sleep()
