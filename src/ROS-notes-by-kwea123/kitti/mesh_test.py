#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf
import numpy as np

rospy.init_node("marker_node", anonymous=True)

pub = rospy.Publisher("marker_test", Marker, queue_size=10)
rate = rospy.Rate(25)

while not rospy.is_shutdown():
	marker = Marker()
	marker.header.frame_id = "map"
	marker.header.stamp = rospy.Time.now()

	marker.id = 0
	marker.action = Marker.ADD
	marker.lifetime = rospy.Duration()
	marker.type = Marker.MESH_RESOURCE
	marker.mesh_resource = "package://kitti/bmw_x5/BMW X5 4.dae"
	# marker.mesh_use_embedded_materials = True

	marker.pose.position.x = 0.0
	marker.pose.position.y = 0.0
	marker.pose.position.z = -1.73

	q = tf.transformations.quaternion_from_euler(np.pi/2, 0, np.pi);
	marker.pose.orientation.x = q[0]
	marker.pose.orientation.y = q[1]
	marker.pose.orientation.z = q[2]
	marker.pose.orientation.w = q[3]

	marker.color.r = 1.0
	marker.color.g = 1.0
	marker.color.b = 1.0
	marker.color.a = 0.8

	marker.scale.x = 1.0
	marker.scale.y = 1.0
	marker.scale.z = 1.0

	pub.publish(marker)
	rospy.loginfo("marker published")
	rate.sleep()
