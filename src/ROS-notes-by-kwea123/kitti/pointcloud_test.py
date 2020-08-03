#!/usr/bin/env python
import rospy
import sys

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

if __name__ == '__main__':
	rospy.init_node('pcl2_pub_example')
	pcl_pub = rospy.Publisher("/my_pcl_topic", PointCloud2)
	rospy.loginfo("Initializing sample pcl2 publisher node...")
	#give time to roscore to make the connections
	rospy.sleep(1.)
	cloud_points = [[1.0, 1.0, 0.0],[1.0, 2.0, 0.0]]
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		#header
		header = std_msgs.msg.Header()
		header.stamp = rospy.Time.now()
		header.frame_id = 'map'
		
		#create pcl from points
		scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_points)
		
		#publish    
		rospy.loginfo("publishing sample pointcloud.. !")
		pcl_pub.publish(scaled_polygon_pcl)
		rate.sleep()
    
