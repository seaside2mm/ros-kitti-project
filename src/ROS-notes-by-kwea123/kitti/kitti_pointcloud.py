#!/usr/bin/env python
import numpy as np
import sys

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

DATA_PATH = '/home/ubuntu/data/kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data/'

if __name__ == '__main__':
	frame = 0
	rospy.init_node('kitti_pointcloud_node', anonymous=True)
	pcl_pub = rospy.Publisher("kitti_pointcloud", PointCloud2, queue_size=10)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		#header
		header = std_msgs.msg.Header()
		header.stamp = rospy.Time.now()
		header.frame_id = 'map'
		
		#create pcl from points
		points = np.fromfile(DATA_PATH+'%010d.bin'%frame, dtype=np.float32).reshape(-1, 4)
		# if we want the intensity
		fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('i', 12, PointField.FLOAT32, 1)]
		pcl_msg = pcl2.create_cloud(header, fields, points)
		# # if we don't want to display the intensity
		# pcl_msg = pcl2.create_cloud_xyz32(header, points[:, :3])
		
		#publish    
		rospy.loginfo("publishing pointcloud.. !")
		pcl_pub.publish(pcl_msg)
		frame += 1
		frame %= 150
		rate.sleep()
