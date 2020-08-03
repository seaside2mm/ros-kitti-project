#!/usr/bin/env python
from data_utils import *
from publish_utils import *

DATA_PATH = '/root/kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync/'

if  __name__ == "__main__":
    frame = 0
    rospy.init_node('kitti_node',anonymous=True)
    cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
    pcl_pub = rospy.Publisher('kitti_point_cloud', PointCloud2, queue_size=10)
    bridge = CvBridge()
    ego_pub = rospy.Publisher('kitti_ego_car',Marker, queue_size=10)
    # model_car_pub = rospy.Publisher('kitti_model_car',Marker, queue_size=10)

    imu_pub = rospy.Publisher('kitti_imu',Imu, queue_size=10)
    gps_pub = rospy.Publisher('kitti_gps',NavSatFix, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        image = read_camera(os.path.join(DATA_PATH, 'image_02/data/%010d.png'%frame))
	point_cloud = read_point_cloud(os.path.join(DATA_PATH, 'velodyne_points/data/%010d.bin'%frame))
        imu_data = read_imu(os.path.join(DATA_PATH,'oxts/data/%010d.txt'%frame))
	publish_camera(cam_pub, bridge, image)
	publish_point_cloud(pcl_pub, point_cloud)
	publish_ego_car(ego_pub)
	# publish_car_model(model_car_pub)
	publish_imu(imu_pub, imu_data )
	publish_gps(gps_pub, imu_data ) #gps rviz cannot visulize, only use rostopic echo
        rospy.loginfo("kitti published")
        rate.sleep()
        frame += 1
        frame %= 154
