#!/usr/bin/env python
from data_utils import *
from publish_utils import *
from utils import *
from kitti_utils import *

DATA_PATH = '/root/kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync/'

if  __name__ == "__main__":
    frame = 0
    rospy.init_node('kitti_node',anonymous=True)
    cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
    pcl_pub = rospy.Publisher('kitti_point_cloud', PointCloud2, queue_size=10)
    bridge = CvBridge()
    ego_pub = rospy.Publisher('kitti_ego_car',Marker, queue_size=10)

    imu_pub = rospy.Publisher('kitti_imu',Imu, queue_size=10)
    gps_pub = rospy.Publisher('kitti_gps',NavSatFix, queue_size=10)
    box3d_pub = rospy.Publisher('kitti_3dbox',MarkerArray, queue_size=10)
    rate = rospy.Rate(10)

    df_tracking = read_tracking('/root/kitti/training/label_02/0000.txt')
    calib = Calibration('/root/kitti/RawData/2011_09_26/',from_video=True)
    while not rospy.is_shutdown():
        # read data
        df_tracking_frame = df_tracking[df_tracking.frame==frame]
        boxes_2d = np.array(df_tracking_frame[['bbox_left','bbox_top','bbox_right','bbox_bottom']])
        types = np.array(df_tracking_frame['type'])
        track_ids = np.array(df_tracking_frame['track_id'])

        image = read_camera(os.path.join(DATA_PATH, 'image_02/data/%010d.png'%frame))
        point_cloud = read_point_cloud(os.path.join(DATA_PATH, 'velodyne_points/data/%010d.bin'%frame))
        imu_data = read_imu(os.path.join(DATA_PATH,'oxts/data/%010d.txt'%frame))

        boxes_3d = np.array(df_tracking_frame[['height','width','length','pos_x','pos_y','pos_z','rot_y']])
        corner_3d_velos = []

        for box_3d in boxes_3d:
            corner_3d_cam2 = compute_3d_box_cam2(*box_3d)
            corner_3d_velo = calib.project_rect_to_velo(np.array(corner_3d_cam2).T)
            corner_3d_velos += [corner_3d_velo]

        # publish
        publish_camera(cam_pub, bridge, image, boxes_2d, types)
        publish_point_cloud(pcl_pub, point_cloud[::3])
        publish_ego_car(ego_pub)

        publish_imu(imu_pub, imu_data )
        publish_gps(gps_pub, imu_data ) #gps rviz cannot visulize, only use rostopic echo
        publish_3dbox(box3d_pub, corner_3d_velos,  track_ids, types)
        rospy.loginfo("kitti published")
        rate.sleep()
        frame += 1
        frame %= 154
