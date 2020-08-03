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
    imu_odom_pub = rospy.Publisher('kitti_imu_odom',MarkerArray, queue_size=10)
    
    rate = rospy.Rate(10)

    df_tracking = read_tracking('/root/kitti/training/label_02/0000.txt')
    calib = Calibration('/root/kitti/RawData/2011_09_26/',from_video=True)
    
    tracker = {} # save all obj odom
    prev_imu_data = None
    while not rospy.is_shutdown():
        # read file
        df_tracking_frame = df_tracking[df_tracking.frame==frame]

        boxes_2d = np.array(df_tracking_frame[['bbox_left','bbox_top','bbox_right','bbox_bottom']])
        boxes_3d = np.array(df_tracking_frame[['height','width','length','pos_x','pos_y','pos_z','rot_y']])

        types = np.array(df_tracking_frame['type'])
        track_ids = np.array(df_tracking_frame['track_id'])
        track_ids = np.append(track_ids, 1000) # append ego car

        # read data
        image = read_camera(os.path.join(DATA_PATH, 'image_02/data/%010d.png'%frame))
        point_cloud = read_point_cloud(os.path.join(DATA_PATH, 'velodyne_points/data/%010d.bin'%frame))
        imu_data = read_imu(os.path.join(DATA_PATH,'oxts/data/%010d.txt'%frame)) # include imu and gpss info
        
        corner_3d_velos = []
        centers = {} # current frame tracker. track id:center
        for track_id, box_3d in zip(track_ids, boxes_3d):
            corner_3d_cam2 = compute_3d_box_cam2(*box_3d)
            corner_3d_velo = calib.project_rect_to_velo(np.array(corner_3d_cam2).T)
            corner_3d_velos += [corner_3d_velo] # one bbox 8 x 3 array
            centers[track_id] = np.mean(corner_3d_velo, axis=0)[:2] # get ccenter of every bbox, don't care about height
        
        centers[-1] = np.array([0,0]) # for ego car, we set its id = -1, center [0,0]
        
        if prev_imu_data is None:
            for track_id in centers:
                tracker[track_id] = Object(centers[track_id], 20)
        else:
            displacement = 0.1*np.linalg.norm(imu_data[['vf','vl']])
            yaw_change = float(imu_data.yaw - prev_imu_data.yaw)
            print track_id
            for track_id in centers: # for one frame id 
                if track_id in tracker:
                    tracker[track_id].update(centers[track_id], displacement, yaw_change)
                else:
                    tracker[track_id] = Object(centers[track_id], 20)
            for track_id in tracker:# for whole ids tracked by prev frame,but current frame did not
                if track_id not in centers: # dont know its center pos
                    tracker[track_id].update(None, displacement, yaw_change)
                    
        prev_imu_data = imu_data
        
        # publish
        publish_camera(cam_pub, bridge, image, boxes_2d, types)
        publish_point_cloud(pcl_pub, point_cloud[::2])
        publish_ego_car(ego_pub)

        publish_imu(imu_pub, imu_data )
        publish_gps(gps_pub, imu_data ) #gps rviz cannot visulize, only use rostopic echo
        publish_3dbox(box3d_pub, corner_3d_velos,  track_ids, types)
        publish_imu_odom(imu_odom_pub, tracker, centers)
        rospy.loginfo("kitti published")
        rate.sleep()
        frame += 1
        if frame == 154:
            frame  = 0
            for track_id in tracker:
                tracker[track_id].reset()
