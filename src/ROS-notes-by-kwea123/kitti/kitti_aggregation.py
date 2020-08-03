#!/usr/bin/env python
import numpy as np
import yaml
import os
import struct
import time
from collections import deque
import cPickle as pickle

import rospy

from publish_utils import *
from kitti_data_utils import *
from processing_utils import *

def compute_3d_box_cam2(h, w, l, x, y, z, yaw):
    """
    Return : 3xn in cam2 coordinate
    """
    R = np.array([[np.cos(yaw), 0, np.sin(yaw)], [0, 1, 0], [-np.sin(yaw), 0, np.cos(yaw)]])
    x_corners = [l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2]
    y_corners = [0,0,0,0,-h,-h,-h,-h]
    z_corners = [w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2]
    corners_3d_cam2 = np.dot(R, np.vstack([x_corners,y_corners,z_corners]))
    corners_3d_cam2[0,:] += x
    corners_3d_cam2[1,:] += y
    corners_3d_cam2[2,:] += z
    return corners_3d_cam2

def cam2_3d_to_velo(corners_3d_cam2):
    """
    Input : 3xn in cam2 coordinate
    Return : 4xn in velo coordinate
    """
    return np.linalg.inv(Tr_velo_to_cam2).dot(np.r_[corners_3d_cam2, np.ones((1, 8))])

def compute_center_of_box(corners_3d_velo):
    return np.mean(corners_3d_velo, axis=0)

def in_hull(p, hull):
    from scipy.spatial import Delaunay
    if not isinstance(hull, Delaunay):
        hull = Delaunay(hull)
    return hull.find_simplex(p)>=0

def extract_pc_in_box3d(pc, box3d):
    ''' pc: (N,3+), box3d: (8,3+) '''
    box3d_roi_inds = in_hull(pc[:, :3], box3d)
    return pc[box3d_roi_inds], box3d_roi_inds

def rgb_to_float32(r, g, b):
    """
    Input : r, g, b integer values in range [0, 255]
    Output : The same number in float32 format
    """
    rgb_uint32 = (r<<16) + (g<<8) + b
    return struct.unpack('f', struct.pack('I', rgb_uint32))[0]

def compute_great_circle_distance(lat1, lon1, lat2, lon2):
    """
    Compute the great circle distance from two gps data
    Input   : latitudes and longitudes in degree
    Output  : distance in meter
    """
    delta_sigma = float(np.sin(lat1*np.pi/180)*np.sin(lat2*np.pi/180)+ \
                        np.cos(lat1*np.pi/180)*np.cos(lat2*np.pi/180)*np.cos(lon1*np.pi/180-lon2*np.pi/180))
    if np.abs(delta_sigma) > 1:
        return 0.0
    return 6371000.0 * np.arccos(delta_sigma)


DATA_PATH = '/root/kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync/'

with open('/root/kitti/RawData/2011_09_26/calib_velo_to_cam.txt', 'r') as f:
    yml = yaml.load(f)

R_velo_to_cam2 = np.array([float(i) for i in yml['R'].split(' ')]).reshape(3, 3)
T_velo_to_cam2 = np.array([float(i) for i in yml['T'].split(' ')]).reshape(3, 1)
Tr_velo_to_cam2 = np.vstack((np.hstack([R_velo_to_cam2, T_velo_to_cam2]), [0, 0, 0, 1]))

RANDOM_COLORS = [np.random.randint(255, size=3) for _ in range(1000)]
COLOR_WHITE = rgb_to_float32(255, 255, 255)

class Object():
    def __init__(self, center, max_length, velocity_smoothing):
        self.locations = deque(maxlen=max_length)
        self.locations.appendleft(center)
        self.velocities = deque(maxlen=max_length)
        self.max_length = max_length
        self.velocity_smoothing = velocity_smoothing

        if velocity_smoothing:
            self.velocities_smoothed = deque(maxlen=max_length)

    def update(self, center, displacement, yaw):
        """
        Update the center of the object, and calculates the velocity
        """
        for i in range(len(self.locations)):
            x0, y0 = self.locations[i]
            x1 = x0 * np.cos(yaw) + y0 * np.sin(yaw) - displacement
            y1 = -x0 * np.sin(yaw) + y0 * np.cos(yaw)
            self.locations[i] = np.array([x1, y1])
        self.locations.appendleft(center)

        if len(self.locations) > 1:
            self.velocities.appendleft(np.linalg.norm(self.locations[0]-self.locations[1]) * RATE)
            if self.velocity_smoothing:
                if len(self.velocities) > 1: # if there's enough data to smooth
                    smooth_size = min(WINDOW_SIZE, len(self.velocities))
                    self.velocities_smoothed.appendleft(hamming_smoothing(np.array(self.velocities)[:smooth_size], smooth_size)[0])
                else:
                    self.velocities_smoothed.appendleft(self.velocities[0])

    def is_full(self):
        return len(self.locations) >= self.max_length//2

class Localizer():
    def __init__(self, loc_pub, max_length=20, velocity_smoothing=True, log=False):
        max_length = max(max_length, WINDOW_SIZE) # to be able to smooth
        self.loc_pub = loc_pub
        self.prev_imu_data = None
        self.velocity_smoothing = velocity_smoothing
        self.max_length = max_length
        self.ego_car = Object([0, 0], max_length=self.max_length, velocity_smoothing=self.velocity_smoothing)
        self.log = log

    def update(self, imu_data):
        if self.prev_imu_data is not None:
            displacement = compute_great_circle_distance(self.prev_imu_data.lat, self.prev_imu_data.lon,
                                                         imu_data.lat, imu_data.lon)
            yaw = float(imu_data.yaw - self.prev_imu_data.yaw)
            self.ego_car.update([0, 0], displacement, yaw)
        self.prev_imu_data = imu_data

    def reset(self):
        """
        Empty the data when the sequence has reached the end
        """
        self.prev_imu_data = None
        self.ego_car = Object([0, 0], max_length=self.max_length, velocity_smoothing=self.velocity_smoothing)

    def publish(self, publish_velocity=False):
        if self.velocity_smoothing:
            publish_location(self.loc_pub, self.ego_car.locations, self.ego_car.velocities_smoothed, publish_velocity=publish_velocity, log=self.log)
        else:
            publish_location(self.loc_pub, self.ego_car.locations, self.ego_car.velocities, publish_velocity=publish_velocity, log=self.log)

    def save(self):
        with open('locations.pickle', 'wb') as f:
            pickle.dump(self.locations, f)
        with open('v.pickle', 'wb') as f:
            pickle.dump(self.velocities_smoothed, f)

class Tracker():
    def __init__(self, tracker_pub, max_length=20, velocity_smoothing=True, log=False):
        max_length = max(max_length, WINDOW_SIZE) # to be able to smooth
        self.tracker_pub = tracker_pub
        self.max_length = max_length
        self.prev_imu_data = None
        self.objects_to_track = {}
        self.velocity_smoothing = velocity_smoothing
        self.log = log

    def update(self, imu_data, track_ids, centers):
        if self.prev_imu_data is None: # if it's the very first frame
            for track_id, center in zip(track_ids, centers):
                self.objects_to_track[track_id] = Object(center, max_length=self.max_length, velocity_smoothing=self.velocity_smoothing)
        else:
            displacement = compute_great_circle_distance(self.prev_imu_data.lat, self.prev_imu_data.lon,
                                                         imu_data.lat, imu_data.lon)
            yaw = float(imu_data.yaw - self.prev_imu_data.yaw)
            for track_id, center in zip(track_ids, centers):
                if track_id not in self.objects_to_track: # if it's a new object
                    self.objects_to_track[track_id] = Object(center, max_length=self.max_length, velocity_smoothing=self.velocity_smoothing)
                else:
                    self.objects_to_track[track_id].update(center, displacement, yaw)

            # delete objects that disappear instantly (need to modify later)
            track_ids_to_delete = []
            for track_id in self.objects_to_track:
                if track_id not in track_ids:
                    track_ids_to_delete += [track_id]
            for track_id in track_ids_to_delete:
                self.objects_to_track.pop(track_id)

        self.prev_imu_data = imu_data

    def reset(self):
        self.prev_imu_data = None
        self.objects_to_track.clear()

    def publish(self, publish_velocity=False):
        publish_trajectory(self.tracker_pub, self.objects_to_track, publish_velocity=publish_velocity, 
                           velocity_smoothing=self.velocity_smoothing, log=self.log)
    def save(self):
        with open('locations_others.pickle', 'wb') as f:
            pickle.dump(self.objects_to_track[0].locations, f)

if __name__ == '__main__':

    log = False # log info to the console or not
    # create node and publishers
    rospy.init_node('kitti_pointcloud_node', anonymous=True)
    cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
    cam_gt_pub = rospy.Publisher('kitti_cam_gt', Image, queue_size=10)
    bridge = CvBridge()
    pcl_pub = rospy.Publisher('kitti_pointcloud', PointCloud2, queue_size=10)
    ego_car_pub = rospy.Publisher('kitti_ego_car', MarkerArray, queue_size=10)
    box3d_pub = rospy.Publisher('kitti_3dboxes', MarkerArray, queue_size=10)
    imu_pub = rospy.Publisher('kitti_imu', Imu, queue_size=10)
    gps_pub = rospy.Publisher('kitti_gps', NavSatFix, queue_size=10)
    loc_pub = rospy.Publisher('kitti_loc', MarkerArray, queue_size=10)
    tracker_pub = rospy.Publisher('kitti_trajectories', MarkerArray, queue_size=10)
    rate = rospy.Rate(10)

    df_tracking = read_tracking('/root/kitti/training/label_02/0000.txt')
    sequence_length = 150

    localizer = Localizer(loc_pub)
    tracker = Tracker(tracker_pub, log=log)

    # start publishing
    frame = 0
    while not rospy.is_shutdown():

        # read camera data of the current frame
        image = read_camera(os.path.join(DATA_PATH, 'image_02/data/%010d.png'%frame))
        
        # extract tracking data of the current frame
        df_tracking_frame = df_tracking[df_tracking['frame']==frame]
        df_tracking_frame.reset_index(inplace=True, drop=True)

        # read imu data of the current frame 
        df_imu_frame = read_imu(os.path.join(DATA_PATH, 'oxts/data/%010d.txt'%frame))

        # read point cloud of the current frame
        point_cloud = read_velodyne(os.path.join(DATA_PATH, 'velodyne_points/data/%010d.bin'%frame))
        # downsample the point cloud
        point_cloud = point_cloud[::3]
        # set default point cloud color to white
        point_cloud[:, 3] = COLOR_WHITE

        # read 2d and 3d boxes
        borders_2d_cam2s = []
        object_types = []
        corners_3d_velos = []
        centers = []
        track_ids = np.array(df_tracking_frame['track_id'])
        for i in range(len(df_tracking_frame)):

            borders_2d_cam2 = np.array(df_tracking_frame.loc[i, ['bbox_left', 'bbox_top', 'bbox_right', 'bbox_bottom']])
            borders_2d_cam2s += [borders_2d_cam2]
            object_types += [df_tracking_frame.loc[i, 'type']]

            corners_3d_cam2 = compute_3d_box_cam2(*np.array(df_tracking_frame.loc[i, ['height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y']]))
            corners_3d_velo = cam2_3d_to_velo(corners_3d_cam2).T # 8x4
            corners_3d_velos += [corners_3d_velo]
            centers += [compute_center_of_box(corners_3d_velo)[:2]]
            
            # # set different color for point cloud in each object (slow)
            # box3droi_pc_velo, box3d_roi_inds = extract_pc_in_box3d(point_cloud, corners_3d_velo[:, :3])
            # point_cloud[box3d_roi_inds, 3] = rgb_to_float32(*RANDOM_COLORS[df_tracking_frame.loc[i, 'track_id']])

        # update the localizer
        localizer.update(df_imu_frame)
        # update the tracker
        tracker.update(df_imu_frame, track_ids, centers)

        # publish location
        localizer.publish(publish_velocity=True)
        # publish trajectories
        tracker.publish(publish_velocity=True)
        # # publish camera image
        # publish_camera(cam_pub, bridge, image, log=log)
        # publish 2d gt
        publish_camera(cam_gt_pub, bridge, image, borders_2d_cam2s, object_types, log=log)
        
        # publish point cloud
        publish_point_cloud(pcl_pub, point_cloud, format='xyzrgb', log=log)
        # publish 3d boxes
        publish_3dbox(box3d_pub, corners_3d_velos, track_ids, object_types, publish_id=False, publish_distance=False, log=log)
        # # publish imu
        # publish_imu(imu_pub, df_imu_frame, log=log)
        # # publish gps
        # publish_gps(gps_pub, df_imu_frame, log=log)
        # publish car FOV and mesh
        publish_ego_car(ego_car_pub)

        frame += 1
        if frame == sequence_length: # if the sequence has reached the end
            frame = 0
            # localizer.save()
            localizer.reset()
            tracker.reset()
            rospy.loginfo("sequence reset !")
            # break

        rate.sleep()
