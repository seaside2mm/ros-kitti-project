#!/usr/bin/env python2
import os
import numpy as np
import pandas as pd

import rospy
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pcl2
import tf

FRAME_ID = 'map'
DETECTION_COLOR_DICT = {}
DETECTION_COLOR_DICT[1] = (255,255,0) # Car
DETECTION_COLOR_DICT[2] = (63,133,205) # Truck/Bus
DETECTION_COLOR_DICT[3] = (0,255,0) # Pedestrian
DETECTION_COLOR_DICT[4] = (147,20,255) # Cyclist
DETECTION_COLOR_DICT[5] = (192,192,192) # Other

LINES = [[0, 1], [1, 2], [2, 3], [3, 0]] # lower face
LINES+= [[4, 5], [5, 6], [6, 7], [7, 4]] # upper face
LINES+= [[4, 0], [5, 1], [6, 2], [7, 3]] # connect lower face and upper face
LINES+= [[4, 1], [5, 0]] # front face

SEQUENCE_NAME = 'result_9048_3_frame'
DATA_PATH = '/media/ubuntu/HDD/data/Apolloscape/Detection_Tracking/tracking_train_pcd_1/%s'%SEQUENCE_NAME
# EGOCAR = np.array([[2.15, 0.9, -1.73], [2.15, -0.9, -1.73], [-1.95, -0.9, -1.73], [-1.95, 0.9, -1.73], 
#                    [2.15, 0.9, -0.23], [2.15, -0.9, -0.23], [-1.95, -0.9, -0.23], [-1.95, 0.9, -0.23]])

RATE = 1
LIFETIME = 1.0/RATE

def read_tracking(path):
    df = pd.read_csv(path, header=None, sep=' ')
    df.columns = ['frame', 'track_id', 'type', 'pos_x', 'pos_y', 'pos_z', 'length', 'width', 'height', 'rot_y']
    return df

def read_point_cloud(path):
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)

def compute_3d_box_velo(x, y, z, l, w, h, yaw):
    """
    Return : 3xn in velo coordinate
    """
    R = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    x_corners = [l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2]
    y_corners = [w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2]
    z_corners = [-h/2,-h/2,-h/2,-h/2,h/2,h/2,h/2,h/2]
    corners_3d_velo = np.dot(R, np.vstack([x_corners,y_corners,z_corners]))
    corners_3d_velo += np.vstack([x, y, z])
    return corners_3d_velo

def publish_point_cloud(pcl_pub, point_cloud):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = FRAME_ID
    pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:, :3]))

def publish_3dbox(box3d_pub, corners_3d_velos, types, track_ids):
    marker_array = MarkerArray()
    for i, corners_3d_velo in enumerate(corners_3d_velos):
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = rospy.Time.now()

        marker.id = i
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(LIFETIME)
        marker.type = Marker.LINE_LIST

        b, g, r = DETECTION_COLOR_DICT[types[i]]
        marker.color.r = r/255.0
        marker.color.g = g/255.0
        marker.color.b = b/255.0

        marker.color.a = 1.0

        marker.scale.x = 0.1

        marker.points = []
        for l in LINES:
            p1 = corners_3d_velo[l[0]]
            marker.points.append(Point(p1[0], p1[1], p1[2]))
            p2 = corners_3d_velo[l[1]]
            marker.points.append(Point(p2[0], p2[1], p2[2]))
        marker_array.markers.append(marker)

        text_marker = Marker()
        text_marker.header.frame_id = FRAME_ID
        text_marker.header.stamp = rospy.Time.now()

        text_marker.id = i + 1000
        text_marker.action = Marker.ADD
        text_marker.lifetime = rospy.Duration(LIFETIME)
        text_marker.type = Marker.TEXT_VIEW_FACING

        p = np.mean(corners_3d_velo, axis=0) # center

        text_marker.pose.position.x = p[0]
        text_marker.pose.position.y = p[1]
        text_marker.pose.position.z = p[2] + 1

        text_marker.text = str(track_ids[i])

        text_marker.scale.x = 1
        text_marker.scale.y = 1
        text_marker.scale.z = 1

        b, g, r = DETECTION_COLOR_DICT[types[i]]
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        marker_array.markers.append(text_marker)

    box3d_pub.publish(marker_array)

class Object():
    def __init__(self, center):
        self.locations = deque(maxlen=20)
        self.locations.appendleft(center)

    def update(self, center, displacement, yaw):
        for i in range(len(self.locations)):
            x0, y0 = self.locations[i]
            x1 = x0 * np.cos(yaw_change) + y0 * np.sin(yaw_change) - displacement
            y1 = -x0 * np.sin(yaw_change) + y0 * np.cos(yaw_change)
            self.locations[i] = np.array([x1, y1])

        if center is not None:
            self.locations.appendleft(center)

    def reset(self):
        self.locations = deque(maxlen=20)

if __name__ == '__main__':
    i = 0
    rospy.init_node('apollo_node', anonymous=True)
    pcl_pub = rospy.Publisher('apollo_point_cloud', PointCloud2, queue_size=10)
    box3d_pub = rospy.Publisher('apollo_3d', MarkerArray, queue_size=10)
    rate = rospy.Rate(RATE)

    df_tracking = read_tracking('/media/ubuntu/HDD/data/Apolloscape/Detection_Tracking/tracking_train_label_1/%s.txt'%SEQUENCE_NAME)

    tracker = {} # track_id : Object

    frames = sorted([int(path[:-4]) for path in os.listdir(DATA_PATH)])
    sequence_length = len(frames)

    while not rospy.is_shutdown():
        frame = frames[i]
        df_tracking_frame = df_tracking[df_tracking.frame==frame]

        types = np.array(df_tracking_frame['type'])
        boxes_3d = np.array(df_tracking_frame[['pos_x', 'pos_y', 'pos_z', 'length', 'width', 'height', 'rot_y']])
        track_ids = np.array(df_tracking_frame['track_id'])

        corners_3d_velos = []
        for track_id, box_3d in zip(track_ids, boxes_3d):
            corners_3d_velo = compute_3d_box_velo(*box_3d).T
            corners_3d_velos += [corners_3d_velo]

        points = read_point_cloud(os.path.join(DATA_PATH, '%d.bin'%frames[i]))

        publish_point_cloud(pcl_pub, points)
        publish_3dbox(box3d_pub, corners_3d_velos, types, track_ids)

        rospy.loginfo("published frame %d"%frame)
        rate.sleep()
        i += 1
        if i == sequence_length:
            i = 0
