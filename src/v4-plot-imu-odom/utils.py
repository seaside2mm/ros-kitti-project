#!/usr/bin/env python
import numpy as np
from collections import deque
import os

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

class Object():
    #trajectory
    def __init__(self, center, max_length):
        self.locations = deque(maxlen=max_length) # save loc
        self.locations.appendleft(center)
        self.max_length = max_length


    def update(self, center, displacement, yaw):
        """
        Update the center of the object, and calculates the velocity
        """
        for i in range(len(self.locations)):
            x0, y0 = self.locations[i]
            x1 = x0 * np.cos(yaw) + y0 * np.sin(yaw) - displacement
            y1 = -x0 * np.sin(yaw) + y0 * np.cos(yaw)
            self.locations[i] = np.array([x1, y1])

        if center is not None:
            self.locations.appendleft(center) 

    def reset(self):
        self.locations = deque(maxlen=self.max_length)

