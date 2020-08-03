#!/usr/bin/env python
import numpy as np
from scipy import optimize

RATE = 10

# for hamming smoothing
WINDOW_SIZE = 11

# for kalman filter
TRANSITION_MATRIX = [[1, 1, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 1],
                     [0, 0, 0, 1]]

OBSERVATION_MATRIX = [[1, 0, 0, 0],
                      [0, 0, 1, 0]]

def hamming_smoothing(signal, window_size):
    padded = np.r_[signal[window_size-1:0:-1], signal, signal[-2:-window_size-1:-1]] # pad the signal at two ends
    window = np.hamming(window_size)
    smoothed = np.convolve(window/window.sum(), padded, mode='valid')
    return smoothed[window_size/2-1:-window_size/2]

def circle_fitting(locations, prediction_points=5):
    """
    Fit a circle on locations using least square.
    Return :    Predictions of x and y
                Tangent angle at the last prediction position
    """
    x = locations[:, 0]
    y = locations[:, 1]

    x_m = np.mean(x)
    y_m = np.mean(y)

    def calc_R(xc, yc):
        """ calculate the distance of each 2D points from the center (xc, yc) """
        return np.sqrt((x-xc)**2 + (y-yc)**2)

    def f(c):
        """ calculate the algebraic distance between the data points and the mean circle centered at c=(xc, yc) """
        Ri = calc_R(*c)
        return Ri - Ri.mean()

    center_estimate = x_m, y_m
    center, _ = optimize.leastsq(f, center_estimate)

    xc, yc = center
    Ri = calc_R(xc, yc)
    R = Ri.mean()

    angles = np.arctan2(y-yc, x-xc)

    angles_diff = angles[:-1]-angles[1:]
    angles_diff[angles_diff<-np.pi] += 2*np.pi
    mean_angle_variation = np.mean(np.minimum(angles_diff, 2*np.pi-angles_diff))

    angle_prediction = np.linspace(0, (prediction_points-1)*mean_angle_variation, prediction_points) + angles[0]
    x_prediction = R*np.cos(angle_prediction)+xc
    y_prediction = R*np.sin(angle_prediction)+yc

    tangent_angle = angle_prediction[-1] + np.pi/2 # the tangent angle is 90 degrees more
    if mean_angle_variation < 0: # if its movement is clock-wise, flip the direction of 180 degrees
        tangent_angle += np.pi

    return np.array(zip(x_prediction, y_prediction)), tangent_angle