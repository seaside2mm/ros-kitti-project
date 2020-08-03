#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import os
import sys
import tensorflow as tf
import time
import cv2
import yaml # load label_dict

from publish_utils import *

MODEL_PATH = '/home/ubuntu/workspace/models/research/object_detection/faster_rcnn_resnet101_kitti_2018_01_28/frozen_inference_graph.pb'

LABEL_DICT_PATH = os.path.join(os.path.dirname(__file__), 'kitti_label_dict.txt')

IMAGE_TOPIC = 'kitti_cam'

class Kitti_Detector():
    def __init__(self, model_path, label_dict_path, subscribed_topic):
        self.__load_model(model_path, label_dict_path)
        print('model loaded')
        self.__image_sub = rospy.Subscriber(subscribed_topic, Image, self.callback)
        self.__detection_pub = rospy.Publisher('kitti_detection', Image, queue_size=10)
        self.bridge = CvBridge()
        self.n_frames = 0

    def __load_model(self, model_path, label_dict_path):

        # load tf model
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        config = tf.ConfigProto()
        config.gpu_options.allow_growth= True

        with self.detection_graph.as_default():
            self.sess = tf.Session(config=config, graph=self.detection_graph)
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')

        # load label_dict
        with open(label_dict_path, 'r') as f:
            self.label_dict = yaml.load(f)
        
        # warmup
        self.detect_image(np.ones((600, 600, 3), dtype=np.uint8))

    def detect_image(self, image_bgr):

        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        image_w, image_h = image_rgb.shape[1], image_rgb.shape[0]

        image_rgb = cv2.resize(image_rgb, None, fx=1.0, fy=1.25)

        # Actual detection.
        t = time.time()
        (boxes, scores, classes) = self.sess.run(
          [self.detection_boxes, self.detection_scores, self.detection_classes],
          feed_dict={self.image_tensor: np.expand_dims(image_rgb, axis=0)})
        print('detection time :', time.time()-t)

        # Visualization of the results of a detection.
        for i, box in enumerate(boxes[scores>0.5]):
            top_left = (int(image_w*box[1]), int(image_h*box[0]))
            bottom_right = (int(image_w*box[3]), int(image_h*box[2]))
            color = DETECTION_COLOR_MAP[self.label_dict[classes[0,i]]]
            cv2.rectangle(image_bgr, top_left, bottom_right, color, 2)
            cv2.putText(image_bgr, self.label_dict[classes[0,i]], top_left, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        return image_bgr
        
    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = self.detect_image(cv_image)
        self.__detection_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        # cv2.imshow('image', cv_image)
        # cv2.waitKey(1)

if __name__ == '__main__':
    object_detector = Kitti_Detector(MODEL_PATH, LABEL_DICT_PATH, IMAGE_TOPIC)

    rospy.init_node('kitti_detector', anonymous=True)
    rospy.spin()
    cv2.destroyAllWindows()
