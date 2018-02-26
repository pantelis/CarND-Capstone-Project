from styx_msgs.msg import TrafficLight
import rospy
import rospkg
import numpy as np
import os
import sys
import tensorflow as tf
from collections import defaultdict
from io import StringIO
from object_detection_classifier import ObjectDetectionClassifier
import time

UNKNOWN = 'UNKNOWN'
YELLOW = 'Yellow'
GREEN = 'Green'
RED = 'Red'
NUM_CLASSES = 14


class MLClassifier(object):
    def __init__(self, model_path, labels_path):

        # set default value for no detection
        self.current_light = TrafficLight.UNKNOWN
        self.classifier = ObjectDetectionClassifier(model_path, labels_path, NUM_CLASSES)
        rospy.logdebug("Loaded the model")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        self.current_light = TrafficLight.UNKNOWN

        class_name = self.classifier.classify_image(image)
        if class_name == RED:
            self.current_light = TrafficLight.RED
        elif class_name == GREEN:
            self.current_light = TrafficLight.GREEN
        elif class_name == YELLOW:
            self.current_light = TrafficLight.YELLOW
        rospy.logdebug('Image classified as {}'.format(class_name))

        return self.current_light
