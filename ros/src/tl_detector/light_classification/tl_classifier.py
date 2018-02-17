from styx_msgs.msg import TrafficLight

import tensorflow as tf
from keras.models import load_model
from keras.utils.data_utils import get_file
import os
import numpy as np
import cv2
from cv_classifier import estimate_label

class TLClassifier(object):
    def __init__(self, model_path=None):
        # DONE load classifier
        if model_path:
            self.model_path = model_path
            self.is_cv_based = False # Use ML based model
        else:
            self.is_cv_based = True # Use Computer Vision based classification

        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        if self.is_cv_based:
            return estimate_label(image)
        return TrafficLight.UNKNOWN
