from styx_msgs.msg import TrafficLight

from cv_classifier import CVClassifier
from ml_classifier import MLClassifier


class TLClassifier(object):
    def __init__(self, model_path=None, labels_path=None):
        # DONE load classifier
        if model_path:
            self.model_path = model_path
            self.classifier = MLClassifier(model_path, labels_path)
        else:  # computer vision based Classifier
            self.classifier = CVClassifier()

        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # TODO implement light color prediction
        return self.classifier.get_classification(image)
