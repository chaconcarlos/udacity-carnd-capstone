import os

import rospy
from styx_msgs.msg import TrafficLight
from .label_image import load_graph, load_labels


class TLClassifier(object):
    def __init__(self, tf_graph, tf_labels):
        if not os.path.exists(tf_graph) or not os.path.exists(tf_labels):
            raise ValueError
        self.graph = load_graph(tf_graph)
        rospy.loginfo('tensorflow graph loaded')
        self.labels = load_labels(tf_labels)
        rospy.loginfo('classification labels loaded')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # TODO implement light color prediction
        return TrafficLight.UNKNOWN
