import os
import time

import rospy
from styx_msgs.msg import TrafficLight
import numpy as np
import tensorflow as tf
import cv2

# green
# none
# red
# yellow


class TLClassifier(object):
    input_height = 299
    input_width = 299
    input_mean = 128
    input_std = 128
    input_layer = "Mul"
    output_layer = "final_result"

    def __init__(self, tf_graph, tf_labels):
        if not os.path.exists(tf_graph) or not os.path.exists(tf_labels):
            raise ValueError

        self.graph = self.load_graph(tf_graph)
        rospy.loginfo('tensorflow graph loaded')
        self.labels = self.load_labels(tf_labels)
        rospy.loginfo('classification labels loaded: {}'.format(self.labels))

        input_name = "import/" + self.input_layer
        output_name = "import/" + self.output_layer

        self.input_operation = self.graph.get_operation_by_name(input_name)
        self.output_operation = self.graph.get_operation_by_name(output_name)
        self.label2trafficLightState = {
            'nonred': TrafficLight.GREEN,
            'red': TrafficLight.RED,
            # 'yellow': TrafficLight.YELLOW,
            # 'none': TrafficLight.UNKNOWN
        }
        rospy.loginfo(self.label2trafficLightState)

    def load_graph(self, model_file):
        graph = tf.Graph()
        graph_def = tf.GraphDef()

        with open(model_file, "rb") as f:
            graph_def.ParseFromString(f.read())
        with graph.as_default():
            tf.import_graph_def(graph_def)
        return graph

    def load_labels(self, label_file):
        label = []
        proto_as_ascii_lines = tf.gfile.GFile(label_file).readlines()
        for l in proto_as_ascii_lines:
            label.append(l.rstrip())
        return label

    @staticmethod
    def tensor_from_rgb_img(rgb,
                            input_height=299,
                            input_width=299,
                            input_mean=0,
                            input_std=255):
        float_caster = tf.cast(rgb, tf.float32)
        dims_expander = tf.expand_dims(float_caster, 0)
        resized = tf.image.resize_bilinear(dims_expander,
                                           [input_height, input_width])
        normalized = tf.divide(tf.subtract(resized, [input_mean]), [input_std])
        sess = tf.Session()
        result = sess.run(normalized)

        return result

    def _get_classification(self, rgb_image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light in RGB color format

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # TODO implement light color prediction
        t = self.tensor_from_rgb_img(
            rgb_image,
            input_height=self.input_height,
            input_width=self.input_width,
            input_mean=self.input_mean,
            input_std=self.input_std)

        with tf.Session(graph=self.graph) as sess:
            start = time.time()
            results = sess.run(self.output_operation.outputs[0], {
                self.input_operation.outputs[0]: t
            })
            end = time.time()
            rospy.logdebug('image classified in {:.3f}s'.format(end - start))

        results = np.squeeze(results)
        top_k = results.argsort()[-5:][::-1]
        label = self.labels[top_k[0]]
        rospy.logdebug("TLClassifier best guess {}: {}".format(
            label, results[top_k[0]]))
        return self.label2trafficLightState[label]

    def get_classification(self, rgb_image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light in RGB color format

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # threshold red pixels in hsv space

        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)

        # thresholds and parameters
        # below taken from https://github.com/carnd-teambacon/CarND-Capstone
        # Thresholds for red in HSV color space
        low_red = np.array([0, 50, 50])
        high_red = np.array([10, 255, 255])
        red1 = cv2.inRange(hsv, low_red, high_red)
        low_red = np.array([170, 50, 50])
        high_red = np.array([180, 255, 255])
        red2 = cv2.inRange(hsv, low_red, high_red)

        converted_img = cv2.addWeighted(red1, 1.0, red2, 1.0, 0.0)
        # Add Gaussian blur for better edge detection
        blur_img = cv2.GaussianBlur(converted_img, (15, 15), 0)
        # Apply Hough transform to search for circles
        detected_circles = cv2.HoughCircles(
            blur_img,
            cv2.HOUGH_GRADIENT,
            0.5,
            41,
            param1=70,
            param2=30,
            minRadius=5,
            maxRadius=150)

        if detected_circles is not None:
            return TrafficLight.RED, hsv, blur_img
        return TrafficLight.GREEN, hsv, blur_img
