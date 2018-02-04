#!/usr/bin/env python
import rospy
import sensor_msgs.msg as sensor_msgs
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow

# demo node to detect traffic light and draw a bounding box
# in the image displayed in an opencv window

# object detection uses code from 
# https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb


# TODO, read this from the model
NUM_CLASSES = 90
TRAFFIC_SIGNAL_CLASS_ID = 10


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector1')
        self.window = "window"
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.moveWindow(self.window, 640, 0)
        cv2.resizeWindow(self.window, 640, 480)

        self.cvbridge = CvBridge()
        self.detection_graph = self.load_model("../data/ssd_mobilenet_v1_coco_2017_11_17/frozen_inference_graph.pb")
        self.session = tensorflow.Session(graph=self.detection_graph)
        self.sub6 = rospy.Subscriber('/image_color', sensor_msgs.Image,
                                     self.image_cb, queue_size=1)
        rospy.spin()

    def load_model(self, model_path):
        detection_graph = tensorflow.Graph()
        with detection_graph.as_default():
            od_graph_def = tensorflow.GraphDef()
            with tensorflow.gfile.GFile(model_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tensorflow.import_graph_def(od_graph_def, name='')
        return detection_graph

    def drawbox(self, img, box, color):
        ymin, xmin, ymax, xmax = box
        im_height, im_width, nch = img.shape
        (left, right, top, bottom) = map(int, (xmin * im_width,
                                               xmax * im_width,
                                               ymin * im_height,
                                               ymax * im_height))
        cv2.rectangle(img, (left, top), (right, bottom), color)


    def image_cb(self, msg):
        image = self.cvbridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        boxes, scores, num = self.detect(image)
        if num > 0:
            for idx, box in enumerate(boxes):
                if scores[idx] > 0.5:
                    self.drawbox(image, box, (0, 255, 0))
        
        cv2.imshow(self.window, image)
        cv2.waitKey(100)
        return

    def detect(self, image):
        with self.detection_graph.as_default():
            # Definite input and output Tensors for detection_graph
            image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
            # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
            image_np_expanded = np.expand_dims(image, axis=0)
            # Actual detection.
            (boxes, scores, classes, num) = self.session.run(
              [detection_boxes, detection_scores, detection_classes, num_detections],
              feed_dict={image_tensor: image_np_expanded})
            idx = np.where(classes==TRAFFIC_SIGNAL_CLASS_ID)
            return boxes[idx], scores[idx], len(idx[0])

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
