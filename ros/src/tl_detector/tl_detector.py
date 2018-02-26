#!/usr/bin/env python
import rospy
import scipy.spatial
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import io
from PIL import Image as PIL_Image
import numpy as np
import uuid

STATE_COUNT_THRESHOLD = 3
LOOP_RATE = 20


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher(
            '/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier('./tf_files/retrained_graph.pb',
                                             './tf_files/retrained_labels.txt')
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.has_waypoints = False
        self.has_pose = False

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray,
                                self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        self.save_images_min_dist = rospy.get_param('~save_images_min_dist')
        rospy.loginfo('save img min dist: {}'.format(
            self.save_images_min_dist))

        self.save_images = rospy.get_param('~save_images')
        self.use_classifier = rospy.get_param('~use_classifier')
        self.max_tl_dist = rospy.get_param('~max_tl_distance')
        
        if self.use_classifier:
            rospy.loginfo('using mobilenet traffic light classifier')

        self.has_image = False
        self.start()

    def pose_cb(self, msg):
        self.has_pose = True
        self.pose = msg

    def waypoints_cb(self, waypoints):
        if self.has_waypoints:
            return

        self.waypoints = waypoints.waypoints

        data = np.zeros((len(self.waypoints), 2), dtype=np.float32)
        for idx, wp in enumerate(self.waypoints):
            xy = (wp.pose.pose.position.x, wp.pose.pose.position.y)
            data[idx, :] = xy
        self.kdtree = scipy.spatial.KDTree(data)
        self.has_waypoints = True

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

    def publish(self):
        if not (self.has_pose and self.has_waypoints):
            return
        light_wp, state = self.process_traffic_lights()
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        # rospy.loginfo("tl_detector - publishing state: %s light_wp %s", state,
        #   light_wp)
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def calculate_distance(self, x1, x2, y1, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def get_closest_waypoint_xy(self, pt):
        d, kdwp = self.kdtree.query(pt)
        return kdwp

    def get_closest_waypoint_idx(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        d, kdwp = self.kdtree.query((pose.position.x, pose.position.y))
        return kdwp

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if (not self.has_image):
            self.prev_light_loc = None
            return TrafficLight.UNKNOWN

        rgb = self.bridge.imgmsg_to_cv2(self.camera_image)

        if self.use_classifier:
            light_state = self.light_classifier.get_classification(rgb)
            rospy.logdebug('inference: {} / ground truth {}'.format(
                light_state, light.state))
        else:
            light_state = light.state

        if self.save_images:
            dist_to_light = self.calculate_distance(
                self.pose.pose.position.x, light.pose.pose.position.x,
                self.pose.pose.position.y, light.pose.pose.position.y)

            if dist_to_light <= self.save_images_min_dist:
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                cv2.imwrite("./data/test/test-{}-{}.{}.jpg".format(
                    light_state, light.state, self.camera_image.header.seq),
                            bgr)

        return light_state

    def is_ahead_of(self, pose, x, y):
        """Determines whether a wappoint is ahead of position
        Args:
            pose: base position
            x (float): waypoint's global x-coordinate
            y (float): waypoint's global y-coordinate
        Returns:
            bool: Whether the waypoint is ahead of this position
        """
        x1 = pose.position.x
        y1 = pose.position.y
        orientation = pose.orientation
        euler = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        yaw = euler[2]
        return ((x - x1) * math.cos(yaw) + (y - y1) * math.sin(yaw)) > 0

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_wp = -1

        if not self.waypoints:
            return -1, TrafficLight.UNKNOWN
        if not self.lights:
            return -1, TrafficLight.UNKNOWN

        stop_line_positions = self.config['stop_line_positions']
        car_position_idx = 0
        if (self.pose):
            car_position_idx = self.get_closest_waypoint_idx(self.pose.pose)
        car_position = self.waypoints[car_position_idx]
        #rospy.loginfo("tl_detector - pose %s %s", self.pose.pose.position, car_position.pose.pose.position)

        #TODO find the closest visible traffic light (if one exists)
        # Fined the closet visible traffic light based on current position
        dist_min = float('inf')
        closest_stop_line_idx = -1
        for i, stop_line_pose in enumerate(stop_line_positions):
            if self.is_ahead_of(car_position.pose.pose, stop_line_pose[0],
                                stop_line_pose[1]):
                dist = self.calculate_distance(
                    car_position.pose.pose.position.x, stop_line_pose[0],
                    car_position.pose.pose.position.y, stop_line_pose[1])
                if dist < dist_min:
                    dist_min = dist
                    closest_stop_line_idx = i

        if dist_min < self.max_tl_dist and closest_stop_line_idx >= 0:
            # Find wp index in waypoints which is closest to the traffic light
            light = self.lights[closest_stop_line_idx]
            dist_min = float('inf')
            wp = self.get_closest_waypoint_xy(
                stop_line_positions[closest_stop_line_idx])

            if wp > car_position_idx:
                light_wp = wp

            state = self.get_light_state(light)
            return light_wp, state
        return -1, TrafficLight.UNKNOWN

    def start(self):
        rospy.loginfo("Traffic Light Detector - Starting")
        rate = rospy.Rate(LOOP_RATE)
        while not rospy.is_shutdown():
            if self.has_image:
                self.publish()
            rate.sleep()


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
