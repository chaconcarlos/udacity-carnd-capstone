#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import rospkg
import os
import time

STATE_COUNT_THRESHOLD = 3
LOOP_RATE = 20

CAPTURE_IMAGE = True
NUM_OF_IMAGE = 100
VISIBLE_TL_DIST = 200

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.has_image = False
        if CAPTURE_IMAGE:
            self.num_of_red = 0
            self.num_of_yellow = 0
            self.num_of_green = 0
            self.num_of_unknown = 0
            self.last_time = time.time()
            rospack = rospkg.RosPack()
            run_dir = rospack.get_path('tl_detector')
            self.image_dir = os.path.join(run_dir, 'image')
            if not os.path.exists(self.image_dir):
                os.makedirs(self.image_dir)
                os.makedirs(os.path.join(self.image_dir, 'RED'))
                os.makedirs(os.path.join(self.image_dir, 'YELLOW'))
                os.makedirs(os.path.join(self.image_dir, 'GREEN'))
                os.makedirs(os.path.join(self.image_dir, 'UNKNOWN'))

        self.start()
        #rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg
        #rospy.loginfo("current pose: %s", self.pose)

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        #rospy.loginfo("waypoints: %s", self.waypoints)

    def traffic_cb(self, msg):
        self.lights = msg.lights
        #rospy.loginfo("lights: %s", self.lights)

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

    def publish(self):
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
	#rospy.loginfo("tl_detector - publishing state: %s light_wp %s", state, light_wp)
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
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def get_closest_waypoint_idx(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        index = -1
        dist_min = float("inf")
        for i, wp in enumerate(self.waypoints):
            dist = self.calculate_distance(pose.position.x, wp.pose.pose.position.x, pose.position.y, wp.pose.pose.position.y)
            if dist < dist_min:
                dist_min = dist
                index = i
        return index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return TrafficLight.UNKNOWN

        dist = self.calculate_distance(self.pose.pose.position.x, light.pose.pose.position.x, self.pose.pose.position.y, light.pose.pose.position.y)
        if dist > VISIBLE_TL_DIST:
            return TrafficLight.UNKNOWN

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        if CAPTURE_IMAGE:
            is_enough = False
            st = time.time()
            if st - self.last_time >= 1:
                self.last_time = st
		if light.state == TrafficLight.RED:
		    image_path = os.path.join(os.path.join(self.image_dir, 'RED'), '{0}.jpg'.format(st))
                    self.num_of_red = self.num_of_red + 1
                    if self.num_of_red > NUM_OF_IMAGE:
                        is_enough = True
		elif light.state == TrafficLight.YELLOW:
		    image_path = os.path.join(os.path.join(self.image_dir, 'YELLOW'), '{0}.jpg'.format(st))
                    self.num_of_yellow = self.num_of_yellow + 1
                    if self.num_of_yellow > NUM_OF_IMAGE:
                        is_enough = True
		elif light.state == TrafficLight.GREEN:
		    image_path = os.path.join(os.path.join(self.image_dir, 'GREEN'), '{0}.jpg'.format(st))
                    self.num_of_green = self.num_of_green + 1
                    if self.num_of_green > NUM_OF_IMAGE:
                        is_enough = True
		else:
		    image_path = os.path.join(os.path.join(self.image_dir, 'UNKNOWN'), '{0}.jpg'.format(st))
                    self.num_of_unknown = self.num_of_unknown + 1
                    if self.num_of_unknown > NUM_OF_IMAGE:
                        is_enough = True
                if not is_enough:
		    cv2.imwrite(image_path, cv_image)
		    rospy.loginfo('saved image %s', image_path)

        #Get classification
        #return self.light_classifier.get_classification(cv_image)
	return light.state

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
        euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
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

	#rospy.loginfo("tl_detector - waypoints len: %s light num %s", len(self.waypoints), len(self.lights))
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        car_position_idx = 0
        if(self.pose):
            car_position_idx = self.get_closest_waypoint_idx(self.pose.pose)
        car_position = self.waypoints[car_position_idx]
	#rospy.loginfo("tl_detector - pose %s %s", self.pose.pose.position, car_position.pose.pose.position)

        #TODO find the closest visible traffic light (if one exists)
        # Fined the closet visible traffic light based on current position
        dist_min = float('inf')
        closest_stop_line_idx = -1
        for i, stop_line_pose in enumerate(stop_line_positions):
            if self.is_ahead_of(car_position.pose.pose, stop_line_pose[0], stop_line_pose[1]):
                dist = self.calculate_distance(car_position.pose.pose.position.x, stop_line_pose[0], car_position.pose.pose.position.y, stop_line_pose[1])
                if dist < dist_min:
                    dist_min = dist
                    closest_stop_line_idx = i

	#rospy.loginfo("tl_detector - stop line %s %s", stop_line_positions[closest_stop_line_idx], len(stop_line_positions))

        if closest_stop_line_idx >= 0:
            # Find wp index in waypoints which is closest to the traffic light
            light = self.lights[closest_stop_line_idx]
            dist_min = float('inf')
            for i, wp in enumerate(self.waypoints):
                if self.is_ahead_of(wp.pose.pose, stop_line_positions[closest_stop_line_idx][0], stop_line_positions[closest_stop_line_idx][1]):
                    dist = self.calculate_distance(wp.pose.pose.position.x, stop_line_positions[closest_stop_line_idx][0], wp.pose.pose.position.y, stop_line_positions[closest_stop_line_idx][1])
                    if dist < dist_min:
                        dist_min = dist
                        light_wp = i
                else:
                    break

	    #rospy.loginfo("tl_detector - wp %s light %s", self.waypoints[light_wp].pose.pose.position, light.pose.pose.position)
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
