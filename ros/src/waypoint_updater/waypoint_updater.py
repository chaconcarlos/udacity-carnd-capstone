#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg	 import Lane, Waypoint


'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS           = 200 # Number of waypoints we will publish. You can change this number
DISTANCE_TO_CLOSEST     = 100000
NEXT_WAYPOINT_MAX_ANGLE = math.pi / 4
TARGET_SPEED_MPH        = 10

def to_km(miles):
	return (miles * 1609.34) / (60 * 60)

class WaypointUpdater(object):
	def __init__(self):
		rospy.init_node('waypoint_updater')

		rospy.Subscriber('/current_pose',      PoseStamped, self.pose_cb)
		rospy.Subscriber('/base_waypoints',    Lane,		self.waypoints_cb)
		rospy.Subscriber('/traffic_waypoint',  PoseStamped, self.traffic_cb)
		rospy.Subscriber('/obstacle_waypoint', Lane,		self.obstacle_cb)

		self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

		# TODO: Add other member variables you need below

		self.current_pose           = None
		self.waypoints              = None
		self.target_speed           = to_km(TARGET_SPEED_MPH)
		self.is_destination_reached = False

		rospy.spin()

	def pose_cb(self, msg):
		self.current_pose = msg.pose
		publish()
		pass

	def waypoints_cb(self, lane):
		self.waypoints       = lane.waypoints
		self.waypoints_count = len(self.waypoints)
		publish()
		pass

	def traffic_cb(self, msg):
		# TODO: Callback for /traffic_waypoint message. Implement
		pass

	def obstacle_cb(self, msg):
		# TODO: Callback for /obstacle_waypoint message. We will implement it later
		pass

	def get_waypoint_velocity(self, waypoint):
		return waypoint.twist.twist.linear.x

	def set_waypoint_velocity(self, waypoint, velocity):
		waypoint.twist.twist.linear.x = velocity

	def distance(self, waypoints, wp1, wp2):
		dist = 0
		dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
	  
		for i in range(wp1, wp2+1):
			dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
	   		wp1 = i

		return dist

	def distance(self, p1, p2):
		return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2  + (p1.z - p2.z) ** 2)

	def get_closest_waypoint_index(self):
        current_min_distance   = DISTANCE_TO_CLOSEST
        current_waypoint_index = 0
 		current_position       = self.current_pose.position

        for index, waypoint in enumerate(self.waypoints):
            distance = distance(current_position, waypoint.pose.pose.position)

            if (distance < current_min_distance):
                current_min_distance   = distance
                current_waypoint_index = index

	def get_next_waypoint_index(self):
        closest_index = self.get_closest_waypoint_index(pose, waypoints)

        if (closest_index == self.waypoints_count - 1)
        	return None

        closest_x   = waypoints[closest_index].pose.pose.position.x
        closest_y   = waypoints[closest_index].pose.pose.position.y
        current_x   = self.current_pose.position.x
        current_y   = self.current_pose.position.y
        orientation = self.current_pose.orientation

        heading    = math.atan2((closest_y - current_y), (closest_x - current_x))
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        _, _, yaw  = tf.transformations.euler_from_quaternion(quaternion)
        angle      = abs(yaw - heading)

        if angle > NEXT_WAYPOINT_MAX_ANGLE:        
        	closest_index += 1
        
        return closest_index

	def publish(self):
		if self.waypoints = None or self.current_pose = None:
			return

		next_waypoint_index = self.get_next_waypoint_index()

		if (next_waypoint_index == None)
			rospy.loginfo("Destination reached.")
			return

		look_ahead_index = next_waypoint_index + LOOKAHEAD_WPS

		if (look_ahead_index >= self.waypoints_count)
			look_ahead_index = self.waypoints_count - 1

        future_waypoints = self.waypoints[next_waypoint_index : look_ahead_index]

		for i in range(len(future_waypoints)):
			set_waypoint_velocity(future_waypoints[i], self.target_speed)

		lane                 = Lane()
		lane.header.frame_id = '/world'
		lane.header.stamp    = rospy.get_time()
		lane.waypoints       = lookahead_waypoints

		self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
	try:
		WaypointUpdater()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start waypoint updater node.')
