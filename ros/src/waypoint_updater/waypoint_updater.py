#!/usr/bin/env python

import math
import rospy
import tf

from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg     import Lane, Waypoint
from std_msgs.msg      import Int32

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

LOOP_RATE               = 10  # Hz
LOOKAHEAD_WPS           = 100 # 200
DISTANCE_TO_CLOSEST     = 999999999
STOP_DISTANCE           = 45
METERS_PER_KILOMETER    = 1000
SECONDS_PER_HOUR        = 3600
MAX_ACCELERATION        = 0.8
MAX_DECELERATION        = 0.2
NEXT_WAYPOINT_MAX_ANGLE = math.pi / 4

def to_meters_per_second(kilometers_per_hour):
  """
  Converts a value in kilometers per hour to meters per second.

  Args:
      kilometers_per_hour (float): The value in kilometers per hour.

  Returns:
      float: The value in meters per second.
  """
  return (kilometers_per_hour * METERS_PER_KILOMETER) / SECONDS_PER_HOUR

def get_distance(position_1, position_2):
  """
  Gets the distance between two positions.

  Args:
      position_1 (Point): The first point.
      position_2 (Point): The second point.

  Returns:
      The distance between the given positions.
  """
  x_coefficient = (position_1.x - position_2.x) ** 2
  y_coefficient = (position_1.y - position_2.y) ** 2
  z_coefficient = (position_1.z - position_2.z) ** 2

  return math.sqrt(x_coefficient + y_coefficient  + z_coefficient)

def get_distance_from_list(waypoints, index_waypoint1, index_waypoint2):
  """
  Gets the distance between two positions in a waypoint list.

  Args:
      waypoints  (Waypoint): The waypoints list.
      index_waypoint1 (Int): The index of the first waypoint.
      index_waypoint2 (Int): The index of the second waypoint.

  Returns:
      The distance between the given points.
  """
  distance         = 0
  current_waypoint = index_waypoint1

  for i in range(index_waypoint1, index_waypoint2 + 1):
    position1 = waypoints[current_waypoint].pose.pose.position
    position2 = waypoints[i].pose.pose.position
    distance += get_distance(position1, position2)
    current_waypoint = i

  return distance

def set_waypoint_velocity(waypoint, velocity):
  """
  Sets the velocity of a waypoint.

  Args:
      waypoint (Lane): The waypoint to set the velocity to.
      velocity (float): The velocity.
  """
  waypoint.twist.twist.linear.x = velocity

class WaypointUpdater(object):
  """
  Implements the route planning, updating from the current car position and speed,
  obstacles and traffic lights.
  """

  def __init__(self):
    """
    Initializes an instance of the WaypointUploader class.
    """
    rospy.init_node('waypoint_updater')

    rospy.loginfo("WaypointUpdater - Initializing waypoint updater...")
    rospy.logdebug("WaypointUpdater - Suscribing to channels...")

    rospy.Subscriber('/current_pose',      PoseStamped,  self.on_current_pose_received)
    rospy.Subscriber('/current_velocity',  TwistStamped, self.on_velocity_received)
    rospy.Subscriber('/base_waypoints',    Lane,         self.on_base_waypoints_received)
    rospy.Subscriber('/traffic_waypoint',  Int32,        self.on_traffic_waypoint_received)
    rospy.Subscriber('/obstacle_waypoint', Lane,         self.on_obstacle_waypoint_received)

    rospy.logdebug("WaypointUpdater - Creating required publishers...")

    self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size = 1)

    rospy.logdebug("WaypointUpdater - Initializing variables...")

    self.target_velocity     = to_meters_per_second(rospy.get_param('/waypoint_loader/velocity'))
    self.current_pose        = None
    self.current_velocity    = 0
    self.map_waypoints       = None
    self.last_closest_index  = 0
    self.stop_waypoint_index = None

    rospy.logdebug("WaypointUpdater - Target_speed set to %s. m/s", self.target_velocity)
    rospy.loginfo("WaypointUpdater - Waypoint updater initialization finished.")

  def on_current_pose_received(self, msg):
    """
    Handles the event when the current pose of the vehicle has been received.

    Args:
        msg (PoseStamped): The message with the current pose.
    """
    self.current_pose = msg.pose

  def on_velocity_received(self, msg):
    """
    Handles the event when the current velocity of the vehicle has been received.

    Args:
        msg (TwistStamped): The message with the current velocity.
    """
    self.current_velocity = msg.twist.linear.x

  def on_base_waypoints_received(self, lane):
    """
    Handles the event when the base waypoints of the route has been received.

    Args:
        lane (Lane): The message with the set of base waypoints.
    """
    self.map_waypoints       = lane.waypoints
    self.map_waypoints_count = len(self.map_waypoints)

  def on_traffic_waypoint_received(self, msg):
    """
    Handles the event when the a traffic waypoint has been received.

    Args:
        msg (Int32): The index of the traffic waypoint.
    """
    self.stop_waypoint_index = msg.data
    #self.stop_waypoint_index = 800

  def on_obstacle_waypoint_received(self, msg):
    """
    Handles the event when the an obstacle waypoint has been received.

    Args:
        msg (Lane): The message with the information of the obstacle position.
    """
    # TODO: Callback for /obstacle_waypoint message. We will implement it later
    pass

  def get_closest_waypoint_index(self):
    """
    Gets the index of the closest waypoint to the current position.

    Returns:
        The index of the closest waypoint to the current position.
    """
    current_min_distance = DISTANCE_TO_CLOSEST
    closest_index        = self.last_closest_index
    current_position     = self.current_pose.position

    for i in range(self.map_waypoints_count - closest_index):
      index             = self.last_closest_index + i
      waypoint_position = self.map_waypoints[index].pose.pose.position 
      distance          = get_distance(current_position, waypoint_position)

      if (distance < current_min_distance):
        current_min_distance = distance
        closest_index        = index
      else:
        break

    self.last_closest_index = closest_index

    return closest_index

  def get_next_waypoint_index(self):
    """
    Gets the index of the next waypoint to the current position.

    Returns:
        The index of the next waypoint to the current position.
    """
    closest_index = self.get_closest_waypoint_index()

    if (closest_index < self.map_waypoints_count - 1):
      closest_x   = self.map_waypoints[closest_index].pose.pose.position.x
      closest_y   = self.map_waypoints[closest_index].pose.pose.position.y
      current_x   = self.current_pose.position.x
      current_y   = self.current_pose.position.y
      orientation = self.current_pose.orientation

      heading    = math.atan2((closest_y - current_y), (closest_x - current_x))
      quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
      _, _, yaw  = tf.transformations.euler_from_quaternion(quaternion)
      angle      = abs(yaw - heading)

      if (angle > NEXT_WAYPOINT_MAX_ANGLE):
        closest_index += 1
    
    return closest_index

  def get_next_stop_index(self, start_index):  
    """
    Gets the index of the next stop waypoint.

    Args:
        start_index (Int): The index of the starting waypoint.
  
    Returns:
        The index of the next stop waypoint.
    """
    next_stop_index     = self.map_waypoints_count - 1
    is_valid_stop_index = self.stop_waypoint_index != None and self.stop_waypoint_index > 0

    if (is_valid_stop_index and self.stop_waypoint_index < self.map_waypoints_count):
      next_stop_index = self.stop_waypoint_index

    return next_stop_index

  def accelerate(self, waypoints, target_velocity):
    """
    Sets the target velocity of the waypoints accelerating to the target velocity.

    Args:
        waypoints        (List): [out] The waypoints.
        target_velocity (Float): The top speed to accelerate to.
    """
    for i in range(len(waypoints)):
      waypoint_velocity = min(self.current_velocity + (i + 1) * MAX_ACCELERATION, target_velocity)
      set_waypoint_velocity(waypoints[i], waypoint_velocity)

  def decelerate(self, waypoints, next_stop_index):    
    """
    Sets the target velocity of the waypoints decelerating to stop in the last waypoint.

    Args:
        waypoints (List): [out] The waypoints.
    """
    set_waypoint_velocity(waypoints[next_stop_index], 0)

    for index, waypoint in enumerate(waypoints):
      if (index < next_stop_index):
        distance = get_distance_from_list(waypoints, index, next_stop_index) 
        velocity = math.sqrt(2 * MAX_DECELERATION * distance)
      else:
        velocity = 0

      if (velocity < 1.1):
        velocity = 0.0

      set_waypoint_velocity(waypoint, min(velocity, waypoint.twist.twist.linear.x))

  def publish(self):
    """
    Publishes the updated waypoints of the route to follow.
    """
    next_waypoint_index = self.get_next_waypoint_index()
    next_stop_index     = self.get_next_stop_index(next_waypoint_index)
    stop_distance       = get_distance_from_list(self.map_waypoints, next_waypoint_index, next_stop_index)
    future_waypoints    = []

    if (next_stop_index > next_waypoint_index):
      look_ahead_index = min(next_waypoint_index + LOOKAHEAD_WPS, next_stop_index)
      future_waypoints = self.map_waypoints[next_waypoint_index : look_ahead_index]

      if (stop_distance <= STOP_DISTANCE):
        self.decelerate(future_waypoints, len(future_waypoints) - 1)
      else:
        self.accelerate(future_waypoints, self.target_velocity)

    lane                 = Lane()
    lane.header.frame_id = '/world'
    lane.header.stamp    = rospy.Time(0)
    lane.waypoints       = future_waypoints

    self.final_waypoints_pub.publish(lane)

  def start(self):
    """
    Starts the main loop of the waypoint updater.
    """
    rospy.loginfo("WaypointUpdater - Starting waypoint updater node...")

    rate = rospy.Rate(LOOP_RATE)
    
    while not rospy.is_shutdown():
      if (self.map_waypoints != None and self.current_pose != None):
        self.publish()

      rate.sleep()

if __name__ == '__main__':
  try:
    waypoint_updater = WaypointUpdater()
    waypoint_updater.start()
  except rospy.ROSInterruptException:
    rospy.logerr('Could not start waypoint updater node.')
