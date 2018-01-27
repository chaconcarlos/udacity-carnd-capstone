#!/usr/bin/env python

import math
import rospy

from std_msgs.msg      import Bool
from dbw_mkz_msgs.msg  import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from twist_controller  import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

LOOP_RATE           = 50 # 50Hz
DEFAULT_SAMPLE_TIME = 1 / LOOP_RATE 

class Vehicle(object):
  def __init__(self):
    self.vehicle_mass    = None
    self.fuel_capacity   = None
    self.brake_deadband  = None
    self.decel_limit     = None
    self.accel_limit     = None
    self.wheel_radius    = None
    self.wheel_base      = None
    self.steer_ratio     = None
    self.max_lat_accel   = None
    self.max_steer_angle = None
    self.min_speed       = None

class DBWNode(object):
  def __init__(self):
    rospy.init_node('dbw_node', log_level = rospy.DEBUG)

    rospy.loginfo("DBWNode - Initializing drive-by-wire node...")

    rospy.logdebug("DBWNode - Initializing variables...")

    self.initialize_vehicle()

    self.controller         = Controller(self.vehicle)
    self.current_speed      = None
    self.current_yaw        = None
    self.target_speed       = None
    self.target_yaw         = None
    self.dbw_enabled        = False
    self.previous_timestamp = None

    rospy.logdebug("DBWNode - Suscribing to channels...")

    rospy.Subscriber('/current_velocity',    TwistStamped, self.current_velocity_cb)
    rospy.Subscriber('/twist_cmd',           TwistStamped, self.twist_cmd_cb)
    rospy.Subscriber('/vehicle/dbw_enabled', Bool,         self.dbw_enabled_cb)

    rospy.logdebug("DBWNode - Creating required publishers...")

    self.steer_pub    = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size = 1)
    self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size = 1)
    self.brake_pub    = rospy.Publisher('/vehicle/brake_cmd',    BrakeCmd,    queue_size = 1)
    
    rospy.loginfo("DBWNode - Drive-by-wire node initialization finished.")

    self.loop()

  def initialize_vehicle(self):
    self.vehicle                 = Vehicle()
    self.vehicle.vehicle_mass    = rospy.get_param('~vehicle_mass',    1736.35)
    self.vehicle.fuel_capacity   = rospy.get_param('~fuel_capacity',   13.5)
    self.vehicle.brake_deadband  = rospy.get_param('~brake_deadband',  .1)
    self.vehicle.decel_limit     = rospy.get_param('~decel_limit',     -5)
    self.vehicle.accel_limit     = rospy.get_param('~accel_limit',     1.)
    self.vehicle.wheel_radius    = rospy.get_param('~wheel_radius',    0.2413)
    self.vehicle.wheel_base      = rospy.get_param('~wheel_base',      2.8498)
    self.vehicle.steer_ratio     = rospy.get_param('~steer_ratio',     14.8)
    self.vehicle.max_lat_accel   = rospy.get_param('~max_lat_accel',   3.)
    self.vehicle.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

  def current_velocity_cb(self, msg):
    if (self.dbw_enabled):
      rospy.logdebug("DBWNode - Received current velocity message. Linear speed: %s, Angular speed: %s", msg.twist.linear.x, msg.twist.angular.z)

    self.current_speed = msg.twist.linear.x
    self.current_yaw   = msg.twist.angular.z

  def twist_cmd_cb(self, msg):
    if (self.dbw_enabled):
      rospy.logdebug("DBWNode - Received twist command. Target linear speed: %s, Target angular speed: %s", msg.twist.linear.x, msg.twist.angular.z)

    self.target_speed = msg.twist.linear.x
    self.target_yaw   = msg.twist.angular.z

  def dbw_enabled_cb(self, msg):
    self.dbw_enabled = msg.data

  def is_data_complete(self):
    return self.target_speed != None and self.target_yaw != None and self.current_speed != None

  def loop(self):
    rate = rospy.Rate(LOOP_RATE)
    
    while not rospy.is_shutdown():
      sample_time       = None
      current_timestamp = rospy.get_time()
      
      if (self.previous_timestamp == None):
        sample_time = DEFAULT_SAMPLE_TIME
      else:
        sample_time = current_timestamp - self.previous_timestamp
      
      self.previous_timestamp = current_timestamp
      
      if (self.dbw_enabled and self.is_data_complete()):
        rospy.logdebug("DBWNode - Computing controls...")
        throttle, brake, steer = self.controller.get_controls(self.current_speed, self.target_speed, self.target_yaw, sample_time)
        self.publish(throttle, brake, steer)
      elif (self.dbw_enabled == False):
        #rospy.logdebug("DBWNode - DBW disabled. Resetting the controller.") 
        self.controller.reset()
        
      rate.sleep()

  def publish(self, throttle, brake, steer):
    rospy.logdebug("DBWNode - Publishing controls...")
    rospy.logdebug("DBWNode - Throttle: %s, Brake: %s, Steer: %s", throttle, brake, steer)

    tcmd                = ThrottleCmd()
    tcmd.enable         = True
    tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
    tcmd.pedal_cmd      = throttle
    
    self.throttle_pub.publish(tcmd)

    scmd                          = SteeringCmd()
    scmd.enable                   = True
    scmd.steering_wheel_angle_cmd = steer
    
    self.steer_pub.publish(scmd)

    bcmd                = BrakeCmd()
    bcmd.enable         = True
    bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
    bcmd.pedal_cmd      = brake
    
    self.brake_pub.publish(bcmd)

if __name__ == '__main__':
  DBWNode()
