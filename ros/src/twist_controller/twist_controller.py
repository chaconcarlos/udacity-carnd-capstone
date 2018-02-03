import rospy

from pid            import PID
from lowpass        import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858

class Controller(object):
  """
  Implements the controller that computes the values for 
  throttle, brake and steering.
  """

  def __init__(self, vehicle):
    """
    Initializes an instance of the Controller class.
    """
    self.vehicle        = vehicle
    self.pid_controller = PID(kp = 0.2, ki = 0.005, kd = 0.1, mn = vehicle.decel_limit, mx = vehicle.accel_limit)
    self.steer_lpf      = LowPassFilter(tau = 3, ts = 1)
    self.throttle_lpf   = LowPassFilter(tau = 3, ts = 1)
    self.yaw_controller = YawController(
        wheel_base      = vehicle.wheel_base,
        steer_ratio     = vehicle.steer_ratio,
        min_speed       = vehicle.min_speed,
        max_lat_accel   = vehicle.max_lat_accel,
        max_steer_angle = vehicle.max_steer_angle)
    
    self.vehicle_total_mass = self.vehicle.vehicle_mass + self.vehicle.fuel_capacity * GAS_DENSITY

  def reset(self):
    """
    Resets the controller.
    """
    self.pid_controller.reset()

  def get_brake_torque(self, deceleration_rate):
    """
    Gets the brake torque value for the current situation of the vehicle.
    
    Args:
        deceleration_rate (Float): The deceleration rate.

    Returns:
        The brake torque value.
    """
    return deceleration_rate * self.vehicle_total_mass * self.vehicle.wheel_radius 

  def get_steering(self, current_speed, target_speed, target_yaw):
    """
    Gets the steering control value for the current situation of the vehicle.
    
    Args:
        current_speed (Float): The current speed of the vehicle.
        target_speed  (Float): The target speed for the vehicle.
        target_yaw    (Float): The target yaw for the vehicle.

    Returns:
        The steering control value.
    """
    steer = self.yaw_controller.get_steering(target_speed, target_yaw, current_speed)
    steer = self.steer_lpf.filt(steer)

    return steer
  
  def get_dynamics(self, current_speed, target_speed, sample_time):
    """
    Gets the brake and throttle control values for the current situation of
    the vehicle.
    
    Args:
        current_speed (Float): The current speed of the vehicle.
        target_speed  (Float): The target speed for the vehicle.
        sample_time   (Float): The sample timestamp in linux epoch.

    Returns:
        The throttle and brake control values.
    """
    speed_error  = target_speed - current_speed
    acceleration = self.pid_controller.step(speed_error, sample_time)
    acceleration = self.throttle_lpf.filt(acceleration)
    throttle     = 0
    brake        = 0
    
    if (acceleration > 0):
      throttle = acceleration
    else:
      acceleration = abs(acceleration)
      
      if (acceleration < self.vehicle.brake_deadband):
        acceleration = 0
      
      brake = self.get_brake_torque(acceleration)
    
    return throttle, brake
  
  def get_controls(self, current_speed, target_speed, target_yaw, sample_time):
    """
    Gets the throttle, brake and steer control values for the current situation of
    the vehicle.
    
    Args:
        current_speed (Float): The current speed of the vehicle.
        target_speed  (Float): The target speed for the vehicle.
        target_yaw    (Float): The target yaw for the vehicle.
        sample_time   (Float): The sample timestamp in linux epoch.

    Returns:
        The throttle, brake and steer control values.
    """
    steer = self.get_steering(current_speed, target_speed, target_yaw)

    if (current_speed < 1 and target_speed == 0):
      throttle = 0
      brake    = 2000
    else:
      throttle, brake = self.get_dynamics(current_speed, target_speed, sample_time)
    
    return throttle, brake, steer
