import rospy

from pid            import PID
from lowpass        import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH     = 0.44704

class Controller(object):
  def __init__(self, vehicle):
    self.vehicle        = vehicle
    self.pid_controller = PID(kp = 5, ki = 0.5, kd = 0.5, mn = vehicle.decel_limit, mx = vehicle.accel_limit)
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
    self.pid_controller.reset()

  def get_brake_torque(self, acceleration):
    return acceleration * self.vehicle_total_mass * self.vehicle.wheel_radius 

  def get_steering(self, target_speed, target_yaw, current_speed):
    steer = self.yaw_controller.get_steering(target_speed, target_yaw, current_speed)
    steer = self.steer_lpf.filt(steer)
    return steer
  
  def get_dynamics(self, current_speed, target_speed, sample_time):
    acceleration = self.pid_controller.step(speed_error, sample_time)
    acceleration = self.t_lpf.filt(acceleration)
    throttle     = 0
    brake        = 0
    
    if (acceleration > 0):
      throttle = acceleration
    else:
      acceleration = abs(acceleration)
      
      if (acceleration < self.vehicle.brake_deadband):
        acceleration = 0
      
      brake = get_brake_torque(acceleration)
    
    return throttle, brake
  
  def get_controls(self, current_speed, target_speed, target_yaw, sample_time):
    # Return throttle, brake, steer
    speed_error     = target_speed - current_speed
    steer           = self.get_steering(target_speed, target_yaw, current_speed)
    throttle, brake = self.get_dynamics(current_speed, target_speed, sample_time)
    
    return throttle, steer, brake
