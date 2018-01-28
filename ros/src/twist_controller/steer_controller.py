from yaw_controller import YawController
from lowpass import LowPassFilter

# Class representing the steering control for a SDC
class Steer_Controller(object):
    def __init__(self, *args, **kwargs):
        self.yaw_controller = None
        self.wheel_base = None
        self.steer_ratio = None
        self.max_lat_accel = None
        self.min_speed = None
        self.max_steer_angle = None
        self.lp_fiter = LowPassFilter(3,1)

    # Method to set up necessary values for the steering controller
    # Defines the Yaw controller that calculates steering angle
    def setup(self, wheel_base, steer_ratio, max_lat_accel, min_speed,
              max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        self.min_speed = min_speed
        self.yaw_controller = YawController(wheel_base=wheel_base,
                                            steer_ratio=steer_ratio,
                                            max_lat_accel=max_lat_accel,
                                            min_speed=min_speed,
                                            max_steer_angle=max_steer_angle)

    # Method to calculate the new steering angle based on target speed
    # and angular velocity, as well as current speed.
    # Utilizes the yaw controller helper
    def control(self, linear_velocity, angular_velocity, current_velocity):
        steer = self.yaw_controller.get_steering(linear_velocity,
                                                 angular_velocity,
                                                 current_velocity)
        steer = self.lp_fiter.filt(steer)
        return steer

    # Method to limit a value between min and max arguments
    def clamp(self, value, min_value, max_value):
		if value > max_value:
			return max_value
		elif value < min_value:
			return min_value
		return value