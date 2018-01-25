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

    def reset(self):
    	self.pid_controller.reset()

    def get_controls(self, current_speed, target_speed, target_yaw):
	    # Return throttle, brake, steer
	    speed_error = target_speed - current_speed
	   	steer       = self.yaw_controller.get_steering(target_speed, target_yaw, current_speed)
	   	steer       = self.steer_lpf.filt(steer)

        return 1., 0., 0.
