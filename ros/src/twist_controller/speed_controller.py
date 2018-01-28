from pid import PID


class Speed_Controller(object):
    def __init__(self, *args, **kwargs):

        self.throttle_pid = PID(0.2, 0.005, 0.1)

        self.mass = None
        self.fuel_capacity = None
        self.fuel_density = None
        self.total_mass = None
        self.wheel_radius = None
        self.max_deceleration = None
        self.max_brake_torque = None
        self.rate = None
        self.dt = None

    def setup(self, rate, m, fuel_capacity, density, wheel_radius, max_deceleration):
        self.rate = rate
        self.dt = 1.0 / rate
        self.mass = m
        self.fuel_capacity = fuel_capacity
        self.fuel_density = density
        self.total_mass = (m + fuel_capacity * density)
        self.wheel_radius = wheel_radius

        self.max_deceleration = max_deceleration

        self.max_brake_torque = self.calculate_brake(self.total_mass, self.max_deceleration, wheel_radius)

    def calculate_brake(self, mass, acc, wheel_radius):
        return abs(mass * acc * wheel_radius)

    def control(self, current_speed, target_speed):
        speed_error = target_speed - current_speed

        throttle = 0
        brake = 0
        if (speed_error >= 0):
            throttle = self.throttle_pid.step(speed_error, self.dt)
            throttle = self.clamp(throttle, 0.0, 0.75)
        else:
            acc = speed_error / self.dt
            brake = self.calculate_brake(self.total_mass, acc, self.wheel_radius)
            brake = self.clamp(brake, 0, self.max_brake_torque)
            self.throttle_pid.reset()

        return throttle, brake

    def clamp(self, value, min_value, max_value):
        if value > max_value:
            return max_value
        elif value < min_value:
            return min_value
        return value

    def reset(self):
        self.throttle_pid.reset()