
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle, decel_limit, wheel_radius, vehicle_mass):
        # TODO: Implement

        # Yaw controller attribute
        # __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        # controller coefs
        kp = 0.3
        ki = 0.1
        kd = 0.
        min_throttle = 0.
        max_throttle = 0.2

        # PID controller attribute
        self.throttle_controller = PID(kp, kd, ki, min_throttle, max_throttle)

        tau = 0.5 # 1/(2pi*tau) cutoff freq
        ts = 0.02 # Sample time
        self.vel_lowPassFilter = LowPassFilter(tau, ts)

        self.decel_limit = decel_limit
        self.wheel_radius = wheel_radius
        self.vehicle_mass = vehicle_mass
        
        self.last_time = rospy.get_time()

    def control(self, dbw_enabled, current_velocity, linear_velocity, angular_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0. 
        
        # Filtering out high frequencies
        current_velocity = self.vel_lowPassFilter.filt(current_velocity)

        # Steering controller
        steering = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)
        #print "steerting = ", steering, " - linear_vel = ", linear_velocity, " - angular_vel = ", angular_velocity, " - current_vel = ", current_velocity
        
        # Throttle controller
        velocity_error = linear_velocity - current_velocity # target - current         
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        throttle = self.throttle_controller.step(velocity_error, sample_time)

        # Brake command
        brake = 0

        if linear_velocity == 0. and current_velocity < 0.1:
            throttle = 0
            brake = 400 # N*m
        
        elif throttle < 0.1 and velocity_error < 0: # current_velocity > target_velocity
            throttle = 0
            decel = max(velocity_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius
            

        # DEBUG
        #throttle = 0.3*2
        #brake = 0
        #steering = -0.2
        #rospy.logwarn("Throttle: {0} - Steering: {1} - Brake: {2}".format(throttle, steering, brake))
        
        return throttle, brake, steering
        
        
