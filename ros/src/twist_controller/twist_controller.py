import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

#GAS_DENSITY = 2.858
#ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, vehicle_mass, brake_deadband, decel_limit, accel_limit,
                        wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.brake_deadband = brake_deadband
        
        # pid control for throttle
        kp = 1.0
        ki = 0.1
        kd = 0.
        mn = 0.
        mx = 0.5
        self.pid_ctrl = PID(kp, ki, kd, mn, mx)
        
        self.yaw_ctrl = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        
        tau = 0.314 # 1/(2*pi*f) where f = 50Hz/10
        ts = 1./50.
        self.low_pass = LowPassFilter(tau, ts)
        
        self.last_time = rospy.get_time()

    def control(self, linear_cmd, angular_cmd, curr_vel, dbw_en):
        if not dbw_en:
            self.pid_ctrl.reset()
            return 0., 0., 0.
        
        curr_vel = self.low_pass.filt(curr_vel)
        
        steering = self.yaw_ctrl.get_steering(linear_cmd, angular_cmd, curr_vel)
        
        error = min(linear_cmd - curr_vel, self.accel_limit*1.)
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        throttle = self.pid_ctrl.step(error, sample_time)
        
        brake = 0
        
        if error < 0 and throttle < .1: # release throttle and brake
            throttle = 0
            if error > -2.5:
                decel = max(self.decel_limit, error/2.)
            else:
                decel = max(self.decel_limit, error/1.)
            brake = self.vehicle_mass * self.wheel_radius * abs(decel)
            brake = 0 if brake < self.brake_deadband else brake
        elif linear_cmd == 0 and curr_vel < .1:
            throttle = 0
            brake = 700
        
        return throttle, brake, steering
