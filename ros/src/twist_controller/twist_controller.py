from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import time
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

#####################################################################################################
# define parameters to tune
# PID gains for velocity control
KP = 3000
KI = 0.00005
KD = 1

# low-pass filter parameters
TAU = 0.5
TS = 0.02
# parameter to avoid division by zero
EPS = 1e-6

#####################################################################################################
############################### main controller class  ##############################################
#####################################################################################################

class Controller(object):
    # initialize controller
    def __init__(self, *args, **kwargs):
        # mass-density / vehicle params
        self.wheel_base = kwargs.get('wheel_base')
        self.wheel_radius = kwargs.get('wheel_radius')
        self.vehicle_mass = kwargs.get('vehicle_mass')
        self.fuel_capacity = kwargs.get('fuel_capacity')
        self.total_mass = self.vehicle_mass + (GAS_DENSITY*self.fuel_capacity)

        # motion params
        self.brake_deadband = kwargs.get('brake_deadband')
        self.decel_limit = kwargs.get('decel_limit')
        self.accel_limit = kwargs.get('accel_limit')
        self.max_lat_accel = kwargs.get('max_lat_accel')
        self.steer_ratio = kwargs.get('steer_ratio')
        self.max_steer_angle = kwargs.get('max_steer_angle')

        # define PID controller & LP filter
        self.pid = PID(KP, KI, KD)
        self.low_pass_filter = LowPassFilter(TAU, TS)

        # define Yaw controller
        self.yaw = YawController(self.wheel_base, self.steer_ratio, ONE_MPH,
                                 self.max_lat_accel, self.max_steer_angle)

        # add initial timestamp
        self.last_time = 0.0

    #####################################################################################################
    def control(self, *args, **kwargs):
        # Return throttle, brake, steer
        if self.last_time==0.0:
            throttle, brake, steer = 1.0, 0.0, 0.0
        else:
            # get parameters
            target_vel = kwargs.get("target_velocity")
            current_vel = kwargs.get("current_velocity")
            target_omega = kwargs.get("target_omega")

            # get delta-time
            current_time = rospy.get_time()
            dt = current_time - self.last_time + EPS

            # speed control (throttle)
            delta_v= target_vel.x - current_vel.x
            throttle = self.pid.step(delta_v, dt)
            throttle = max(0.0, min(1.0,throttle))

            # if negative acceleration apply brakes
            if delta_v < 0:
                throttle = 0# set throttle to zero

                # apply brakes
                decel = abs(delta_v) / dt
                if abs(decel) > abs(self.decel_limit):
                    decel = self.decel_limit

                brake = self.total_mass * decel * self.wheel_radius
                brake = max(0.0, min(1.0,brake))

            else:
                brake = 0

            steer = self.yaw.get_steering(target_vel.x, target_omega.z, current_vel.x)

        self.last_time = rospy.get_time()

        return throttle, brake, steer