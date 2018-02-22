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
vKP = 0.5# 0.1 #10
vKI = 0.0005 #0.00005# 0.005
vKD = 0.01 #0.002# 0.001# 1

# PID gains for steering control
sKP = 1000#10.0
sKI = 0.00005#0.05
sKD = 0.005#0.2

# low-pass filter parameters
TAU = 0.5
TS = 0.02

#####################################################################################################
############################### main controller class  ##############################################
#####################################################################################################

class Controller(object):
    # initialize controller
    def __init__(self, **kwargs):
        """
        kwargs = {
            "vehicle_mass": vehicle_mass,
            "fuel_capacity": fuel_capacity,
            "brake_deadband": brake_deadband,
            "decel_limit": decel_limit,
            "accel_limit": accel_limit,
            "wheel_radius": wheel_radius,
            "wheel_base": wheel_base,
            "steer_ratio": steer_ratio,
            "max_lat_accel": max_lat_accel,
            "max_steer_angle": max_steer_angle
        }
        :param kwargs:
        """
        # mass-density / vehicle params
        self.sample_freq = kwargs['sample_freq']
        self.wheel_base = kwargs['wheel_base']
        self.wheel_radius = kwargs['wheel_radius']
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.total_mass = self.vehicle_mass + (GAS_DENSITY*self.fuel_capacity)

        # motion params
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_steer_angle = kwargs['max_steer_angle']
        self.max_brake_torque = self.total_mass*self.wheel_radius

        # define PID controller & LP filter
        self.pid_throttle = PID(vKP, vKI, vKD, self.decel_limit, self.accel_limit)
        self.pid_steering = PID(sKP, sKI, sKD, -self.max_steer_angle/2, self.max_steer_angle /2)
        # self.low_pass_filter = LowPassFilter(TAU, TS)

        # define Yaw controller
        self.yaw = YawController(self.wheel_base, self.steer_ratio, ONE_MPH,
                                 self.max_lat_accel, self.max_steer_angle)

        # add initial timestamp
        # self.last_time = 0.0

    #####################################################################################################
    # define reset function
    def pid_reset(self):
        self.pid_throttle.reset()
        self.pid_steering.reset()

    #####################################################################################################
    def control(self, **kwargs):
        """
        kwargs = {
                    "target_linear_velocity": self.target_linear_velocity,
                    "target_angular_velocity": self.target_angular_velocity,
                    "current_linear_velocity": self.current_linear_velocity,
                    "steer_angle": self.steer_angle
        }
        """

        target_linear_velocity = kwargs['target_linear_velocity']
        target_angular_velocity = kwargs['target_angular_velocity']
        current_linear_velocity = kwargs['current_linear_velocity']
        # steer_feedback = kwargs['steer_feedback']

        # initialize parameters
        throttle, brake, steer = 0.0, 0.0, 0.0

        # time increment
        dt = 1./self.sample_freq

        # velocity error
        delta_v = target_linear_velocity - current_linear_velocity

        # desired acceleration
        acceleration = self.pid_throttle.step(delta_v, dt)

        if acceleration > 0.0:
            brake = 0.0
            throttle = acceleration
        else:
            throttle = 0
            if abs(acceleration) > self.brake_deadband:
                brake = abs(acceleration) * self.max_brake_torque

        steer = self.yaw.get_steering(target_linear_velocity,
                                         target_angular_velocity,
                                         current_linear_velocity)
        # steer = self.pid_steering.step(steer - steer_feedback, dt)

        # for debugging
        rospy.logwarn("------------------------------")
        rospy.logwarn("target, current-velocity: " + str(target_linear_velocity) + ", " + str(current_linear_velocity))
        rospy.logwarn("vals: T: " + str(throttle) + ", B: " + str(brake) + ", S: " + str(steer))

        return throttle, brake, steer






















################ old logic
        # if self.last_time == 0.0:
        #     throttle, brake, steer = 0.0, 0.0, 0.0
        #
        # else:
        #     # get parameters
        #     target_vel_x = kwargs["target_velocity_x"]
        #     current_vel_x = kwargs["current_velocity_x"]
        #     target_omega_z = kwargs["target_omega_z"]
        #     # steer_angle = kwargs.get("steer_angle")
        #
        #     if target_vel_x == 0 and abs(current_vel_x) < 0.01:
        #         self.pid_throttle.reset()
        #
        #     else:
        #
        #         # velocity error
        #         delta_v = target_vel_x - current_vel_x
        #         # current time
        #         current_time = rospy.get_time()
        #         # get delta-time
        #         dt = current_time - self.last_time
        #         # dt = 1./50.
        #
        #         # compute steering
        #         steer = self.yaw.get_steering(target_vel_x, target_omega_z, current_vel_x)
        #         # steer = self.low_pass_filter(steer)
        #
        #         #acceleration
        #         acceleration = self.pid_throttle.step(delta_v, dt)
        #
        #         throttle, brake, = 0.0, 0.0
        #         if target_vel_x<ONE_MPH/2: # vehicle needs to slow down
        #             brake = self.decel_limit * self.max_brake_torque
        #         else:
        #             if acceleration<0.0: # need to apply brake
        #                 deceleration = -acceleration
        #                 if deceleration < self.brake_deadband:
        #                     brake = 0.0
        #                 else:
        #                     brake = deceleration * self.max_brake_torque
        #
        #             else:
        #                 throttle = acceleration
        #
        #
        # self.last_time = rospy.get_time()
        #
        # return throttle, brake, steer