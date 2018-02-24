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
vKP =  0.005 #0.065 #5.0# 0.1 #10
vKI = 0.0005 #0.05 #0.00005# 0.005
vKD = 0.0 #0.0 #0.002# 0.001# 1

#####################################################################################################


class Controller(object):
    # initialize controller
    def __init__(self, **kwargs):
        """
        kwargs = {
            "sample_freq": self.sample_freq,
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

        parameters for simulation
        self.vehicle_mass = 1080
        self.fuel_capacity = 0
        self.brake_deadband = 0.2
        self.wheel_radius = 0.335
        self.wheel_base = 3
        :param kwargs:
        """
        # system parameters
        self.sample_freq = kwargs['sample_freq']

        # vehicle parameters
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.wheel_base = kwargs['wheel_base']
        self.wheel_radius = kwargs['wheel_radius']

        self.simulation = kwargs['simulation']
        if self.simulation:
            self.vehicle_mass = 1080.
            self.fuel_capacity = 0.
            self.brake_deadband = 0.2
            self.wheel_radius = 0.335
            self.wheel_base = 3.

        self.total_mass = self.vehicle_mass + (GAS_DENSITY*self.fuel_capacity)

        # motion params
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_steer_angle = kwargs['max_steer_angle']
        self.max_brake_torque = self.total_mass*self.wheel_radius

        # define PID controller & LP filter for throttle/brake
        self.pid_throttle = PID(vKP, vKI, vKD, self.decel_limit, self.accel_limit)
        self.filter_throttle = LowPassFilter(0.5, 0.5)
        self.filter_brake = LowPassFilter(0.25, 0.75)
        # self.velocity_error = 0.0

        # define steering controller/filter
        self.yaw = YawController(self.wheel_base, self.steer_ratio, 0.0,
                                 self.max_lat_accel, self.max_steer_angle)
        self.filter_steer = LowPassFilter(0.05, 0.95)
        # self.filter_steer = LowPassFilter(0.1, 0.9) # carla



    #####################################################################################################
    def control(self, **kwargs):
        """
        kwargs = {
                "target_linear_velocity": self.target_linear_velocity,
                "target_angular_velocity": self.target_angular_velocity,
                "current_linear_velocity": self.current_linear_velocity,
                "dbw_status": self.dbw_enabled
            }
        """

        # initialize parameters
        throttle, brake, steer = 0.0, 0.0, 0.0

        dbw_status = kwargs["dbw_status"]

        if dbw_status: # if true, compute values and publish

            # load parameters
            target_linear_velocity = kwargs['target_linear_velocity']
            target_angular_velocity = kwargs['target_angular_velocity']
            current_linear_velocity = kwargs['current_linear_velocity']

            # debug incoming data type
            # rospy.logwarn("data type: (L, A, C)" + str(type(target_linear_velocity)) + ", " +
            #               str(type(target_angular_velocity)) + ", " +
            #               str(type(current_linear_velocity)))

            # time step
            dt = 1.0/self.sample_freq

            # reset brake
            brake = 0.0

            # compute steering
            steer = self.yaw.get_steering(target_linear_velocity,
                                                   target_angular_velocity,
                                                   current_linear_velocity)
            steer = self.filter_steer.filt(steer)

            # debug
            # rospy.logwarn("Steering Angle :%.2f", steer)
            # rospy.logwarn("Expected Velocity :%.2f", target_linear_velocity)
            # rospy.logwarn("Angular Velocity :%.2f", target_angular_velocity)

            # compute throttle and brake
            velocity_error = target_linear_velocity - current_linear_velocity
            throttle = self.pid_throttle.step(velocity_error, dt)

            # self.velocity_error = velocity_error

            # use brake if throttle is negative
            if throttle <0.0:
                brake = -throttle
                brake = self.filter_brake.filt(brake)
                throttle = 0.0

                if brake >1:
                    brake = 1

            else: # if throttle is positive
                throttle = self.filter_throttle.filt(throttle)
                # throttle = min(throttle, 1.0)


            # # set limits for simulation and car
            # if self.simulation:
            #     if throttle>0.5:
            #         throttle = 0.5
            #         brake = 0.0
            #     else:
            #         if throttle>0.01:
            #             throttle = 0.01
            #             brake = 0.0

            # rospy.logwarn("----------------------------")
            # rospy.logwarn("throttle: %.4f, brake: %.4f, steer: %.4f", throttle, brake, steer)
            # rospy.logwarn("target-v: %.4f, target-omega: %.6f, current-v: %.4f", target_linear_velocity,
            #               target_angular_velocity, current_linear_velocity)



        else: # if false, manual driving mode is on, reset
            self.pid_throttle.reset()



        # return parameters
        return throttle, brake, steer
