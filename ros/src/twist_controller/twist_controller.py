from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# ========================================================================================
# define parameters to tune
# PID gains for velocity control
vKP =  0.85 # 0.065 #0.005 #0.065 #5.0# 0.1 #10
vKI = 0.05 # 0.00005 #0.05 #0.00005# 0.005
vKD = 0.0 #0.2 # 0.0002 #0.0 #0.002# 0.001# 1

THROTTLE_LPF_TAU = 0.5
THROTTLE_LPF_TS = 0.5

BRAKE_LPF_TAU = 0.25
BRAKE_LPF_TS = 0.75

STEER_LPF_TAU = 0.05
STEER_LPF_TS = 0.95

# ========================================================================================


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

        :param kwargs:
        """
        # system parameters
        self.sample_freq = kwargs['sample_freq']

        # vehicle parameters
        vehicle_mass = kwargs['vehicle_mass']
        fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        wheel_base = kwargs['wheel_base']
        wheel_radius = kwargs['wheel_radius']
        
        # motion params
        decel_limit = kwargs['decel_limit']
        accel_limit = kwargs['accel_limit']
        max_lat_accel = kwargs['max_lat_accel']
        steer_ratio = kwargs['steer_ratio']
        max_steer_angle = kwargs['max_steer_angle']
        total_mass = vehicle_mass + (GAS_DENSITY * fuel_capacity)
        self.max_brake_torque = total_mass * wheel_radius

        # define PID controller & LP filter for throttle/brake
        self.pid_throttle = PID(vKP, vKI, vKD, decel_limit, accel_limit)
        self.filter_throttle = LowPassFilter(THROTTLE_LPF_TAU, THROTTLE_LPF_TS)
        self.filter_brake = LowPassFilter(BRAKE_LPF_TAU, BRAKE_LPF_TS)

        # define steering controller/filter
        self.yaw = YawController(wheel_base, steer_ratio, 0.0,
                                 max_lat_accel, max_steer_angle)
        self.filter_steer = LowPassFilter(STEER_LPF_TAU, STEER_LPF_TS)

    # ========================================================================================
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

            # time step
            dt = 1.0/self.sample_freq

            # reset brake
            brake = 0.0

            # compute steering
            steer = self.yaw.get_steering(target_linear_velocity,
                                                   target_angular_velocity,
                                                   current_linear_velocity)
            steer = self.filter_steer.filt(steer)

            # compute throttle and brake
            velocity_error = target_linear_velocity - current_linear_velocity
            throttle = self.pid_throttle.step(velocity_error, dt)

            # use brake if throttle is negative
            if throttle <0.0:
                brake = -throttle
                brake = self.filter_brake.filt(brake)
                throttle = 0.0

                if brake >1: # this is optional, need to test
                    brake = min(90, brake * self.max_brake_torque)
                    # brake = self.filter_brake.filt(brake)

            else: # if throttle is positive
                throttle = self.filter_throttle.filt(throttle)
                # throttle = min(throttle, 1.0)

            #======== for debugging ==============
            # rospy.logwarn("----------------------------")
            # rospy.logwarn("throttle: %.4f, brake: %.4f, steer: %.4f", throttle, brake, steer)
            # rospy.logwarn("target-v: %.4f, target-omega: %.6f, current-v: %.4f", target_linear_velocity,
            #               target_angular_velocity, current_linear_velocity)


        else: # if false, manual driving mode is on, reset
            self.pid_throttle.reset()

        # return parameters
        return throttle, brake, steer