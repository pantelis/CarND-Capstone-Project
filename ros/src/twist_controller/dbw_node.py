#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''


class DBWNode(object):

    # ========================================================================================
    # initialize
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5.)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        # publishers
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        # initialize parameters
        self.current_linear_velocity = 0.0
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.dbw_enabled = False
        sample_freq = 50.0 # 50Hz frequency for Carla

        # TODO: Create `Controller` object
        # initialize control object
        kwargs = {
            "sample_freq": sample_freq,
            "vehicle_mass": vehicle_mass,
            "fuel_capacity": fuel_capacity,
            "brake_deadband": brake_deadband,
            "decel_limit": decel_limit,
            "accel_limit": accel_limit,
            "wheel_radius": wheel_radius,
            "wheel_base": wheel_base,
            "steer_ratio": steer_ratio,
            "max_lat_accel": max_lat_accel,
            "max_steer_angle": max_steer_angle,
        }
        # initialize controller
        self.controller = Controller(**kwargs)

        # TODO: Subscribe to all the topics you need to
        # subscribe to current velocity parameters
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)
        # subscribe to target velocity parameters (estimated from waypoints)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size=1)
        # subscribe to DBW enable state (enable==autonomous mode, else manual driving)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1)

        self.loop(sample_freq)

    # ========================================================================================
    # define control loop
    def loop(self, sample_freq):
        rate = rospy.Rate(sample_freq) # 50Hz for Carla

        # TODO: Get predicted throttle, brake, and steering using `twist_controller`

        while not rospy.is_shutdown():
            kwargs = {
                "target_linear_velocity": self.target_linear_velocity,
                "target_angular_velocity": self.target_angular_velocity,
                "current_linear_velocity": self.current_linear_velocity,
                "dbw_status": self.dbw_enabled
            }
            # compute throttle, brake, steering
            throttle, brake, steering = self.controller.control(**kwargs)
            # publish only if in autonomous mode
            if self.dbw_enabled:
                self.publish(throttle, brake, steering)

            rate.sleep()

    # ========================================================================================
    # publisher function
    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    # ========================================================================================
    #  define callback functions
    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = bool(msg.data)

        #==== for debugging ====
        # if self.dbw_enabled:
        #     rospy.logwarn("Controller ENABLED")
        # else:
        #     rospy.logwarn("Controller DISABLED")
        #     self.controller.pid_reset()

    def current_velocity_cb(self, msg):
        # current linear velocity from vehicle
        self.current_linear_velocity = msg.twist.linear.x

    def twist_cmd_cb(self, msg):
        # linear speed in vehicle coordinates
        self.target_linear_velocity = msg.twist.linear.x
        # Yaw (angle about z-axis)
        self.target_angular_velocity = msg.twist.angular.z

        # ==== for debugging ====
        # rospy.logwarn('angular-velocity: ' + str(msg.twist.angular.z))
        # rospy.logwarn("Linear Velocity: %.2f, Angular Velocity: %2f",
        #               self.target_linear_velocity, self.target_angular_velocity)

# ========================================================================================
#  main function call
if __name__ == '__main__':
    DBWNode()