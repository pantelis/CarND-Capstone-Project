#!/usr/bin/env python

import sys

# The following lines are specific to Pycharm Remote Debugging configuration (pydev) that allows
# the host os to run the IDE and the Simulator and the remote Ubuntu VM to run ROS.
if sys.platform == "darwin":
    # OSX
    sys.path.append('/home/student/CarND-Capstone-Project/macos/pycharm-debug.egg')
elif sys.platform == "linux" or sys.platform == "linux2":
    # linux
    sys.path.append('')
# elif platform == "win32":
#     # Windows
#     sys.path.append('/home/student/CarND-Capstone-Project/ros/win/pycharm-debug.egg')
import pydevd

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math
import numpy as np
from scipy.spatial import distance

#pydevd.settrace('192.168.1.220', port=6700, stdoutToServer=True, stderrToServer=True)
pydevd.settrace('135.222.156.1', port=6700, stdoutToServer=True, stderrToServer=True)

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):

    def __init__(self):
        rospy.init_node('waypoint_updater')

        # The /base_waypoints topic publishes a list of all waypoints for the track, so this list includes waypoints
        # both before and after the vehicle (note that the publisher for /base_waypoints publishes only once).
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # PoseStamped is a composite type consisting of a Pose type and a Header type
        # that contains a reference coordinate frame and a timestamp.
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        # rospy.Subscriber('/traffic_waypoint', PoseStamped, self.traffic_cb(msg))
        # rospy.Subscriber('/obstacle_waypoints', Lane, self.obstacle_cb(msg))

        # The queue_size=1 argument tells rospy to only buffer a single outbound message. In case the node sending
        # the messages is transmitting at a higher rate than the receiving node(s) can receive them,
        # rospy will simply drop any messages beyond the queue_size.
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.num_waypoints = 0
        self.base_waypoints = []
        self.base_waypoints_position = np.empty
        self.base_waypoints_orientation = np.empty
        self.current_pose_position = np.empty
        self.current_pose_orientation = np.empty

        self.closest_waypoint_index = 0

        # We give control over to ROS by calling rospy.spin().  This function will only return when the node is ready to
        # shutdown. This is just a useful shortcut to avoid having to define a top-level while loop.
        rospy.spin()

    def pose_cb(self, msg):

        # TODO: Implement

        pose_position = msg.pose.position
        pose_orientation = msg.pose.orientation

        self.current_pose_position = np.array([pose_position.x, pose_position.y, pose_position.z])
        self.current_pose_orientation = np.array([pose_orientation.x, pose_orientation.y, pose_orientation.z, pose_orientation.w])

        # rate = rospy.Rate(1)
        # rospy.loginfo(rate, pose)
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement

        # populate the numpy arrays that are needed for calculating the closest waypoint.

        # rospy.loginfo(waypoints)
        # get the number of base waypoints
        self.num_waypoints = len(waypoints.waypoints)
        self.base_waypoints = waypoints

        base_waypoints_position = []
        base_waypoints_orientation = []
        for i in waypoints.waypoints:
            base_waypoints_position.append([i.pose.pose.position.x, i.pose.pose.position.y,
                                                 i.pose.pose.position.z])
        for j in waypoints.waypoints:
            base_waypoints_orientation.append([j.pose.pose.orientation.x, j.pose.pose.orientation.y,
                                                    j.pose.pose.orientation.z, j.pose.pose.orientation.w])

        # convert the list into np arrays - this will allow us to use very efficient KD-Tree algs for
        # estimating the distance
        self.base_waypoints_position = np.asarray(base_waypoints_position)
        self.base_waypoints_orientation = np.asarray(base_waypoints_orientation)

        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def final_waypoints_publisher(self):

        look_ahead_waypoints = Lane()

        closest_waypoint_indx = self.closest_waypoint_index

        for i in range(0, LOOKAHEAD_WPS-1):
            look_ahead_waypoints.waypoints.append(self.base_waypoints[closest_waypoint_indx + i])

        for i in range(0, LOOKAHEAD_WPS-1):
            self.set_waypoint_velocity(look_ahead_waypoints, i, 5.0)

        self.final_waypoints_pub.publish(look_ahead_waypoints)

        pass

    def closest_waypoint_index(self):

        closest_waypoint_indx = distance.cdist(self.current_pose_position, self.base_waypoints_position,
                                                'euclidean', p=2)

        return closest_waypoint_indx

    def get_waypoint_velocity(self, waypoint):

        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):

        waypoints[waypoint].twist.twist.linear.x = velocity

    def ramped_velocity(v_prev, v_target, t_prev, t_now, ramp_rate):

        # compute maximum velocity step
        step = ramp_rate * (t_now - t_prev).to_sec()

        sign = 1.0 if (v_target > v_prev) else -1.0

        error = math.fabs(v_target - v_prev)

        if error < step:  # we can get there within this timestep-we're done.
            return v_target
        else:
            return v_prev + sign * step  # take a step toward the target

    def distance(self, waypoints, wp1, wp2):

        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
