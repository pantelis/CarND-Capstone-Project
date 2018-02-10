#!/usr/bin/env python

import sys

sys.path.append('/home/student/CarND-Capstone-Project/ros/pycharm-debug.egg')

import pydevd
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math

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

        # PoseStamped is a composite type consisting of a Pose type and a Header type
        # that contains a reference coordinate frame and a timestamp.
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # rospy.Subscriber('/traffic_waypoint', PoseStamped, self.traffic_cb(msg))
        # rospy.Subscriber('/obstacle_waypoints', Lane, self.obstacle_cb(msg))

        # The queue_size=1 argument tells rospy to only buffer a single outbound message. In case the node sending
        # the messages is transmitting at a higher rate than the receiving node(s) can receive them,
        # rospy will simply drop any messages beyond the queue_size.
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        # We give control over to ROS by calling rospy.spin().  This function will only return when the node is ready to
        # shutdown. This is just a useful shortcut to avoid having to define a top-level while loop.
        rospy.spin()

    def pose_cb(self, msg):

        # TODO: Implement
        pydevd.settrace('135.222.156.85', port=6700, stdoutToServer=True, stderrToServer=True)
        pose = msg.msg
        rospy.loginfo("Pose message Rx = ", pose)
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement

        # should this be a structure that consists of numpy arrays ?
        # base_waypoints = waypoints.waypoints
        rospy.loginfo("Waypoints message Rx = ", waypoints)

        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

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
