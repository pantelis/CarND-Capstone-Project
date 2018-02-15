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


pydevd.settrace('192.168.1.224', port=6700, stdoutToServer=True, stderrToServer=True)


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

        # Other member variables we need below
        self.current_pose_position = np.zeros([1, 3])
        self.current_pose_orientation = np.zeros([1, 4])


        # We give control over to ROS by calling rospy.spin().  This function will only return when the node is ready to
        # shutdown. This is just a useful shortcut to avoid having to define a top-level while loop.
        rospy.spin()

    def pose_cb(self, msg):
        '''Current pose callback'''

        #rospy.loginfo(msg)

        pose_position = msg.pose.position
        pose_orientation = msg.pose.orientation

        self.current_pose_position = np.array([[pose_position.x, pose_position.y, pose_position.z]])
        self.current_pose_orientation = np.array([[pose_orientation.x, pose_orientation.y, pose_orientation.z, pose_orientation.w]])


        pass

    def waypoints_cb(self, waypoints):
        '''Base waypoints callback'''

        # rospy.loginfo(waypoints.waypoints)

        # get the number of base waypoints
        self.num_waypoints = len(waypoints.waypoints)

        self.base_waypoints = waypoints
        self.base_waypoints_position = np.zeros([self.num_waypoints, 3])
        self.base_waypoints_orientation = np.zeros([self.num_waypoints, 4])


        k=0
        for wp in waypoints.waypoints:
            self.base_waypoints_position[k] = [wp.pose.pose.position.x, wp.pose.pose.position.y, wp.pose.pose.position.z]
            k += 1

        k=0
        for wp in waypoints.waypoints:
            self.base_waypoints_orientation[k] =[wp.pose.pose.orientation.x, wp.pose.pose.orientation.y,
                                                    wp.pose.pose.orientation.z, wp.pose.pose.orientation.w]
            k += 1

        # calculate and publish the final waypoints
        self.final_waypoints_publisher(waypoints)

        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def final_waypoints_publisher(self, waypoints):

        closest_waypoint_indx = distance.cdist(XA=self.current_pose_position, XB=self.base_waypoints_position,
                                               metric='euclidean', p=2).argmin()

        # create the look_ahead waypoints anc consider the wrap around in closed tracks
        self.look_ahead_waypoints = Lane()

        for i in range(0, LOOKAHEAD_WPS):
            self.look_ahead_waypoints.waypoints.append(
                self.base_waypoints.waypoints[(closest_waypoint_indx + i) % self.num_waypoints])

        for i in range(0, LOOKAHEAD_WPS):
            self.set_waypoint_velocity(self.look_ahead_waypoints.waypoints, i, 5.0)

        rospy.loginfo(self.look_ahead_waypoints)

        self.final_waypoints_pub.publish(self.look_ahead_waypoints)

        pass


    def get_waypoint_velocity(self, waypoint):

        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):

        waypoints[waypoint].twist.twist.linear.x = velocity

    pass

    def ramped_velocity(v_prev, v_target, t_prev, t_now, ramp_rate):

        # compute maximum velocity step
        step = ramp_rate * (t_now - t_prev).to_sec()

        sign = 1.0 if (v_target > v_prev) else -1.0

        error = math.fabs(v_target - v_prev)

        if error < step:  # we can get there within this timestep-we're done.
            return v_target
        else:
            return v_prev + sign * step  # take a step toward the target

    pass

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
