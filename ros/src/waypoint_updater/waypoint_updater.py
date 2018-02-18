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

pydevd.settrace('192.168.1.224', port=6700, stdoutToServer=True, stderrToServer=True)

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
# note: we import TrafficLightArray for testing purposes only
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightArray
import math
import numpy as np
from scipy.spatial import distance


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

# GLOBAL VARIABLES
LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
TRAFFIC_LIGHT_VEHICLE_DISTANCE = 1  # Distance between the vehicle when it comes to a stop and the traffic light.


class WaypointUpdater(object):

    def __init__(self):
        """
        Waypoint updated initialization.
        """
        rospy.init_node('waypoint_updater')

        # Subscribers
        # The /base_waypoints topic publishes a list of all waypoints for the track, so this list includes waypoints
        # both before and after the vehicle (note that the publisher for /base_waypoints publishes only once).
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # PoseStamped is a composite type consisting of a Pose type and a Header type
        # that contains a reference coordinate frame and a timestamp.
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # Note this subscriber is for testing the waypoint updater using ideal traffic light detection.
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.ideal_traffic_light_detection_cb)

        rospy.Subscriber('/obstacle_waypoints', Lane, self.obstacle_cb)

        # Publisher
        # Publish a list of waypoints ahead of the car with target velocities to the /final_waypoints topic.
        # The queue_size=1 argument tells rospy to only buffer a single outbound message. In case the node sending
        # the messages is transmitting at a higher rate than the receiving node(s) can receive them,
        # rospy will simply drop any messages beyond the queue_size.
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Pose member variables
        self.current_pose_position = np.zeros([1, 3])
        self.current_pose_orientation = np.zeros([1, 4])

        # Waypoint member variables
        self.num_waypoints = 0
        self.base_waypoints = Waypoint()
        self.base_waypoints_position = np.array
        self.base_waypoints_orientation = np.array
        self.closest_waypoint_indx = 0
        self.look_ahead_waypoints_msg = Lane()

        # Traffic Light member variables
        self.num_traffic_lights = 0
        self.distance_to_next_traffic_light = 0
        self.next_traffic_light_waypoint_index = 0
        self.next_traffic_light_position = np.zeros([1, 3])  # init to type
        self.ideal_next_traffic_light_state = 4  # Unknown state as defined in the TrafficLight.msg

        # Main control loop
        self.main_control_loop()

        # We give control over to ROS. rospy.spin() will only return when the node is ready to
        # shutdown. This way we avoid having to define a top-level while loop.
        rospy.spin()

    def main_control_loop(self):
        """
        Main control loop that contains all functions that must be constantly executed with a certain rate (Hz).
        """
        # The Rate instance will attempt to keep the loop at 10 Hz by accounting for the time used by any
        # operations during the loop.
        rate = rospy.Rate(10)  # in Hz

        # State transition and trajectory generation

        # Publish the selected trajectory
        self.final_waypoints_publisher()

        rospy.spin()

        pass


    def pose_cb(self, msg):
        """
        Current pose callback
        :param StampedPose msg: message containing the current pose of the vehicle.
        """
        #rospy.loginfo(msg)

        pose_position = msg.pose.position
        pose_orientation = msg.pose.orientation

        self.current_pose_position = np.array([[pose_position.x, pose_position.y, pose_position.z]])
        self.current_pose_orientation = np.array([[pose_orientation.x, pose_orientation.y, pose_orientation.z, pose_orientation.w]])

        pass

    def waypoints_cb(self, waypoints):
        """
        Waypoints callback.
        :param Lane waypoints: message containing the waypoints of the track that the car must follow.
        """
        # rospy.loginfo(waypoints.waypoints)

        # get the number of base waypoints
        self.num_waypoints = len(waypoints.waypoints)

        self.base_waypoints = waypoints
        self.base_waypoints_position = np.zeros([self.num_waypoints, 3])
        self.base_waypoints_orientation = np.zeros([self.num_waypoints, 4])


        k = 0
        for wp in waypoints.waypoints:
            self.base_waypoints_position[k] = [wp.pose.pose.position.x, wp.pose.pose.position.y, wp.pose.pose.position.z]
            k += 1

        k = 0
        for wp in waypoints.waypoints:
            self.base_waypoints_orientation[k] =[wp.pose.pose.orientation.x, wp.pose.pose.orientation.y,
                                                    wp.pose.pose.orientation.z, wp.pose.pose.orientation.w]
            k += 1

        pass

    def ideal_traffic_light_detection_cb(self, msg):
        """
        Sets the variables that determine the position and state of the next traffic light
        as well as its distance from the vehicle.
        :param msg: message of custom type TrafficLightArray published by the vehicle with the ideal light position and state.
        """
        # rospy.loginfo(msg)

        self.num_traffic_lights = len(msg.lights)

        # assuming that the lights are ** in order ** all we need to do is to check the traffic light position
        # with respect to the current pose and produce the next wrapped traffic light index.

        for i in msg.lights:
            self.ideal_next_traffic_light_position = np.array([[i.pose.pose.position.x, i.pose.pose.position.y, i.pose.pose.position.z]])

            # if the distance is less then 1m - effectively when the car just passed the light
            if (distance.cdist(self.current_pose_position,
                               self.ideal_next_traffic_light_position, 'euclidean', p=2) < 1.):

                light_index = ((i+1) % self.num_traffic_lights)
                self.ideal_next_traffic_light_position = np.array([[light_index.pose.pose.position.x, light_index.pose.pose.position.y, light_index.pose.pose.position.z]])
                self.ideal_next_traffic_light_state = msg.lights[light_index].state

        pass

    def traffic_cb(self, traffic_light_msg):
        """
        The callback for processing the traffic_waypoint message. If for example 12 is published in /traffic_waypoint topic,
        an upcoming red light's stop line is nearest to base_waypoints[12], This index is used by the waypoint updater node
        to set the target velocity for look_ahead_waypoints_msg.waypoints[12] to 0 and smoothly decrease the vehicle velocity
        in the waypoints leading up to look_ahead_waypoints_msg.waypoints[12].

        :param Int32 traffic_light_msg: indicates the waypoint index where the **red** traffic light was detected by the tl_detector module.
        """

        self.next_traffic_light_waypoint_index = traffic_light_msg

        self.distance_to_next_traffic_light = self.distance(self.base_waypoints_position, self.closest_waypoint_indx,
                                                            self.next_traffic_light_waypoint_index)

        pass

    def obstacle_cb(self, msg):
        """
        :param msg:
        """
        # TODO: Callback for /obstacle_waypoint message. We will implement it later

    pass

    def final_waypoints_publisher(self):

        """
        Publishes final waypoints.
        :param waypoints:
        """

        self.closest_waypoint_indx = distance.cdist(XA=self.current_pose_position, XB=self.base_waypoints_position,
                                               metric='euclidean', p=2).argmin()

        # create the look_ahead waypoints considering the wrap around in closed tracks
        self.look_ahead_waypoints_msg = Lane()
        for i in range(0, LOOKAHEAD_WPS):
            self.look_ahead_waypoints_msg.waypoints.append(
                self.base_waypoints.waypoints[(self.closest_waypoint_indx + i) % self.num_waypoints])


        # message header
        self.set_look_ahead_waypoints_msg_header(self.look_ahead_waypoints_msg, '/World', rospy.Time.now())

        # message inear velocity (x, y)
        for i in range(0, LOOKAHEAD_WPS):
            self.set_look_ahead_waypoints_msg_velocity(self.look_ahead_waypoints_msg.waypoints, i, 5.0)


        rospy.loginfo(self.look_ahead_waypoints_msg)

        # publish waypoints
        self.final_waypoints_pub.publish(self.look_ahead_waypoints_msg)

        pass

    def set_look_ahead_waypoints_msg_header(self, look_ahead_waypoints_msg, frame_id, time_stamp):

        look_ahead_waypoints_msg.header.frame_id = frame_id
        look_ahead_waypoints_msg.header.stamp = time_stamp

    pass

    def get_waypoint_velocity(self, waypoint):

        return waypoint.twist.twist.linear.x

    pass

    def set_look_ahead_waypoints_msg_velocity(self, look_ahead_waypoints_msg, wp, velocity_mps):

        look_ahead_waypoints_msg[wp].twist.twist.linear.x = velocity_mps

    pass

    def ramped_velocity(self, v_prev, v_target, t_prev, t_now, ramp_rate):

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

    pass

    def deg2rad(self, deg):

        return deg * math.pi / 180.
    pass

    def rad2deg(self, rad):
        return rad * 180. / math.pi
    pass

    # mph to meters per sec
    def mph2mps(self, miles_per_hour):

        return miles_per_hour * 0.44704
    pass

    # meter per sec to mph
    def mps2mph(self, meters_per_second):

        return meters_per_second * 2.23694
    pass


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
