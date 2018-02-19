#!/usr/bin/env python

# Uncomment lines 3-19 for debugging
# import sys
#
# # The following lines are specific to Pycharm Remote Debugging configuration (pydev) that allows
# # the host os to run the IDE and the Simulator and the remote Ubuntu VM to run ROS.
# if sys.platform == "darwin":
#     # OSX
#     sys.path.append('/home/student/CarND-Capstone-Project/macos/pycharm-debug.egg')
# elif sys.platform == "linux" or sys.platform == "linux2":
#     # linux
#     sys.path.append('')
# # elif platform == "win32":
# #     # Windows
# #     sys.path.append('/home/student/CarND-Capstone-Project/ros/win/pycharm-debug.egg')
# import pydevd
#
# pydevd.settrace('192.168.1.224', port=6700, stdoutToServer=True, stderrToServer=True)

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
IDEAL_LIGHT_DETECTION = True  # Turns on or off the ideal traffic light detection.


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

        if IDEAL_LIGHT_DETECTION:
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
        self.current_pose_topic_msg = PoseStamped()
        self.current_pose_position = np.zeros([1, 3])
        self.current_pose_orientation = np.zeros([1, 4])

        # Waypoint member variables
        self.base_waypoints_topic_msg = Lane()
        self.num_waypoints = 0
        self.base_waypoints_position = np.array
        self.base_waypoints_orientation = np.array
        self.closest_to_car_waypoint_indx = 0
        self.final_waypoints_topic_msg = Lane()

        # Traffic Light member variables
        self.traffic_waypoint_topic_msg = Int32
        self.vehicle_traffic_lights_topic_msg = TrafficLightArray()
        self.num_traffic_lights = 0
        self.distance_to_next_traffic_light = 0
        self.next_traffic_light_waypoint_index = int
        self.next_traffic_light_position = np.zeros([1, 3])  # init to type
        self.next_traffic_light_state = 4  # Unknown state as defined in the TrafficLight.msg In real life since the
        # traffic light detector publishes in the /traffic_light topic only when the next_traffic_light_state = 0
        # (a red light), the state is not used.  Its included here as a variable for completeness and for later
        # enhancements (testing acceleration when the state is orange :) ).

        # Vehicle States
        self.vehicle_states = ['DECELERATING', 'STOPPED', 'ACCELERATING', 'CRUISING']
        self.current_state = 'STOPPED'

        # Trajectory parameters
        self.acceleration_limit = 0
        self.deceleration_limit = 0
        self.velocity_target = 0

        # Main control loop
        self.main_control_loop()

    pass

    def main_control_loop(self):
        """
        Main control loop that contains all functions that must be constantly executed with a certain rate (Hz).
        """
        # The Rate instance will attempt to keep the loop at 10 Hz by accounting for the time used by any
        # operations during the loop.
        rate = rospy.Rate(50)  # in Hz

        while not rospy.is_shutdown():

            # base waypoints callback
            self.waypoints_cb(self.base_waypoints_topic_msg)

            # current vehicle pose callback
            self.pose_cb(self.current_pose_topic_msg)

            # traffic light callback - will be uncommented when the light detector is working
            # self.traffic_cb(self.traffic_waypoint_topic_msg)

            # ideal traffic light callback
            if IDEAL_LIGHT_DETECTION:
                self.ideal_traffic_light_detection_cb(self.vehicle_traffic_lights_topic_msg)

            # State transition - will be uncommented when state transition is working
            # self.state_transition()

            # Trajectory generation - currently constant linear velocity
            self.trajectory_generation()

            # Publish the selected trajectory
            self.final_waypoints_publisher(self.final_waypoints_topic_msg)

            rospy.sleep()
    pass

    def state_transition(self):
        """
        Transitions the state depending on traffic light message payload
        """
        if self.current_state == 'STOPPED':
            # TODO: clarify how the traffic light state is communicated.
            if self.next_traffic_light_state == 2:  # Green light state
                self.current_state = 'ACCELERATING'

    pass

    def trajectory_generation(self):
        """
        Produces the trajectory for the vehicle - the header and payload of the look_ahead_waypoints_msg
        """

        # Calculate a 10902 vector of distances in an efficient way using KD-Trees available in scipy.spatial
        self.closest_to_car_waypoint_indx = distance.cdist(XA=self.current_pose_position,
                                                           XB=self.base_waypoints_position,
                                                           metric='euclidean', p=2).argmin()

        # create the look_ahead waypoints considering the wrap around in closed tracks
        self.final_waypoints_topic_msg = Lane()
        for i in range(0, LOOKAHEAD_WPS):
            self.final_waypoints_topic_msg.waypoints.append(
                self.base_waypoints_topic_msg.waypoints[(self.closest_to_car_waypoint_indx + i) % self.num_waypoints])

        # message header
        self.set_look_ahead_waypoints_msg_header(self.final_waypoints_topic_msg, '/World', rospy.Time.now())

        # message paylod - set the linear velocity (x)
        for i in range(0, LOOKAHEAD_WPS):
            self.set_look_ahead_waypoints_msg_velocity(self.final_waypoints_topic_msg.waypoints, i, 5.0)

    pass

    def pose_cb(self, msg):
        """
        Current pose callback. In this call back we calculate the closest waypoint to the current vehicle pose
        :param StampedPose msg: message containing the current pose of the vehicle.
        """
        #rospy.loginfo(msg)

        pose_position = msg.pose.position
        pose_orientation = msg.pose.orientation

        self.current_pose_position = np.array([[pose_position.x, pose_position.y, pose_position.z]])
        self.current_pose_orientation = np.array([[pose_orientation.x, pose_orientation.y, pose_orientation.z, pose_orientation.w]])

        pass

    def waypoints_cb(self, msg):
        """
        Waypoints callback.
        :param Lane msg: message containing the waypoints of the track that the vehicle must follow.
        """
        # rospy.loginfo(waypoints.waypoints)

        # get the number of base waypoints
        self.num_waypoints = len(msg.waypoints)

        # store the incoming waypoints - note this message is sent by the simulator only once
        self.base_waypoints_topic_msg = msg

        # create the numpy 2D array (1092, 3) necessary for estimating the closest waypoint in an efficient manner.
        tmp = []
        for wp in msg.waypoints:
            tmp.append([wp.pose.pose.position.x, wp.pose.pose.position.y, wp.pose.pose.position.z])

        self.base_waypoints_position = np.asarray(tmp)

        # store also the orientatio as a numpy 2D array (10902, 4)
        tmp = []
        for wp in msg.waypoints:
            tmp.append([wp.pose.pose.orientation.x, wp.pose.pose.orientation.y,
                                                    wp.pose.pose.orientation.z, wp.pose.pose.orientation.w])

        self.base_waypoints_orientation = np.asarray(tmp)

        pass

    def ideal_traffic_light_detection_cb(self, msg):
        """
        Sets the variables that determine the position and state of the next traffic light
        as well as its distance from the vehicle. Note that in the ideal traffic light detection we get an array
        (there are 8 traffic lights in the simulated track) of waypoint (x, y, z) positions as opposed to the real-life
        scenario where the traffic light detector is providing the waypoint index of the state=red light.

        :param TrafficLightArray msg: message published by the vehicle with the ideal light position and state.
        """
        rospy.loginfo(msg)

        self.num_traffic_lights = len(msg.lights)

        self.next_traffic_light_distance(msg)

        pass

    def next_traffic_light_distance(self, msg):
        """
        Estimates the distance to next traffic light in the case of ideal traffic light detection.

        :param TrafficLightArray msg: message published by the vehicle with the ideal light position and state.
        """
        # check each  traffic light position with respect to the current pose and produce the next traffic light index.
        self.distance_to_next_traffic_light = 99999.
        for i in msg.lights:

            self.next_traffic_light_position = np.array(
                [[i.pose.pose.position.x, i.pose.pose.position.y, i.pose.pose.position.z]])

            # Convert provided position to waypoint index.
            self.next_traffic_light_waypoint_index = distance.cdist(self.base_waypoints_position,
                                                                    self.next_traffic_light_position, 'euclidean',
                                                                    p=2).argmin()

            rospy.loginfo(self.next_traffic_light_waypoint_index)

            temp_distance = self.distance(self.base_waypoints_topic_msg.waypoints, self.closest_to_car_waypoint_indx,
                                          self.next_traffic_light_waypoint_index)

            # sort the distances and produce the closest red traffic light
            if temp_distance < self.distance_to_next_traffic_light and i.state == 0:
                self.distance_to_next_traffic_light = temp_distance

        pass

    def traffic_cb(self, traffic_light_msg):
        """
        The callback for processing the traffic_waypoint message. If for example 12 is published in /traffic_waypoint topic,
        an upcoming red light's stop line is nearest to base_waypoints[12], This index is used by the waypoint updater node
        to set the target velocity for look_ahead_waypoints_msg.waypoints[12] to 0 and smoothly decrease the vehicle velocity
        in the waypoints leading up to look_ahead_waypoints_msg.waypoints[12].

        :param Int32 traffic_light_msg: indicates the waypoint index where the **red** traffic light was detected by the tl_detector module.
        """

        self.next_traffic_light_waypoint_index = traffic_light_msg.data

        # Here we use the provided distance function that generates a non 0 distance when the
        # red traffic light waypoint index is > closest to the vehicle waypoint index.
        self.distance_to_next_traffic_light = self.distance(self.base_waypoints_topic_msg.waypoints,
                                                            self.closest_to_car_waypoint_indx,
                                                            self.next_traffic_light_waypoint_index)

        pass

    def obstacle_cb(self, msg):
        """
        :param msg:
        """
        # TODO: Callback for /obstacle_waypoint message.

    pass

    def final_waypoints_publisher(self, msg):

        """
        Publishes the look ahead waypoints message in the /final_waypoints topic.
        """

        rospy.loginfo(msg)

        # publish the message
        self.final_waypoints_pub.publish(msg)

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
