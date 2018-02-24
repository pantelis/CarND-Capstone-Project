#!/usr/bin/env python

#Uncomment lines 3-19 for debugging
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
#pydevd.settrace('172.20.10.6', port=6700)
pydevd.settrace('192.168.1.224', port=6700, stdoutToServer=True, stderrToServer=True)

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Int32
# note: we import TrafficLightArray for testing purposes only
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightArray
import math
import numpy as np
from scipy.spatial import distance
import tf


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
IDEAL_LIGHT_DETECTION = False  # Turns on or off the ideal traffic light detection.


class WaypointUpdater(object):

    def __init__(self):
        """
        Waypoint updated initialization.
        """
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

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

        rospy.Subscriber('/obstacle_waypoints', Int32, self.obstacle_cb)

        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        # Publisher
        # Publish a list of waypoints ahead of the car with target velocities to the /final_waypoints topic.
        # The queue_size=1 argument tells rospy to only buffer a single outbound message. In case the node sending
        # the messages is transmitting at a higher rate than the receiving node(s) can receive them,
        # rospy will simply drop any messages beyond the queue_size.
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Pose member variables
        self.current_pose_position = None
        self.current_pose_orientation_quaternion = None
        self.current_pose_roll = 0.
        self.current_pose_pitch = 0.
        self.current_pose_yaw = 0.

        # Waypoint member variables
        self.base_waypoints_topic_msg = Lane()
        self.final_waypoints_topic_msg = Lane()
        self.num_waypoints = None
        self.base_waypoints_position = np.array
        self.base_waypoints_orientation = np.array
        self.closest_to_vehicle_waypoint_index = None

        # Traffic Light member variables
        self.num_traffic_lights = None
        self.distance_to_next_traffic_light = 99999
        self.next_traffic_light_waypoint_index = -1
        self.next_traffic_light_position = np.zeros([1, 3])  # init to type
        self.next_traffic_light_state = 4  # Unknown initial state as defined in the TrafficLight.msg

        # Trajectory parameters
        self.deceleration_limit = rospy.get_param('~decel_limit', -5)
        self.acceleration_limit = rospy.get_param('~accel_limit', 1.)
        self.nominal_velocity_mps = self.kmph2mps(rospy.get_param('/waypoint_loader/velocity'))
        self.velocity_target_mps = 0
        self.current_velocity_mps = 0
        self.minimum_stopping_distance = 0


        # Main control loop
        # self.main_control_loop()
        rospy.spin()

    pass

    def main_control_loop(self):
        """
        Main control loop that contains all functions that must be constantly executed with a certain rate (Hz).
        """
        # The Rate instance will attempt to keep the loop at 50 Hz by accounting for the time used by any
        # operations during the loop.
        rate = rospy.Rate(50)  # in Hz

        while not rospy.is_shutdown():

            if self.num_waypoints is None and self.current_pose_position is not None:
                continue

            # Publish the selected trajectory
            self.final_waypoints_publisher(self.final_waypoints_topic_msg)

            # Added to maintain th 200ms (50 Hz) loop execution interval
            rate.sleep()

        pass

    def waypoints_cb(self, msg):
        """
        Waypoints callback. Populates required 2D arrays.
        :param Lane msg: message containing the waypoints of the track that the vehicle must follow.
        """
        rospy.loginfo("Waypoints Callback - Called Once")
        # rospy.loginfo(msg.waypoints)

        # store the message that is only transmitted once.
        self.base_waypoints_topic_msg = msg

        # get the number of base waypoints
        self.num_waypoints = len(msg.waypoints)

        pass

    def pose_cb(self, msg):
        """
        Current pose callback. In this call back we calculate the closest waypoint to the current vehicle pose
        :param StampedPose msg: message containing the current pose of the vehicle.
        """
        rospy.loginfo_throttle(1, "Pose Callback")
        # rospy.loginfo_throttle(1, msg)

        self.current_pose_position = np.array([[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]])

        # get the Eurler angle from the provided quaternion
        self.current_pose_orientation_quaternion = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                                                             msg.pose.orientation.z, msg.pose.orientation.w])

        self.current_pose_roll, self.current_pose_pitch, self.current_pose_yaw = \
            tf.transformations.euler_from_quaternion(self.current_pose_orientation_quaternion)

        if self.num_waypoints is not None:  # Ensures that the waypoint_cb was executed.

            # create the numpy 2D array (1092, 3) necessary for estimating the closest waypoint in an efficient manner.
            tmp = []
            for wp in self.base_waypoints_topic_msg.waypoints:
                tmp.append([wp.pose.pose.position.x, wp.pose.pose.position.y, wp.pose.pose.position.z])

            self.base_waypoints_position = np.asarray(tmp)

            # Calculate a 10902 vector of distances in an efficient way using KD-Trees available in scipy.spatial
            self.closest_to_vehicle_waypoint_index = distance.cdist(XA=self.current_pose_position,
                                                                    XB=self.base_waypoints_position,
                                                                    metric='euclidean', p=2).argmin()

            rospy.loginfo_throttle(1, self.closest_to_vehicle_waypoint_index)

            # store also the orientatio as a numpy 2D array (10902, 4)
            tmp = []
            for wp in self.base_waypoints_topic_msg.waypoints:
                tmp.append([wp.pose.pose.orientation.x, wp.pose.pose.orientation.y,
                            wp.pose.pose.orientation.z, wp.pose.pose.orientation.w])

            self.base_waypoints_orientation = np.asarray(tmp)

            rospy.loginfo_throttle(1, self.current_pose_position)

            # TODO: Since the traffic light msg.data is not coming from the TL detector we call the callback here
            # TODO: to test.
            self.traffic_cb(msg)
        pass

    def traffic_cb(self, msg):
        """
        The callback for processing the traffic_waypoint message. If for example 12 is published in /traffic_waypoint topic,
        an upcoming red light's stop line is nearest to base_waypoints[12], This index is used by the waypoint updater node
        to set the target velocity for look_ahead_waypoints_msg.waypoints[12] to 0 and smoothly decrease the vehicle velocity
        in the waypoints leading up to look_ahead_waypoints_msg.waypoints[12].

        :param Int32 msg: indicates the waypoint index where the **red** traffic light was
        detected by the tl_detector module.
        """
        rospy.loginfo("Traffic Light Callback")
        #rospy.loginfo(msg.data)

        self.next_traffic_light_waypoint_index = -1 #msg.data

        print("Next traffic light waypoint index = ", self.next_traffic_light_waypoint_index)

        #  Generate the required trajectory
        self.trajectory_generation(self.next_traffic_light_waypoint_index)

        pass

    def trajectory_generation(self, next_traffic_light_waypoint_index):
        """
        Produces the trajectory for the vehicle - the header and payload of the look_ahead_waypoints_msg
        """

        rospy.loginfo("Trajectory Generation function")

        # create the look_ahead waypoints considering the wrap around in closed loop tracks
        self.final_waypoints_topic_msg = Lane()
        for i in range(0, LOOKAHEAD_WPS):
            self.final_waypoints_topic_msg.waypoints.append(
                self.base_waypoints_topic_msg.waypoints[
                    (self.closest_to_vehicle_waypoint_index + i) % self.num_waypoints])

        # message header
        self.set_look_ahead_waypoints_msg_header(self.final_waypoints_topic_msg, '/world', rospy.Time.now())

        # minimum stopping distance - to be compared against the distance from the traffic light
        self.minimum_stopping_distance = self.minimum_stopping_distance_calcularor(self.current_velocity_mps,
                                                                                   self.deceleration_limit)

        # if the next red traffic light is beyond the traffic light detection distance
        # (parameter in td_detector set currently to 100m) the tl_detector will produce a -1 as the light waypoint index
        if next_traffic_light_waypoint_index == -1:

            self.distance_to_next_traffic_light = 99999

            # target is the nominal velocity [retrieved by the ROS parameter server]
            self.velocity_target_mps = self.nominal_velocity_mps

            # the waypoint velocity
            starting_waypoint_velocity_mps = self.current_velocity_mps

            for i in range(0, LOOKAHEAD_WPS):
                # Here we use the provided distance function that generates a non 0 distance when the
                # red traffic light waypoint index is > the closest to the vehicle waypoint index.
                waypoint_distance_to_next_waypoint = self.distance(self.base_waypoints_topic_msg.waypoints, 0, i)

                waypoint_velocity_mps = min(math.sqrt(starting_waypoint_velocity_mps*starting_waypoint_velocity_mps +
                                                      2 * self.acceleration_limit * waypoint_distance_to_next_waypoint),
                                            self.velocity_target_mps)
                # print(waypoint_velocity_mps)
                self.set_look_ahead_waypoints_msg_velocity(self.final_waypoints_topic_msg.waypoints, i,
                                                           waypoint_velocity_mps)

        # If a red traffic light was found
        elif next_traffic_light_waypoint_index != -1:

            relative_traffic_waypoint_index = abs(self.next_traffic_light_waypoint_index - self.closest_to_vehicle_waypoint_index)

            # and is within the planning horizon
            if relative_traffic_waypoint_index <= LOOKAHEAD_WPS:

                self.velocity_target_mps = 0.  # at traffic light waypoint and beyond the target is set to 0

                # the waypoint velocity
                starting_waypoint_velocity_mps = self.current_velocity_mps

                print("Distance to next traffic light = ", self.distance_to_next_traffic_light)

                # from 0 to min(relative, 200) velocity is decreasing
                for i in range(0, min(relative_traffic_waypoint_index, LOOKAHEAD_WPS)):

                    # Here we use the provided distance function that generates a non 0 distance when the
                    # red traffic light waypoint index is > the closest to the vehicle waypoint index.
                    waypoint_distance_to_next_traffic_light = self.distance(self.base_waypoints_topic_msg.waypoints, i,
                                                                        self.next_traffic_light_waypoint_index)

                    waypoint_velocity_mps = math.sqrt(starting_waypoint_velocity_mps*starting_waypoint_velocity_mps -
                                                      2 * self.deceleration_limit * waypoint_distance_to_next_traffic_light)

                    self.set_look_ahead_waypoints_msg_velocity(self.final_waypoints_topic_msg.waypoints, i,
                                                               waypoint_velocity_mps)

                # from min(traffic_wp-car_wp, 200) to 200 velocity is 0.
                for j in range(min(relative_traffic_waypoint_index, LOOKAHEAD_WPS), LOOKAHEAD_WPS):
                    waypoint_velocity_mps = 0
                    self.set_look_ahead_waypoints_msg_velocity(self.final_waypoints_topic_msg.waypoints, j,
                                                               waypoint_velocity_mps)
            else:
                print("Red light outside of planning horizon")

        pass

    def current_velocity_cb(self, msg):
        self.current_velocity_mps = msg.twist.linear.x

    def final_waypoints_publisher(self, msg):

        """
        Publishes the look ahead waypoints message in the /final_waypoints topic.
        """

        rospy.loginfo_throttle(1, "Final Waypoints Publisher")

        # publish the message
        self.final_waypoints_pub.publish(msg)

        pass

    def obstacle_cb(self, msg):
        """
        :param msg:
        """
        # TODO: Callback for /obstacle_waypoint message.

    pass

    def ideal_traffic_light_detection_cb(self, msg):
        """
        Sets the variables that determine the position and state of the next traffic light
        as well as its distance from the vehicle. Note that in the ideal traffic light detection we get an array
        (there are 8 traffic lights in the simulated track) of waypoint (x, y, z) positions as opposed to the real-life
        scenario where the traffic light detector is providing the waypoint index of the state=red light.

        :param TrafficLightArray msg: message published by the vehicle with the ideal light position and state.
        """
        rospy.loginfo_throttle(1, msg)

        # set the number of traffic lights in the track
        self.num_traffic_lights = len(msg.lights)

        # calculate the distance to next traffic light
        self.distance_to_next_traffic_light = self.next_traffic_light_distance_calculator(msg)

        pass

    def next_traffic_light_distance_calculator(self, msg):
        """
        Estimates the distance to next traffic light in the case of ideal traffic light detection. This function is called
        when the TL detector is not used for testing purposes.

        :param TrafficLightArray msg: message published by the vehicle with the ideal light position and state.
        """

        # check each traffic light position with respect to the current pose and produce the next traffic light index.
        distance_to_next_traffic_light = 99999.
        for i in msg.lights:

            self.next_traffic_light_position = np.array(
                [[i.pose.pose.position.x, i.pose.pose.position.y, i.pose.pose.position.z]])

            # Convert provided position to waypoint index.
            self.next_traffic_light_waypoint_index = distance.cdist(self.base_waypoints_position,
                                                                    self.next_traffic_light_position, 'euclidean',
                                                                    p=2).argmin()

            rospy.loginfo('next_traffic_light_waypoint_index %d', self.next_traffic_light_waypoint_index)

            temp_distance = self.distance(self.base_waypoints_topic_msg.waypoints, self.closest_to_vehicle_waypoint_index,
                                          self.next_traffic_light_waypoint_index)

            # sort the distances and produce the closest red traffic light
            if temp_distance < distance_to_next_traffic_light and i.state == 0:  # RED
                distance_to_next_traffic_light = temp_distance

        rospy.loginfo_throttle(1, distance_to_next_traffic_light)
        return distance_to_next_traffic_light

    def set_look_ahead_waypoints_msg_header(self, look_ahead_waypoints_msg, frame_id, time_stamp):

        look_ahead_waypoints_msg.header.frame_id = frame_id
        look_ahead_waypoints_msg.header.stamp = time_stamp

    pass

    def get_waypoint_velocity(self, waypoint):

        return waypoint.twist.twist.linear.x

    pass

    def set_look_ahead_waypoints_msg_velocity(self, look_ahead_waypoints_msg, wp, velocity_mps):

        # vx = v * cos(yaw)
        look_ahead_waypoints_msg[wp].twist.twist.linear.x = velocity_mps * math.cos(self.current_pose_yaw)

    pass

    def distance(self, waypoints, wp1, wp2):

        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    pass

    def minimum_stopping_distance_calcularor(self, v_i, alpha):
        '''

        :param v_i: Initial velocity
        :param alpha: deceleration limit provided by parameter server
        :return: minimum stopping distance of the car in meters
        '''

        # using the formula: v_f^2 = v_i^2 + 2*alpha (x_f - x_i)
        # where x_f-x_i is the stopping distance if the final velocity v_f = 0.0.

        minimim_stopping_distance = (v_i * v_i)/(2.*alpha)

        return minimim_stopping_distance

    def deg2rad(self, deg):

        return deg * math.pi / 180.
    pass

    def rad2deg(self, rad):
        return rad * 180. / math.pi
    pass

    # km/h to meters per second
    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    # mph to meters per sec
    def mph2mps(self, miles_per_hour):

        return miles_per_hour * 0.44704
    pass

    # meter per sec to mph
    def mps2mph(self, meters_per_second):

        return meters_per_second * 2.23694
    pass

    def debug(self, message):
        rospy.logdebug('WAYPOINT UPDATER: ' + message)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
