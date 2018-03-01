#!/usr/bin/env python

#Uncomment lines 3-19 for debugging
import sys

# The following lines are specific to Pycharm Remote Debugging configuration (pydev) that allows
# the host os to run the IDE and the Simulator and the remote Ubuntu VM to run ROS.
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
# #pydevd.settrace('172.20.10.6', port=6700)
# pydevd.settrace('192.168.1.224', port=6700, stdoutToServer=True, stderrToServer=True)




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

        # Pose member variables
        self.current_pose = None
        self.current_pose_position = None
        self.current_pose_orientation_quaternion = None
        self.current_pose_roll = 0.
        self.current_pose_pitch = 0.
        self.current_pose_yaw = 0.

        # Waypoint member variables
        self.base_waypoints_topic_msg = None
        self.final_waypoints = None
        #self.final_waypoints_topic_msg.waypoints = [Waypoint() for _ in range(LOOKAHEAD_WPS)]
        self.num_waypoints = None
        self.closest_to_vehicle_waypoint_index = None

        # Traffic Light member variables
        self.num_traffic_lights = None
        self.next_traffic_light_waypoint_index = None
        self.next_traffic_light_position = np.zeros([1, 3])  # for testing ideal traffic light
        self.next_traffic_light_state = 4  # Unknown initial state as defined in the TrafficLight.msg

        # Trajectory parameters
        self.deceleration_limit = abs(rospy.get_param('~decel_limit', -5))  # positive deceleration limit
        self.acceleration_limit = rospy.get_param('~accel_limit', 1.)
        self.max_target_velocity_mps = self.kmph2mps(rospy.get_param('/waypoint_loader/velocity'))
        self.minimum_stopping_distance = 0

        # Subscribers
        # The /base_waypoints topic publishes a list of all waypoints for the track, so this list includes waypoints
        # both before and after the vehicle (note that the publisher for /base_waypoints publishes only once).
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # PoseStamped is a composite type consisting of a Pose type and a Header type
        # that contains a reference coordinate frame and a timestamp.
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)

        if IDEAL_LIGHT_DETECTION:
            # Note this subscriber is for testing the waypoint updater using ideal traffic light detection.
            rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.ideal_traffic_light_detection_cb)

        rospy.Subscriber('/obstacle_waypoints', Int32, self.obstacle_cb, queue_size=1)

        # asked the forum whether we need to subscribe to this or just use the base waypoints nominal velocity
        # rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        # Publisher
        # Publish a list of waypoints ahead of the car with target velocities to the /final_waypoints topic.
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rate = rospy.Rate(50)

        rospy.spin()

    # ============================================================================================
    def waypoints_cb(self, msg):
        """
        Waypoints callback. Populates required 2D arrays.
        :param Lane msg: message containing the waypoints of the track that the vehicle must follow.
        """
        # rospy.loginfo("Waypoints Callback - Called Once")
        # rospy.loginfo(msg.waypoints)

        # store the message that is only transmitted once.
        self.base_waypoints_topic_msg = msg

        # get the number of base waypoints
        self.num_waypoints = len(msg.waypoints)

    # ============================================================================================
    def pose_cb(self, msg):
        """
        Current pose callback. In this call back we calculate the closest waypoint to the current vehicle pose
        :param StampedPose msg: message containing the current pose of the vehicle.
        """
        # rospy.loginfo_throttle(1, "Pose Callback")
        # rospy.loginfo_throttle(1, msg)

        self.current_pose = msg

        self.current_pose_position = msg.pose.position

        # get the Eurler angle from the provided quaternion
        self.current_pose_orientation_quaternion = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                                                             msg.pose.orientation.z, msg.pose.orientation.w])

        self.current_pose_roll, self.current_pose_pitch, self.current_pose_yaw = \
            tf.transformations.euler_from_quaternion(self.current_pose_orientation_quaternion)

        # get closest to the vehicle waypoint index
        if self.base_waypoints_topic_msg is not None:
            self.closest_to_vehicle_waypoint_index = self.closest_waypoint_estimator(self.base_waypoints_topic_msg.waypoints,
                                                                                     self.current_pose_position)
            # print(self.closest_to_vehicle_waypoint_index)


        # TODO: Remove the hardcoding of the traffic light waypoint index
        # self.next_traffic_light_waypoint_index = -1

        # generate the trajectory
        if self.next_traffic_light_waypoint_index is not None and self.base_waypoints_topic_msg is not None and \
           self.closest_to_vehicle_waypoint_index is not None:

            self.final_waypoints = self.trajectory_generation(self.base_waypoints_topic_msg.waypoints,
                                                              self.closest_to_vehicle_waypoint_index,
                                                              self.next_traffic_light_waypoint_index)

            self.final_waypoints_publisher()
    # ========================================================================================================
    def trajectory_generation(self, base_waypoints_topic_msg_waypoints, closest_to_vehicle_waypoint_index,
                              next_traffic_light_waypoint_index):
        """
        Produces the trajectory for the vehicle - the header and payload of the final_waypoints_topic_msg
        For the kinematics we are using the formula: v_f^2 = v_i^2 + 2*alpha (x_f - x_i)
        f : final position / velocity
        i : initial position / velocity
        alpha : acceleration / deceleration
        x_f-x_i is the stopping distance if the final velocity v_f = 0.0.
        """

        # rospy.loginfo("Trajectory Generation function")

        final_waypoints = []
        for i in range(0, LOOKAHEAD_WPS):
            final_waypoints.append(base_waypoints_topic_msg_waypoints[
                                       (closest_to_vehicle_waypoint_index + i) % self.num_waypoints])

        # starting waypoint velocity is the current waypoint velocity
        starting_waypoint_velocity_mps = self.get_waypoint_velocity(final_waypoints[0])

        # if the next red traffic light is beyond the traffic light detection distance
        # (parameter in td_detector set currently to 100m) the tl_detector will produce a -1 as the light waypoint index
        if next_traffic_light_waypoint_index == -1:

            for i in range(0, LOOKAHEAD_WPS):
                # Here we use the provided distance function that generates a non 0 distance when the
                # red traffic light waypoint index is > the closest to the vehicle waypoint index.
                waypoint_distance_to_next_waypoint = self.distance(final_waypoints, 0, i)

                waypoint_velocity_mps = min(math.sqrt(starting_waypoint_velocity_mps**2 +
                                                      2 * self.acceleration_limit * waypoint_distance_to_next_waypoint),
                                            self.max_target_velocity_mps)

                self.set_look_ahead_waypoints_msg_velocity(final_waypoints, i, waypoint_velocity_mps)


        elif next_traffic_light_waypoint_index != -1:

        # If a red traffic light was found
        # else:

            relative_traffic_waypoint_index = (next_traffic_light_waypoint_index - closest_to_vehicle_waypoint_index) % LOOKAHEAD_WPS
            # print(relative_traffic_waypoint_index)

            # # TODO: fix this
            # if relative_traffic_waypoint_index >= 8:
            #     relative_traffic_waypoint_index = relative_traffic_waypoint_index - 5
            #     # print(str('after -8 =') + str(relative_traffic_waypoint_index))

            # If red traffic light is within the planning horizon
            if relative_traffic_waypoint_index <= LOOKAHEAD_WPS:

                # starting waypoint velocity is the current waypoint velocity
                starting_waypoint_velocity_mps = self.get_waypoint_velocity(final_waypoints[0])

                # minimum stopping distance
                self.minimum_stopping_distance = self.minimum_stopping_distance_calcularor(starting_waypoint_velocity_mps,
                                                                                           self.deceleration_limit)

                traffic_light_distance = self.distance(final_waypoints, 0, relative_traffic_waypoint_index)

                # if vehicle can stop
                if traffic_light_distance > self.minimum_stopping_distance:

                    # from 0 to min(relative, 200) velocity is decreasing
                    for i in range(0, min(relative_traffic_waypoint_index, LOOKAHEAD_WPS)):

                        # Here we use the provided distance function that generates a non 0 distance when the
                        # red traffic light waypoint index is > the closest to the vehicle waypoint index.
                        waypoint_distance_to_next_traffic_light = self.distance(final_waypoints, i,
                                                                                relative_traffic_waypoint_index)

                        # if the starting velocity is 0, just approach slowly the red traffic light
                        # this happens at the very beginning of the simulated track only.
                        if starting_waypoint_velocity_mps < 0.1:
                            waypoint_velocity_mps = 1.
                        else:
                            waypoint_velocity_mps = math.sqrt(max(starting_waypoint_velocity_mps**2 -
                                                              2 * self.deceleration_limit *
                                                              waypoint_distance_to_next_traffic_light, 0.))

                        self.set_look_ahead_waypoints_msg_velocity(final_waypoints, i, waypoint_velocity_mps)

                    # from min(traffic_wp-car_wp, 200) to 200, velocity is 0.
                    for j in range(min(relative_traffic_waypoint_index, LOOKAHEAD_WPS), LOOKAHEAD_WPS):
                        waypoint_velocity_mps = 0
                        self.set_look_ahead_waypoints_msg_velocity(final_waypoints, j, waypoint_velocity_mps)
                # else:
                #     # if vehicle cant stop, continue with waypoint speed
                #     waypoint_velocity_mps = self.get_waypoint_velocity(final_waypoints[0])

        return final_waypoints

    # ============================================================================================
    def obstacle_cb(self, msg):
        """
        :param msg:
        """
        # TODO: Callback for /obstacle_waypoint message.

    # =================================================================================================
    def traffic_cb(self, msg):
        """
        The callback for processing the traffic_waypoint message. If for example 12 is published in /traffic_waypoint topic,
        an upcoming red light's stop line is nearest to base_waypoints[12], This index is used by the waypoint updater node
        to set the target velocity for look_ahead_waypoints_msg.waypoints[12] to 0 and smoothly decrease the vehicle velocity
        in the waypoints leading up to look_ahead_waypoints_msg.waypoints[12].
        :param Int32 msg: indicates the waypoint index where the **red** traffic light was
        detected by the tl_detector module.
        """

        #rospy.logwarn("Traffic light callback {}".format(msg))

        self.next_traffic_light_waypoint_index = msg.data
        # print ("default: " + str(self.next_traffic_light_waypoint_index))

        #rospy.logwarn("Traffic light callback {}".format(self.next_traffic_light_waypoint_index))

    # ============================================================================================
    def final_waypoints_publisher(self):

        """
        Publishes the look ahead waypoints message in the /final_waypoints topic.
        """
        # rospy.loginfo_throttle(1, "Final Waypoints Publisher")

        self.final_waypoints_topic_msg = Lane()

        #  message header
        self.final_waypoints_topic_msg.header = self.base_waypoints_topic_msg.header

        # waypoints
        self.final_waypoints_topic_msg.waypoints = self.final_waypoints

        # publish the message
        self.final_waypoints_pub.publish(self.final_waypoints_topic_msg)

    # ============================================================================================

    def closest_waypoint_estimator(self, base_waypoints_topic_msg_waypoints, current_pose_position):

        # if len(base_waypoints_topic_msg_waypoints) == 0:
        #     rospy.loginfo("len(base_waypoints_topic_msg_waypoints) == 0")

        closest_dist = 99999.
        closest_to_vehicle_waypoint_index = 0
        for i in range(len(base_waypoints_topic_msg_waypoints)):
            dist = self.euclidean_distance(current_pose_position, base_waypoints_topic_msg_waypoints[i].pose.pose.position)
            if dist < closest_dist:
                closest_dist = dist
                closest_to_vehicle_waypoint_index = i

        return closest_to_vehicle_waypoint_index

    # ============================================================================================
    def current_velocity_cb(self, msg):
            self.current_velocity_mps = msg.twist.linear.x

    # ============================================================================================

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

    # ============================================================================================
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

            # rospy.loginfo('next_traffic_light_waypoint_index %d', self.next_traffic_light_waypoint_index)

            temp_distance = self.distance(self.base_waypoints_topic_msg.waypoints, self.closest_to_vehicle_waypoint_index,
                                          self.next_traffic_light_waypoint_index)

            # sort the distances and produce the closest red traffic light
            if temp_distance < distance_to_next_traffic_light and i.state == 0:  # RED
                distance_to_next_traffic_light = temp_distance

        # rospy.loginfo_throttle(1, distance_to_next_traffic_light)
        return distance_to_next_traffic_light

    # ============================================================================================
    def set_look_ahead_waypoints_msg_header(self, look_ahead_waypoints_msg, frame_id, time_stamp):

        look_ahead_waypoints_msg.header.frame_id = frame_id
        look_ahead_waypoints_msg.header.stamp = time_stamp

    # ============================================================================================
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x


    # ============================================================================================
    def set_look_ahead_waypoints_msg_velocity(self, look_ahead_waypoints_msg, wp, velocity_mps):

        look_ahead_waypoints_msg[wp].twist.twist.linear.x = velocity_mps

    # ============================================================================================
    # compute distance between two given points (provided by Udacity)
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    # ============================================================================================

    def euclidean_distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)

    # ============================================================================================
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

    # ============================================================================================
    def deg2rad(self, deg):
        return deg * math.pi / 180.

    # ============================================================================================
    def rad2deg(self, rad):
        return rad * 180. / math.pi

    # ============================================================================================
    # km/h to meters per second
    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    # ============================================================================================
    # mph to meters per sec
    def mph2mps(self, miles_per_hour):
        return miles_per_hour * 0.44704

    # ============================================================================================
    # meter per sec to mph
    def mps2mph(self, meters_per_second):
        return meters_per_second * 2.23694

    # ============================================================================================
    # def debug(self, message):
    #     rospy.logdebug('WAYPOINT UPDATER: ' + message)


# ============================================================================================
if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
