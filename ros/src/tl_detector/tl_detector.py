#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3
MAX_DISTANCE = float("inf")
NODE_NAME = "[TL_DETECTOR]: "
MAX_DISTANCE_TO_TL = 100
UNKNOWN_TL_STATE = TrafficLight.UNKNOWN
UNKNOWN_WP_IDX = -1
IS_SIMULATOR = True


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        # same as present in site_traffic_light_config and sim_trafficlight_config
        config_string = rospy.get_param("/traffic_light_config")
        rospy.logwarn(NODE_NAME + "traffic_light_config: %s", config_string)
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # DONE implement
        if not self.waypoints:
            rospy.logwarn(NODE_NAME + "Waypoints not found")

        return TLDetector.get_geometrically_close_point(self.waypoints, pose)

    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        return math.sqrt(dx * dx + dy * dy)

    @staticmethod
    def euclidean_distance_between_pose(pose1, pose2):
        return TLDetector.euclidean_distance(pose1.position.x, pose1.position.y, pose2.position.x, pose2.position.y)

    # https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
    @staticmethod
    def get_geometrically_close_point(waypoints, pose):
        closest_waypoint_distance = MAX_DISTANCE
        closest_waypoint_index = -1
        for i in range(0, len(waypoints)):
            pose1 = waypoints[i].pose.pose

            waypoint_distance = TLDetector.euclidean_distance_between_pose(pose1, pose)
            if waypoint_distance <= closest_waypoint_distance:
                closest_waypoint_distance = waypoint_distance
                closest_waypoint_index = i
        return closest_waypoint_index

    @staticmethod
    def get_pose_from_line(line):
        pose = Pose()
        pose.position.x = line[0]
        pose.position.y = line[1]
        return pose

    def debug(self, message):
        rospy.logdebug(NODE_NAME + message)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if (not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # Get classification
        return self.light_classifier.get_classification(cv_image)

    def get_closest_tl(self):
        return TLDetector.get_geometrically_close_point(self.lights, self.pose.pose)

    def get_closest_light(self):
        closest_light_idx = -1
        closest_wp_idx = self.get_closest_waypoint(self.pose.pose)
        closest_tl_idx = self.get_closest_tl()

        # check if closest TL is not already passed, otherwise we need to pick the next light
        closest_tl = self.lights[closest_tl_idx]
        closest_wp = self.waypoints[closest_wp_idx]

        diff_wpx = closest_wp.pose.pose.position.x - self.pose.pose.position.x
        diff_wpy = closest_wp.pose.pose.position.y - self.pose.pose.position.y

        diff_tlx = closest_tl.pose.pose.position.x - self.pose.pose.position.x
        diff_tly = closest_tl.pose.pose.position.y - self.pose.pose.position.y

        dot_product = diff_wpx * diff_tlx + diff_wpy * diff_tly

        # dot product is less means light is behind car
        if dot_product < 0:
            if closest_light_idx is not len(self.lights) - 1:
                closest_light_idx = closest_light_idx + 1

        return closest_light_idx

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        wp_idx = UNKNOWN_WP_IDX
        state = UNKNOWN_TL_STATE

        # handle no waypoints
        if self.waypoints is not None:
            # List of positions that correspond to the line to stop in front of for a given intersection
            stop_line_positions = self.config['stop_line_positions']

            tl_idx = self.get_closest_light()
            if tl_idx > -1:
                light = self.lights[tl_idx]

                stop_line_position = stop_line_positions[tl_idx]
                # Convert to pose to calculate euclidean distance in consistent way to previous pose types passed
                stop_line_pose = TLDetector.get_pose_from_line(stop_line_position)

                wp_idx = self.get_closest_waypoint(stop_line_pose)
                self.debug("Closest WP index: %s.".format(wp_idx))

                if wp_idx > -1:
                    car_dist = TLDetector.euclidean_distance_between_pose(self.pose.pose, stop_line_pose)

                    if car_dist < MAX_DISTANCE_TO_TL:
                        if IS_SIMULATOR:
                            state = light.state
                        else:
                            if light is not None:
                                state = self.get_light_state(light)
                                self.debug("Nearest TL State - {}".format(state))
                            else:
                                self.debug("No TL found as TL is none")
                    else:
                        self.debug("TL is farther than %d, so ignoring".format(MAX_DISTANCE_TO_TL))

                    self.debug("All info <TL IDx - {} >, <state - {}>, <distance - {}>, <tl_stop_line - ({},{})>"
                               .format(tl_idx, state, car_dist, stop_line_pose.position.x, stop_line_pose.position.y))

            return wp_idx, state

        # DONE find the closest visible traffic light (if one exists)

        self.waypoints = None
        return wp_idx, state


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
