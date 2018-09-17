#!/usr/bin/env python

import cv2
import math
import rospy
import tf
import yaml
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, Pose
from light_classification.tl_classifier import TLClassifier
from sensor_msgs.msg import Image
from scipy.spatial import KDTree
from std_msgs.msg import Int32
from styx_msgs.msg import Lane
from styx_msgs.msg import TrafficLightArray, TrafficLight

# Noise filter for traffic light detection classifier.
STATE_COUNT_THRESHOLD = 3


class TLDetector(object):
    def __init__(self):

        # Start ROS Node.
        rospy.init_node('tl_detector')

        # Set attributes.
        self.limit = 0
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.waypoint_tree = None
        self.waypoints_2d = None

        # ROS Subscribers.
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/image_color', Image, self.image_cb)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

        # Load configurations.
        config_string = rospy.get_param('/traffic_light_config')
        self.config = yaml.load(config_string)

        # ROS Publisher.
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        # Image Classifiers.
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        # Traffic Light State.
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.classifier_initialized = False
        self.light_classifier = TLClassifier()
        self.classifier_initialized = True
        rospy.spin()


    def pose_cb(self, msg):
        """ Callback for ROS topic `/current_pose` that updates the pose of the vehicle 
            relative to the world. 

        Args:
            msg (PoseStamped): Transform of the vehicle relative to the world.

        """
        self.pose = m
        

    def waypoints_cb(self, waypoints):
        """ Callback for ROS topic `/base_waypoints` called once upon system start. Passes in the 
            list of all waypoints around the track.

        Args:
            msg (Lane): The lane defining the vehicle path to follow.

        """
        self.waypoints = waypoints

        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)


    def traffic_cb(self, msg):
        """ Callback for ROS topic `/vehicle/traffic_lights` that passes in the current states of the
            traffic lights in the simulator.

        Args:
            msg (TrafficLightArray): List of all traffic light states.

        """
        self.lights = msg.lights


    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """

        # Throttle Callback
        self.limit = (self.limit + 1) % 5
        if self.limit != 0:
            return

        # Extract Light State from Image
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        # Filter the traffic state results for noise.
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

        # Print light state for debugging.
        if self.state == 0:
            rospy.loginfo("TRAFFIC_LIGHT: RED")
        elif self.state == 1:
            rospy.loginfo("TRAFFIC_LIGHT: YELLOW")
        elif self.state == 2:
            rospy.loginfo("TRAFFIC_LIGHT: GREEN")
        else:
            rospy.loginfo("TRAFFIC_LIGHT: UNKNOWN")


    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """ 
        # Set the coordinates for the next point.
        if not self.waypoint_tree:
            return
        closest_idx = self.waypoint_tree.query([x,y],1)[1]
	
        # Check if closest is ahead or behind vehicle.
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        # Equation for hyper plane through closest_coords.
        cl_vest = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        val = np.dot(cl_vest - prev_vect, pos_vect-cl_vest)
        
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx
        

    def get_light_state(self, light):
        """ Determines the current color of the traffic light. The simulator provides
            800x600 RGB8 images.

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # No Image Classifier.
        # return light.state

        # Image Classifier Enabled.
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        if self.classifier_initialized:
            return self.light_classifier.get_classification(cv_image)
        else:
            return TrafficLight.UNKNOWN


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if self.pose:
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            
            # Iterate through all intersections to find closest.
            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])

                d = temp_wp_idx - car_wp_idx
                if 0 <= d < diff:
                    diff = d 
                    closest_light = light
                    line_wp_idx = temp_wp_idx

        # If there is an intersection nearby.
        if closest_light:
            state = self.get_light_state(closest_light)
            return line_wp_idx, state

        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
