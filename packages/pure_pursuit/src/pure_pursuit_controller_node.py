#!/usr/bin/env python
import collections
import enum
import math
import time
import itertools
from threading import Lock

import numpy as np
import rospy
from duckietown_msgs.msg import (BoolStamped, FSMState, LanePose,
                                 Segment, SegmentList, StopLineReading,
                                 Twist2DStamped, WheelsCmdStamped)
from geometry_msgs.msg import Point

def wrap(angle):
    """Takes in any angle (in radians), and expresses it inside the [-np.pi, np.pi] range.
    
    Arguments:
        angle {float} -- An angle in radians
    
    Returns:
        float -- The angle projected within the range (-np.pi, np.pi]
    """
    two_pi = 2 * np.pi
    angle %= two_pi
    if angle < 0:
        angle += two_pi
    # angle is now inside [0, 2pi]
    if angle > np.pi:
        angle -= two_pi
    assert (- np.pi) < angle <= np.pi
    return angle

def to_3d(point_2d):
    return np.asarray([point_2d[0], 0, point_2d[1]])

def point_to_np(point_or_points):
    if isinstance(point_or_points, list):
        return np.asarray([[p.x, p.y] for p in point_or_points])
    else:
        return np.asarray([point_or_points.x, point_or_points.y])

class Color(enum.Enum):
    WHITE = Segment.WHITE
    YELLOW = Segment.YELLOW
    RED = Segment.RED

class pure_pursuit_controller(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        # Publication
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        
        # Subscriptions
        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.handle_pose, queue_size=1)
        self.sub_seglist_filtered = rospy.Subscriber("~seglist_filtered", SegmentList, self.new_segments_received, queue_size=1)
        
        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        self.yellow_points = list()
        self.white_points = list()

        buffer_length_fallback = 5
        self.buffer_length = self.setupParameter("~buffer_length", buffer_length_fallback) # keep 50 points at a time for each color.
        self.points = collections.defaultdict(lambda: collections.deque(maxlen=self.buffer_length))

        lookahead_dist_fallback = 0.25
        self.lookahead_dist = self.setupParameter("~lookahead_dist", lookahead_dist_fallback)
        
        v_max_fallback = 0.5
        self.max_speed = self.setupParameter("~v_max", v_max_fallback)
        
        # self.clear_points_timer = rospy.Timer(rospy.Duration.from_sec(0.1), self.clear_points)

        self.loginfo("Initialized")

        self.points_lock = Lock()
   
    def custom_shutdown(self):
        self.loginfo("Shutting down...")

        # Stop listening
        self.sub_lane_pose.unregister()
        self.sub_seglist_filtered.unregister()

        # Send stop command
        self.send_car_command(0.0,0.0)

        rospy.sleep(0.5)    #To make sure that it gets published.
        self.loginfo("Shutdown")
 
    def new_segments_received(self, inlier_segments_msg):
        self.header = inlier_segments_msg.header

        segments = inlier_segments_msg.segments
        self.loginfo("Received {} new segments".format(len(segments)))
        if len(segments) == 0:
            return
        self.add_segments(segments)

        centroid = self.find_centroid()
        centroid_np = point_to_np(centroid)
        self.loginfo("Centroid: {}".format(centroid_np))

        # print("average point:", average_point)
        # alpha = np.arctan2(average_point.y, average_point.x)
        hypothenuse = np.sqrt(centroid_np.dot(centroid_np))
        sin_alpha = centroid.y / hypothenuse
        # self.loginfo("sin_alpha: {:.3f}".format(sin_alpha))

        v = 0.5
        omega = sin_alpha / self.lookahead_dist
        self.send_car_command(v, omega)

    def add_segments(self, segments):
        for i, segment in enumerate(segments):
            color = Color(segment.color)
            assert color in [Color.RED, Color.YELLOW, Color.WHITE]
            with self.points_lock:
                self.points[color].extend(segment.points)
            # self.loginfo("segment: {!s}".format(segment))


    def find_centroid(self):
        with self.points_lock:
            points_to_average = list(itertools.chain(self.points[Color.YELLOW], self.points[Color.WHITE]))
            self.loginfo(
                "{} points to average ({} yellow and {} white)".format(
                    len(points_to_average),
                    len(self.points[Color.YELLOW]), len(self.points[Color.WHITE])
                )
            )
            x_centroid = np.mean([p.x for p in points_to_average])
            y_centroid = np.mean([p.y for p in points_to_average])
        centroid = Point(x=x_centroid, y=y_centroid)
        self.centroid = centroid
        return centroid


    def handle_pose(self, pose_msg):
        # self.loginfo("New pose received!")
        self.header = pose_msg.header
        
       
        

    def send_car_command(self, v, omega):
        car_control_msg = Twist2DStamped()
        if self.header is not None:
            car_control_msg.header = self.header

        car_control_msg.v = v
        car_control_msg.omega = omega

        self.loginfo("Sending car command: v: {} omega: {}".format(v, omega))
        self.pub_car_cmd.publish(car_control_msg)
        

    def loginfo(self, s):
        rospy.loginfo("[{}] {}".format(self.node_name, s))

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value)   # Write to parameter server for transparancy
        self.loginfo("{} = {} ".format(param_name,value))
        return value

if __name__ == "__main__":
    rospy.init_node("pure_pursuit_controller_node", anonymous=False)  # adapted to sonjas default file

    lane_control_node = pure_pursuit_controller()
    rospy.spin()
