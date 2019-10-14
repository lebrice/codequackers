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

        buffer_length_fallback = 10
        self.buffer_length = self.setupParameter("~buffer_length", buffer_length_fallback) # keep 50 points at a time for each color.
        self.points = collections.defaultdict(lambda: collections.deque(maxlen=self.buffer_length))
        
        lookahead_dist_fallback = 0.45
        self.lookahead_dist = self.setupParameter("~lookahead_dist", lookahead_dist_fallback)
        
        v_max_fallback = 0.5
        self.max_speed = self.setupParameter("~v_max", v_max_fallback)
        
        delta_t_fallback = 0.1
        self.delta_t = self.setupParameter("~delta_t", delta_t_fallback)   
        self.car_command_timer = rospy.Timer(rospy.Duration.from_sec(self.delta_t), self.update_car_command)
        

        self.header = None
        

        self.loginfo("Initialized")

        self.points_lock = Lock()
   
    def custom_shutdown(self):
        self.loginfo("Shutting down...")

        # Stop listening
        self.sub_lane_pose.unregister()
        self.sub_seglist_filtered.unregister()

        # Send stop command
        self.loginfo("Stopping the robot")
        self.send_car_command(0.0,0.0)

        rospy.sleep(0.5)    #To make sure that it gets published.
        self.loginfo("Shutdown")
 
    def new_segments_received(self, inlier_segments_msg):
        self.header = inlier_segments_msg.header
        segments = inlier_segments_msg.segments
        self.logdebug("Received {} new segments".format(len(segments)))
        for i, segment in enumerate(segments):
            # print(segment)
            color = Color(segment.color)
            assert color in [Color.RED, Color.YELLOW, Color.WHITE]
            with self.points_lock:
                self.points[color].extend(segment.points)


    def update_car_command(self, timerevent):
        self.logdebug("updating car command")
        if not self.has_points(Color.YELLOW):
            if self.has_points(Color.WHITE):
                white_centroid = self.find_centroid(Color.WHITE)
                distance_to_center = 0.15
                if white_centroid.y > 0:
                    self.logwarn("Haven't seen the yellow line yet, but the white line is present and to the RIGHT")
                else:
                    self.logwarn("Haven't seen the yellow line yet, but the white line is present and to the LEFT")

                offset = (1 if white_centroid.y < 0 else -1) * distance_to_center

                centroid = white_centroid
                centroid.y += offset
            else:
                self.logwarn("Can't see any lines, just going straight at max speed.")
                self.send_car_command(self.max_speed, 0)
                return
        else:
            centroid = self.find_centroid(Color.YELLOW)
        
        self.loginfo("Points: {}".format({color: len(values) for color, values in self.points.item()}))

        centroid_np = point_to_np(centroid)
        self.logdebug("Centroid: {}".format(centroid_np))
        # print("average point:", average_point)
        # alpha = np.arctan2(average_point.y, average_point.x)
        hypothenuse = np.sqrt(centroid_np.dot(centroid_np))
        sin_alpha = centroid.y / hypothenuse
        # self.loginfo("sin_alpha: {:.3f}".format(sin_alpha))
        
        v = self.max_speed
        omega = sin_alpha / self.lookahead_dist
        self.send_car_command(v, omega)

    def has_points(self, color=None):
        if color == None:
            return all(len(values) > 0 for color, values in self.points.items())
        return len(self.points[color]) > 0

    def find_centroid(self, color=None):
        with self.points_lock:
            if color is None:
                self.logdebug("Averaging points of all colors")
                points_to_average = list(itertools.chain(*self.points.values()))
            else:
                self.logdebug("Averaging points of color {}".format(color))
                points_to_average = self.points[color]
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

        self.logdebug("Sending car command: v: {} omega: {}".format(v, omega))
        self.pub_car_cmd.publish(car_control_msg)
        

    def loginfo(self, s):
        rospy.loginfo("[{}] {}".format(self.node_name, s))

    def logdebug(self, s):
        rospy.logdebug("[{}] {}".format(self.node_name, s))

    def logwarn(self, s):
        rospy.logwarn("[{}] {}".format(self.node_name, s))

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)   # Write to parameter server for transparancy
        self.loginfo("{} = {} ".format(param_name, value))
        return value

if __name__ == "__main__":
    rospy.init_node("pure_pursuit_controller_node", anonymous=False)  # adapted to sonjas default file

    lane_control_node = pure_pursuit_controller()
    rospy.spin()
