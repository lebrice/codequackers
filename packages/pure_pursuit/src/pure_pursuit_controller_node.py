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
# from duckietown_msgs.msg import PointList
from geometry_msgs.msg import Point

import os
import datetime
# import test_pure_pursuit

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
    return np.asarray([point_2d[0], point_2d[1], 0])


def point_to_np(point_or_points):
    if isinstance(point_or_points, list):
        return np.asarray([[p.x, p.y] for p in point_or_points])
    else:
        return np.asarray([point_or_points.x, point_or_points.y])


def R(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])


def TR(theta_rad, Ax, Ay):
    return np.array([
        [np.cos(theta_rad), -np.sin(theta_rad), Ax],
        [np.sin(theta_rad),  np.cos(theta_rad), Ay],
        [0, 0, 1],
    ])


class Color(enum.Enum):
    WHITE = Segment.WHITE
    YELLOW = Segment.YELLOW
    RED = Segment.RED

class pure_pursuit_controller(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.loginfo("Node Name: {}".format(self.node_name))
        
        last_edit_time = os.environ.get("LAST_EDIT_TIME", "UNKNOWN")
        self.logwarn("This code was last edited on {}".format(last_edit_time))
        
        self.header = None
        # Publication
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        # self.pub_path_points = rospy.Publisher("~path_points", PointList, queue_size=1)
        
        # Subscriptions
        self.sub_seglist_filtered = rospy.Subscriber("~seglist_filtered", SegmentList, self.new_segments_received, queue_size=1)
        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        buffer_length_fallback = 10
        self.buffer_length = self.setupParameter("~buffer_length", buffer_length_fallback) # keep 50 points at a time for each color.
        self.points = collections.defaultdict(lambda: collections.deque(maxlen=self.buffer_length))
        
        lookahead_dist_fallback = 0.5
        self.lookahead_dist = self.setupParameter("~lookahead_dist", lookahead_dist_fallback)
        
        max_speed_fallback = 0.1
        self.max_speed = self.setupParameter("~v_max", max_speed_fallback)
        
        delta_t_fallback = 0.1
        self.delta_t = self.setupParameter("~delta_t", delta_t_fallback)   
        self.car_command_timer = rospy.Timer(rospy.Duration.from_sec(self.delta_t), self.update_car_command)
        
        # self.clear_points_timer = rospy.Timer(rospy.Duration.from_sec(1), self.clear_points)

        self.loginfo("Initialized")

        self.points_lock = Lock()
   
    def custom_shutdown(self):
        self.loginfo("Shutting down...")

        # Stop listening
        self.sub_seglist_filtered.unregister()
        
        # stop timer
        self.car_command_timer.shutdown()        

        # Send stop command
        self.loginfo("Stopping the robot")
        self.send_car_command(0.0,0.0)

        rospy.sleep(1.0)    #To make sure that it gets published.
        self.loginfo("Shutdown")

    def publish_path_points(self):
        path_points_msg = PointList()
        if self.header is not None:
            path_points_msg.header = self.header
        with self.points_lock:
            path_points_msg.points = [p for p in self.points[Color.YELLOW]]
        self.pub_path_points.publish(path_points_msg)


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

    def update_car_command(self, timer_event):
        self.logdebug("updating car command")
        
        # self.update_past_path_point_coordinates(timer_event)
        # self.publish_path_points()

        self.logdebug("Points: {}".format({color: len(values) for color, values in self.points.items()}))

        if not self.has_points():
            self.logwarn("Can't see any lines, just going straight at max speed.")
            self.send_car_command(self.max_speed, 0)
            return
        elif not self.has_points(Color.YELLOW) and self.has_points(Color.WHITE):
            white_centroid = self.find_point_closest_to_lookahead_distance(Color.WHITE)
            # white_centroid = self.find_centroid(Color.WHITE)
            distance_to_center = 0.15
            if white_centroid.y > 0:
                self.logwarn("Haven't seen the yellow line yet, but the white line is present and to the RIGHT")
            else:
                self.logwarn("Haven't seen the yellow line yet, but the white line is present and to the LEFT")

            offset = (1 if white_centroid.y < 0 else -1) * distance_to_center

            target = white_centroid
            target.y += offset
        elif self.has_points(Color.YELLOW):
            # target = self.find_centroid(Color.YELLOW)
            target = self.find_point_closest_to_lookahead_distance()

        else:
            self.logwarn("Weird, didn't fit into any of the above cases...")
            self.logwarn("Points: {}".format({color: len(values) for color, values in self.points.items()}))
            target = Point(1.0, 0.0)
        
        self.logdebug("Target: {}".format(target))
        
        self.target = target    

        target_np = point_to_np(target)
        self.logdebug("Target: {}".format(target_np))
        hypothenuse = np.sqrt(target_np.dot(target_np))
        sin_alpha = target.y / hypothenuse
        
        v = self.max_speed
        omega = 2 * sin_alpha / self.lookahead_dist
        self.send_car_command(v, omega)

    def find_point_closest_to_lookahead_distance(self, color=Color.YELLOW):
        def distance_mag(point):
            point_np = point_to_np(point)
            return point_np.dot(point_np)
        mag = self.lookahead_dist ** 2
        with self.points_lock:
            points = self.points[color]
            distances = [abs(distance_mag(p) - mag) for p in points]
            min_index = np.argmin(distances)
            best_point = points[min_index]
        return best_point
    
    def has_points(self, color=None):
        if color is None:
            return any(len(values) > 0 for values in self.points.values())
        return len(self.points[color]) > 0

    def clear_points(self, timer_event):
        with self.points_lock:
            for color, point_list in self.points.items():
                point_list.clear()

    def update_past_path_point_coordinates(self, timer_event):
        """
        After each timestep, we update the coordinates of the points we stored in the buffer
        """
        if timer_event.last_real is None:
            self.logwarn("Can't update past point coordinates as the timer_event has last_real of None.")
            return
        duration = rospy.Time.now() - timer_event.last_real
        delta_t = duration.secs + duration.nsecs / 1e9
        self.logdebug("delta_t: {}".format(delta_t))
        
        # change in heading: alpha = omega * dt
        alpha = self.omega * delta_t
        # displacement in the x and y direction.
        delta_x = self.v * np.cos(alpha)
        delta_y = self.v * np.sin(alpha)
        displacement = np.asarray([
            delta_x, delta_y
        ])
        rotation = R(alpha)

        def update_point(point):
            old_pos = point_to_np(point)
            new_pos = old_pos - displacement
            new_pos = np.matmul(rotation, new_pos)
            point.x = new_pos[0]
            point.y = new_pos[1]
            return point

        with self.points_lock:
            for color, old_points_buffer in self.points.items():
                self.points[color] = collections.deque([update_point(p) for p in old_points_buffer])
        self.logdebug("Updated Points successfully.")

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
        
    def send_car_command(self, v, omega):
        car_control_msg = Twist2DStamped()
        if self.header is not None:
            car_control_msg.header = self.header

        if np.isnan(v) or np.isnan(omega):
            self.logerror("NAN car_command!")
            return
        self.v = v
        self.omega = omega
        
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

    def logerror(self, s):
        rospy.logerror("[{}] {}".format(self.node_name, s))

    def setupParameter(self, param_name, default_value):
        if "brigitte" in self.node_name:
            self.logwarn("Using default value for parameter {} since we're running on the duckiebot.".format(param_name))
            value = default_value
        else:
            value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)   # Write to parameter server for transparancy
        self.loginfo("{} = {} ".format(param_name, value))
        return value

if __name__ == "__main__":
    rospy.init_node("pure_pursuit_controller_node", anonymous=False, log_level=rospy.DEBUG)  # adapted to sonjas default file

    lane_control_node = pure_pursuit_controller()
    rospy.spin()
