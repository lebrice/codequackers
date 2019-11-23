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
                                 Twist2DStamped, WheelsCmdStamped, Vector2D)
# from duckietown_msgs.msg import PointList
from geometry_msgs.msg import Point

import os
import datetime

from utils import *

class Color(enum.Enum):
    WHITE = Segment.WHITE
    YELLOW = Segment.YELLOW
    RED = Segment.RED

class pure_pursuit_controller(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.loginfo("HEYHEYHOHO")
        self.loginfo("Node Name: {}".format(self.node_name))
        
        last_edit_time = os.environ.get("LAST_EDIT_TIME", "UNKNOWN")
        self.logwarn("This code was last edited on {}".format(last_edit_time))
        
        self.header = None
        # Publication
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        # self.pub_path_points = rospy.Publisher("~path_points", PointList, queue_size=1)
        self.pub_follow_point = rospy.Publisher("~follow_point", Vector2D, queue_size=1)
        
        # Subscriptions
        self.sub_seglist_filtered = rospy.Subscriber("~seglist_filtered", SegmentList, self.new_segments_received, queue_size=1)
        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.new_pose_received, queue_size=1)
        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        buffer_length_fallback = 50
        self.buffer_length = self.setupParameter("~buffer_length", buffer_length_fallback) # keep 50 points at a time for each color.
        self.points = collections.defaultdict(lambda: collections.deque(maxlen=self.buffer_length))
        
        lookahead_dist_fallback = 1.0
        self.lookahead_dist = self.setupParameter("~lookahead_dist", lookahead_dist_fallback)
        
        max_speed_fallback = 0.5
        self.v_max = self.setupParameter("~v_max", max_speed_fallback)
        
        delta_t_fallback = 0.5
        self.delta_t = self.setupParameter("~delta_t", delta_t_fallback)   
        self.car_command_timer = rospy.Timer(rospy.Duration.from_sec(self.delta_t), self.update_car_command)
        
        offset_fallback = 0.15
        self.offset = self.setupParameter("~offset", offset_fallback)


        self.points_lock = Lock()
   
        self.v = 0
        self.omega = 0

        self.plots_log_file = "brigitte_plots.csv"
        with open(self.plots_log_file, "w") as f:
            f.write("v,omega,d,phi\n")

        self.loginfo("Initialized")
    
    def custom_shutdown(self):
        self.loginfo("Shutting down...")

        # Stop listening
        self.sub_seglist_filtered.unregister()
        self.sub_lane_pose.unregister()
        
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
            color = Color(segment.color)
            assert color in [Color.RED, Color.YELLOW, Color.WHITE]
            with self.points_lock:
                self.points[color].extend(segment.points)

    def new_pose_received(self, lane_pose_message):
        d = lane_pose_message.d
        phi = lane_pose_message.phi
        self.loginfo("Current state: v={}, omega={} d={}, phi={}".format(self.v, self.omega, d, phi))
        with open(self.plots_log_file, "a") as f:
            f.write("{},{},{},{}\n".format(self.v,self.omega,d,phi))

    def update_car_command(self, timer_event):
        self.logdebug("updating car command")
        
        # self.update_past_path_point_coordinates(timer_event)
        # self.publish_path_points()

        self.loginfo("Points: {}".format({color.name: len(values) for color, values in self.points.items()}))
        
        if not self.has_points(Color.YELLOW) and not self.has_points(Color.WHITE):
            self.logwarn("NO POINTS")
            if self.v == 0 and self.omega == 0:
                self.logwarn("Robot is immobile and can't see any lines.")
                self.send_car_command(0.05, 0)
            else:
                self.logwarn("Can't see any lines. Proceeding with same velocity and heading as before (v={}, omega={})".format(self.v, self.omega))
                self.send_car_command(self.v, self.omega)
            return

        ## NOTE: unused, was previously only considering yellow points.
        # elif self.has_points(Color.YELLOW):
        #     self.loginfo("YELLOW")
        #     # best_yellow_point = self.find_point_closest_to_lookahead_distance(Color.YELLOW)
        #     # target = best_yellow_point
        #     centroid_yellow = self.find_centroid(Color.YELLOW)
        #     target = centroid_yellow
        #     target[1] -= self.offset # shifted to the right.
        else:
            # NOTE: unused, was previously doing mean of centroids.
            # centroid_yellow = self.find_centroid(Color.YELLOW)
            # centroid_white = self.find_centroid(Color.WHITE)
            # target = (centroid_white + centroid_yellow) / 2

            # NOTE: unused, was previously doing mean of best points.
            # best_white_point = self.find_point_closest_to_lookahead_distance(Color.WHITE)
            # best_yellow_point = self.find_point_closest_to_lookahead_distance(Color.YELLOW)
            # target = (best_white_point + best_yellow_point) / 2
            
            if self.point_count(Color.YELLOW) > self.point_count(Color.WHITE):
                centroid_yellow = self.find_centroid(Color.YELLOW)
                target = centroid_yellow
                target[1] -= self.offset # shifted to the right.
            else:
                centroid_white = self.find_centroid(Color.WHITE)
                target = centroid_white
                target[1] += self.offset # shift to the left.

        # elif not self.has_points(Color.YELLOW) and self.has_points(Color.WHITE):
        #     self.logwarn("WHITE")
        #     # best_white_point = self.find_point_closest_to_lookahead_distance(Color.WHITE)
        #     white_centroid = self.find_centroid(Color.WHITE)
        #     # assume the white line is always on the right.
        #     target = white_centroid
        #     target[1] += self.offset * 3 # shifted to the left.

        self.logdebug("Target: {}".format(target))
        self.target = target
        target_msg = Vector2D()
        target_msg.x = target[0]
        target_msg.y = target[1]
        self.pub_follow_point.publish(target_msg)

        hypothenuse = np.sqrt(target.dot(target))
        sin_alpha = target[1] / hypothenuse
        
        min_speed = 0.1
        max_speed = 1.0

        # TODO: maybe play around with changing V depending on sin_alpha.
        v = self.v_max * (1 - abs(sin_alpha))
        v = np.clip(v, min_speed, max_speed)

        omega = 2 * sin_alpha / self.lookahead_dist
        self.send_car_command(v, omega)

        # clear the points we have stored.
        self.clear_points()


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
        return point_to_np(best_point)
    
    def has_points(self, color=None):
        return self.point_count(color) != 0

    def point_count(self, color=None):
        with self.points_lock:
            if color is None:
                return sum(len(values) for values in self.points.values())
            return len(self.points[color])

    def clear_points(self):
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
            x = np.mean([p.x for p in points_to_average])
            y = np.mean([p.y for p in points_to_average])
        return np.asarray([x, y])
        # centroid = Point(x=x_centroid, y=y_centroid)
        # return centroid
        
    def send_car_command(self, v, omega):
        car_control_msg = Twist2DStamped()
        if self.header is not None:
            car_control_msg.header = self.header

        if np.isnan(v) or np.isnan(omega):
            self.logwarn("NAN car_command!")
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

    def setupParameter(self, param_name, default_value):
        # self.logwarn("USING DEFAULT VALUE OF PARAMETER {}".format(param_name))
        # TODO: figure out how to fix this.
        # value = default_value
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)   # Write to parameter server for transparancy
        self.loginfo("{} = {} ".format(param_name, value))
        return value

if __name__ == "__main__":
    rospy.init_node("pure_pursuit_controller_node", anonymous=False, log_level=rospy.INFO)  # adapted to sonjas default file

    lane_control_node = pure_pursuit_controller()
    rospy.spin()
