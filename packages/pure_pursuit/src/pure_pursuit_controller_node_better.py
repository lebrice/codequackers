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
from geometry_msgs.msg import Point, Polygon, Point32

import os
import datetime

from utils import *
from point_tracking import PointTracker

class Color(enum.Enum):
    WHITE = Segment.WHITE
    YELLOW = Segment.YELLOW
    RED = Segment.RED

    
class pure_pursuit_controller_node_better(object):

    def __init__(self):
        self.node_name = "pure_pursuit_controller_node_better"
        self.loginfo("Node Name: {}".format(self.node_name))
                
        self.header = None
        
        # Publication
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)


        self.pub_follow_point = rospy.Publisher("~follow_point", Vector2D, queue_size=1)

        # TODO: publish the path points markers to be viewed in RVIZ.
        self.pub_filtered_yellow_points = rospy.Publisher("~filtered_yellow_points", Polygon, queue_size=1)
        self.pub_filtered_white_points  = rospy.Publisher("~filtered_white_points",  Polygon, queue_size=1)
        
        
        # Filter Parameters:


        yellow_num_points_fallback = 5
        yellow_num_points = self.setupParameter("~yellow_num_points", yellow_num_points_fallback)
        yellow_points_memory_secs_fallback = 1.0
        yellow_points_memory_secs = self.setupParameter("~yellow_points_memory_secs", yellow_points_memory_secs_fallback)
        yellow_points_max_distance_fallback = 1.0
        yellow_points_max_distance = self.setupParameter("~yellow_points_max_distance", yellow_points_max_distance_fallback)
        yellow_max_buffer_size_fallback = 300
        yellow_max_buffer_size = self.setupParameter("~yellow_max_buffer_size", yellow_max_buffer_size_fallback)

        white_num_points_fallback = 5
        white_num_points = self.setupParameter("~white_num_points", white_num_points_fallback)
        white_points_memory_secs_fallback = 1.0
        white_points_memory_secs = self.setupParameter("~white_points_memory_secs", white_points_memory_secs_fallback)  
        white_points_max_distance_fallback = 3.0
        white_points_max_distance = self.setupParameter("~white_points_max_distance", white_points_max_distance_fallback)  
        white_max_buffer_size_fallback = 300
        white_max_buffer_size = self.setupParameter("~white_max_buffer_size", white_max_buffer_size_fallback)


        # Trackers for yellow and white points. 
        self.yellow_points_tracker = PointTracker(
            num_points_to_observe=yellow_num_points,
            memory_secs=yellow_points_memory_secs,
            max_distance=yellow_points_max_distance,
            max_buffer_size=yellow_max_buffer_size
        )
        self.white_points_tracker  = PointTracker(
            num_points_to_observe=white_num_points,
            memory_secs=white_points_memory_secs,
            max_distance=white_points_max_distance,
            max_buffer_size=white_max_buffer_size
        )

        # Subscriptions
        self.sub_seglist_filtered = rospy.Subscriber("~seglist_filtered", SegmentList, self.new_segments_received, queue_size=1)
        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.new_pose_received, queue_size=1)
         # subscribe to the car_cmd, tu update the location of the points in the trackers' buffers.
        # TODO: subscribe to the forward kinematics node, if that can help.
        # TODO: make sure there isn't some weird feedback happening with the published topic of the same name.
        self.sub_update_trackers = rospy.Subscriber("~car_cmd", Twist2DStamped, self.update_trackers_callback)

        # Parameters:
        lookahead_dist_fallback = 1.0
        self.lookahead_dist = self.setupParameter("~lookahead_dist", lookahead_dist_fallback)
        
        max_speed_fallback = 0.5
        self.v_max = self.setupParameter("~v_max", max_speed_fallback)
        
        delta_t_fallback = 0.5
        self.delta_t = self.setupParameter("~delta_t", delta_t_fallback)   
        self.car_command_timer = rospy.Timer(rospy.Duration.from_sec(self.delta_t), self.update_car_command)
        
        offset_fallback = 0.15
        self.offset = self.setupParameter("~offset", offset_fallback)

        self.v = 0
        self.omega = 0

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)      
        self.loginfo("Initialized")
    
    def update_trackers_callback(self, twist_msg):
        """Update the estimated position of the stored points given the commanded velocities
        
        Arguments:
            twist_msg {twist_msg} -- a message object which contains the tangential (v) and angular (omega) velocities of the robot.
        """
        # self.loginfo("Received new twist message: {}".format(twist_msg))
        
        self.yellow_points_tracker.update_points_callback(twist_msg)
        self.white_points_tracker.update_points_callback(twist_msg)
        self.loginfo("White point tracker has {} points and Yellow PointTracker has {}.".format(self.white_points_tracker.buffer_length, self.yellow_points_tracker.buffer_length))

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

    def publish_filtered_yellow_points(self, yellow_points):
        # Publish the filtered yellow points:
        filtered_points_msg = Polygon()
        filtered_points_msg.points = [
            Point32(p[0], p[1], 0.) for p in yellow_points
        ]
        self.logdebug("Publishing {} new yellow points".format(len(filtered_points_msg.points)))
        self.pub_filtered_yellow_points.publish(filtered_points_msg)

    def publish_filtered_white_points(self, white_points):
        # publish the filtered white points:
        filtered_points_msg = Polygon()
        filtered_points_msg.points = [
            Point32(p[0], p[1], 0.) for p in white_points
        ]
        self.logdebug("Publishing {} new white points".format(len(filtered_points_msg.points)))
        self.pub_filtered_white_points.publish(filtered_points_msg)


    def new_segments_received(self, inlier_segments_msg):
        self.header = inlier_segments_msg.header
        segments = inlier_segments_msg.segments

        points = collections.defaultdict(list)
        for i, segment in enumerate(segments):
            color = Color(segment.color)
            assert color in [Color.RED, Color.YELLOW, Color.WHITE]
            points[color].extend(segment.points)
        
        self.yellow_points_tracker.add_points(points[Color.YELLOW])
        self.white_points_tracker.add_points(points[Color.WHITE])
       

    def new_pose_received(self, lane_pose_message):
        d = lane_pose_message.d
        phi = lane_pose_message.phi
        self.logdebug("Current state: v={}, omega={} d={}, phi={}".format(self.v, self.omega, d, phi))


    def update_car_command(self, timer_event):
        
        yellow_points = np.copy(self.yellow_points_tracker.tracked_points)
        white_points = np.copy(self.white_points_tracker.tracked_points)

        self.publish_filtered_yellow_points(yellow_points)
        self.publish_filtered_white_points(white_points)

        if len(yellow_points) == 0 and len(white_points) == 0:
            self.logwarn("NO POINTS")
            if self.v == 0 and self.omega == 0:
                self.logwarn("Robot is immobile and can't see any lines.")
                self.send_car_command(0.05, 0)
            else:
                self.logwarn("Can't see any lines. Proceeding with same velocity and heading as before (v={}, omega={})".format(self.v, self.omega))
                self.send_car_command(self.v, self.omega)
            return
        else:
            # NOTE: unused, was previously doing mean of centroids.
            # centroid_yellow = self.find_centroid(Color.YELLOW)
            # centroid_white = self.find_centroid(Color.WHITE)
            # target = (centroid_white + centroid_yellow) / 2

            # NOTE: unused, was previously doing mean of best points.
            # best_white_point = self.find_point_closest_to_lookahead_distance(Color.WHITE)
            # best_yellow_point = self.find_point_closest_to_lookahead_distance(Color.YELLOW)
            # target = (best_white_point + best_yellow_point) / 2
            if len(yellow_points) > len(white_points):
                # centroid_yellow = self.find_centroid(yellow_points)
                # target = centroid_yellow
                target = self.find_point_closest_to_lookahead_distance(yellow_points)
                target[1] -= self.offset # shifted to the right.
            else:
                centroid_white = self.find_centroid(white_points)
                target = centroid_white
                # target = self.find_point_closest_to_lookahead_distance(white_points)
                target[1] += self.offset # shift to the left.

        # elif not self.has_points(Color.YELLOW) point_to_npand self.has_points(Color.WHITE):
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
        cos_alpha = target[0] / hypothenuse
        min_speed = 0.1
        max_speed = 1.0

        # TODO: maybe play around with changing V depending on sin_alpha.
        v = self.v_max * (cos_alpha ** 2)
        v = np.clip(v, min_speed, max_speed * (0.5 if len(yellow_points) == 0 else 1.0))

        omega = 2 * sin_alpha / self.lookahead_dist
        self.send_car_command(v, omega)


    def find_point_closest_to_lookahead_distance(self, points):
        lookahead_mag = self.lookahead_dist ** 2
        distances = np.sum(points ** 2, axis=0)
        min_index = np.argmin(distances - lookahead_mag, axis=0)
        best_point = points[min_index]
        return best_point
    
    def has_points(self, color=None):
        return self.point_count(color) != 0

    def point_count(self, color=None):
        with self.points_lock:
            if color is None:
                return sum(len(values) for values in self.points.values())
            return len(self.points[color])

    def clear_points(self):
        for color, point_list in self.points.items():
                point_list.clear()

    def find_centroid(self, points_to_average):
        return np.mean(points_to_average, axis=0)

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

    lane_control_node = pure_pursuit_controller_node_better()
    rospy.spin()
