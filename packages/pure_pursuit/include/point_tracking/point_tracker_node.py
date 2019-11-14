
#!/usr/bin/env python
from cv_bridge import CvBridge
from duckietown_msgs.msg import SegmentList, LanePose, BoolStamped, Twist2DStamped, FSMState
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
import json

import rospy
from collections import deque
from abc import ABCMeta, abstractmethod
import copy

from .utils import cos_and_sin
from threading import Lock

class PointTrackingNode():
    """
    Keeps track of the movement of points over a short period of time.

    displacement in x and y direction:
    x direction is forward in robot frame.
    Y direction is left in the robot frame.
    alpha is the angle from the forward x axis going left, reaching 90 degrees at the Y axis.
    (hence alpha is following the right-hand rule with respect to the Z axis, which points up)
        
    """
    __metaclass__ = ABCMeta

    def __init__(self, memory_secs=1.0, max_distance=5.0, max_buffer_length=None):
        # the amount of time a given point should be kept in memory for.
        self.memory_secs = memory_secs
        # the maximum distance from the robot from which a point should be tracked.
        # If a point becomes further from the robot than this quantity, it is removed.
        self.max_distance = max_distance
        # the maximum number of tracked points at anygiven time. None for infinite.
        self.max_buffer_length = max_buffer_length
        
        self.sub_velocity = rospy.Subscriber("~car_cmd", Twist2DStamped, self.update_points)
        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.update_pose)

        # A lock used to prevent updating the points while it is being updated.
        self.points_lock = Lock()
        # a buffer of length max_buffer_length containing tuples of the form (x: float, y: float, timestamp: float)
        # where the timestamp is the time at which the point was placed within the buffer, relative to the first data acquired.
        self.buffer = deque(maxlen=self.max_buffer_length)

        self.last_update_time = None

    @property
    def tracked_points(self):
        with self.points_lock:
            copy = [(x,y) for (x, y, timestamp) in self.buffer]
        return np.array(copy)

    def add_points(self, points_to_add):
        """Adds points to the 'points' attribute to be tracked.
        
        Arguments:
            points_to_add {List[Point]} -- the list of points to add.
        """
        with self.points_lock:
            current_time = rospy.get_time()
            self.buffer.extend((point[0], point[1], current_time) for point in points_to_add)

    def remove_old_points(self, current_time):
        """Removes all the points which are older than `self.memory_secs` seconds.
        
        
        Arguments:
            current_time {float} -- the current time, in seconds.
        """
        num_points = len(self.buffer)
        for i in range(num_points):
            tracked_point = self.buffer.popleft()
            _, _, timestamp = tracked_point
            if current_time - timestamp <= self.memory_secs:
                self.buffer.append(tracked_point)    


    def remove_distant_points(self, current_time):
        """Removes all the points which are further than `self.max_distance` from the robot.
        
        
        Arguments:
            current_time {float} -- the current time, in seconds.
        """
        num_points = len(self.buffer)
        for i in range(num_points):
            tracked_point = self.buffer.popleft()
            _, _, timestamp = tracked_point
            if current_time - timestamp <= self.memory_secs:
                self.buffer.append(tracked_point)  

    
    def update_points(self, twist_msg):
        """Update the estimated position of the stored points given the commanded velocities
        
        Arguments:
            twist_msg {twist_msg} -- a message object which contains the tangential (v) and angular (omega) velocities of the robot.
        """
        if self.last_update_time is None:
            return
        with self.points_lock:
            current_time = rospy.get_time()
            # first get rid of points that are too old
            self.remove_old_points(current_time)

            # update the points position using the received velocity.
            dt = current_time - self.last_update_time
            v = twist_msg.v
            w = twist_msg.omega
            self.update_points_location(dt, v, w)
        self.last_update_time = current_time

    def update_pose(self, lane_pose_msg):
        """Update the robot's lane pose.

        Arguments:
            lane_pose_msg {[type]} -- [description]
        """
        # TODO: Do we really need the lane pose? Couldn't we just use the robot's initial position as a reference frame, and update the points
        # using the change in heading? 
        self.d      = lane_pose_msg.d
        self.phi    = lane_pose_msg.phi


    # def update_past_path_point_coordinates(self, timer_event):
    #     """
    #     After each timestep, we update the coordinates of the points we stored in the buffer
    #     """
    #     # change in heading: alpha = omega * dt
    #     alpha = self.omega * delta_t
    #     # displacement in the x and y direction.
    #     delta_x = self.v * np.cos(alpha)
    #     delta_y = self.v * np.sin(alpha)
    #     displacement = np.asarray([
    #         delta_x, delta_y
    #     ])
    #     rotation = R(alpha)

    #     def update_point(point):
    #         old_pos = point_to_np(point)
    #         new_pos = old_pos - displacement
    #         new_pos = np.matmul(rotation, new_pos)
    #         point.x = new_pos[0]
    #         point.y = new_pos[1]
    #         return point

    #     with self.points_lock:
    #         for color, old_points_buffer in self.points.items():
    #             self.points[color] = collections.deque([update_point(p) for p in old_points_buffer])
    #     self.logdebug("Updated Points successfully.")


    def _update_points_location(self, dt, v, w):
            """update the x and y of every point depending on dt, v and w
            
            Arguments:
                dt {float} -- [the time delay elapsed since the last prediction.]
                v {[type]} -- [the commanded linear (tangential) velocity of the robot]
                w {[type]} -- [the commanded angular velocity of the robot]
            """
            
            # change in heading: alpha = omega * dt
            alpha = dt * w
            # displacement in x and y direction:
            # x direction is forward in robot frame.
            # Y direction is left in the robot frame.
            # alpha is the angle from the forward x axis going left, reaching 90 degrees at the Y axis.
            # (hence alpha is following the right-hand rule with respect to the Z axis, which points up)
            displacement = v * cos_and_sin(alpha) 
            
            if w == 0:
                # going in a straight line.
                displacement = dt * v * cos_and_sin(self.phi)
                # d is displaced in 'y', and phi stays the same.
                self.d += displacement[1]
            else:
                # calculate the displacement due to omega
                angle_performed_along_arc = dt * w
                tangential_velocity = v
                angular_velocity = w
                radius_of_curvature = np.abs(tangential_velocity / angular_velocity)

                # get the projections of the distance traveled relative to current heading phi.
                dx = radius_of_curvature * np.sin(angle_performed_along_arc)
                dy = radius_of_curvature * (1 - np.cos(angle_performed_along_arc))
                
                
                # This displacement is in the frame with heading phi though.
                # Therefore we need to correct for that as well by rotating it by -phi.
                rotation = R(-self.phi)
                displacement = np.matmul(np.array([dy, dx]), rotation)
                
                # The orientation changed since we moved along the arc.
                # It can easily be shown that the change in phi due to the arc motion
                # is equal to the angle performed along the arc.
                change_in_phi = angle_performed_along_arc
                self.phi += change_in_phi
                self.d += displacement[1]

            self.d += np.random.normal(0, self.sigma_d)
            self.phi += np.random.normal(0, self.sigma_phi)

            # We clip d and phi to stay within the desired maximal range
            self.d = np.clip(self.d, self.d_min, self.d_max)
            self.phi = np.clip(self.phi, self.phi_min, self.phi_max)