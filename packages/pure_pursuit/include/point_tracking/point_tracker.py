
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
import copy

from .utils import cos_and_sin, rotation, translation_and_rotation
from threading import Lock
from scipy.cluster.vq import kmeans

class PointTracker(object):
    """
    Keeps track of the movement of points over a short period of time.

    displacement in x and y direction:
    x direction is forward in robot frame.
    Y direction is left in the robot frame.
    alpha is the angle from the forward x axis going left, reaching 90 degrees at the Y axis.
    (hence alpha is following the right-hand rule with respect to the Z axis, which points up)
        
    """

    def __init__(self, num_points_to_observe = 10, memory_secs=1.0, max_distance=5.0, max_buffer_size=None):
        """Initialises the PointTrackingNode.
        
        Keyword Arguments:
            num_points_to_observe {int} -- the number of points to return when the property `tracked_points` is accessed.
                Note, this is the number of 'obvservations' we want to make at any given time, which might be different from the
                number of points in the buffer)
                A clustering algorithm (pretty sure we'll use K-means) is used to produce only `num_points_to_observe`
                from the current state of the the buffer, which stores a larger number of (pretty recent) observations.
                (default: {10})
            memory_secs {float} -- the amount of time in seconds a given point should be kept in the buffer for. (default: {1.0})
            max_distance {float} -- the maximum distance from the robot from which a point should be tracked.
                If a point becomes further from the robot than this quantity, it is removed. (default: {3.0})
            max_buffer_length {Optional[int]} --  the maximum number of tracked points at anygiven time. None for infinite.
                Note that the buffer can't get infinitely long in practice, because older points are removed,
                as well as points that get too far from the robot. (default: {None}, meaning no limit to buffer size.)
        """
        self.num_points_to_observe = num_points_to_observe
        self.memory_secs = memory_secs
        self.max_distance = max_distance
        self.max_buffer_length = max_buffer_size
        
        # A lock used to prevent updating the points while it is being updated.
        self.buffer_lock = Lock()
        # a buffer of length max_buffer_length containing tuples of the form (x: float, y: float, timestamp: float)
        # where the timestamp is the time at which the point was placed within the buffer, relative to the first data acquired.
        self.buffer = deque(maxlen=self.max_buffer_length)

        # the stored list of centroids (the result of K-means).
        # will be of length `num_points_to_observe`.
        self.centroids = None
        self.last_update_time = None

    @property
    def tracked_points(self):
        if self.centroids is not None:
            return np.copy(self.centroids)
        else:
            return np.array(dtype=float)

    def add_points(self, points_to_add):
        """Adds points to the 'points' attribute to be tracked.
        This function should be called by subclasses to register a new observation,
        for example, new yellow points, or a new obstacle position.
        
        Arguments:
            points_to_add {List[Tuple[float, float]]} -- the list of points to add.
        """
        with self.buffer_lock:
            current_time = rospy.get_time()
            self.buffer.extend((point[0], point[1], current_time) for point in points_to_add)

    def update_points_callback(self, twist_msg):
        """Update the estimated position of the stored points given the commanded velocities
        
        Arguments:
            twist_msg {twist_msg} -- a message object which contains the tangential (v) and angular (omega) velocities of the robot.
        """

        current_time = rospy.get_time()
        if self.last_update_time is None:
            self.last_update_time = current_time
            return
        with self.buffer_lock:
            # first get rid of points that are too old
            self._remove_old_points(current_time)
            self._remove_distant_points(current_time)
            # update the points position using the received velocity.
            dt = current_time - self.last_update_time
            v = twist_msg.v
            w = twist_msg.omega
            self._update_points_location(dt, v, w)
            # perform a clustering to create the observations.
            self.centroids = self._clustering()

        self.last_update_time = current_time

    def _clustering(self):
        """Perform a K-means clustering on the buffer to obtain the list of observations.
        
        Returns:
            np.array: The centroids of the K-means. (a list of shape of points, at most `self.num_points_to_observe` long)
        """
        if not self.buffer:
            return []
        
        # we start from a random guess of K points from buffer.
        k = min(len(self.buffer), self.num_points_to_observe)
        if self.centroids is not None and len(self.centroids) == k:
            # we can reuse the old centroids as the initial guess to maybe save some computation.
            k = self.centroids
        points = np.array(self.buffer, dtype=float)[..., :2] # don't use the timestamp during k-means.
        centroids, distortion = kmeans(points, k_or_guess=k)
        return centroids

    def _remove_old_points(self, current_time):
        """Removes all the points which are older than `self.memory_secs` seconds.
        
        Args:
            current_time (float): the current time, in seconds.
        """
        num_points = len(self.buffer)
        for i in range(num_points):
            tracked_point = self.buffer.popleft()
            timestamp = tracked_point[2]
            if current_time - timestamp <= self.memory_secs:
                self.buffer.append(tracked_point)

    def _remove_distant_points(self):
        """Removes all the points which are further than `self.max_distance` from the robot.
        """
        num_points = len(self.buffer)
        max_dist_mag = self.max_distance ** 2
        for i in range(num_points):
            tracked_point = self.buffer.popleft()
            x, y, _ = tracked_point
            if (x**2 + y**2) <= max_dist_mag:
                self.buffer.append(tracked_point)   

    def _update_points_location(self, dt, v, w):
        """update the x and y of every point depending on dt, v and w
        
        Arguments:
            dt (float): the time delay elapsed since the last prediction.
            v (float): the commanded linear (tangential) velocity of the robot
            w (float): the commanded angular velocity of the robot
        """
        if not self.buffer:
            # buffer is empty.
            return

        points = np.array(self.buffer, dtype=float)
        
        if w == 0:
            # going in a straight line.
            displacement = dt * v
            points[:, 0] -= displacement
        else:
            angle_along_arc = dt * w
            radius_of_curvature = np.abs(v / w)
            dx = radius_of_curvature * np.sin(angle_along_arc)
            dy = radius_of_curvature * (1 - np.cos(angle_along_arc))
            # print("dx:", dx, "dy:", dy)
            points[:, 0] -= dx
            points[:, 1] -= dy

            rotation_matrix = rotation(angle_along_arc)
            points[:, :2] = np.matmul(points[:, :2], rotation_matrix)

        self.buffer = deque(points.tolist(), maxlen=self.max_buffer_length)   