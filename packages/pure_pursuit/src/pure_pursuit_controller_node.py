#!/usr/bin/env python
import math
import time

import numpy as np
import rospy
from controller import Controller
from duckietown_msgs.msg import (BoolStamped, FSMState, LanePose,
                                 StopLineReading, Twist2DStamped,
                                 WheelsCmdStamped, SegmentList)


class pure_pursuit_controller(object):

    def __init__(self):
        self.node_name = rospy.get_name()

        # Publication
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        
        # Subscriptions
        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.handle_pose, queue_size=1)
        self.sub_seglist_filtered = rospy.Subscriber("~seglist_filtered", SegmentList, self.new_segments_received, queue_size=1)

        self.velocity_to_m_per_s = 0.67
        self.omega_to_rad_per_s = 0.45 * 2 * math.pi
        self.omega_max_abs = 100

        self.loginfo("Initialized")
    
    def new_segments_received(self, inlier_segments_msg):
        segments = inlier_segments_msg.segments
        self.loginfo("%d new segments received." % len(segments))

    def handle_pose(self, pose_msg):
        self.loginfo("New pose received!")
        # TODO: don't know what to do / how to handle the lane pose exactly.

        self.header = pose_msg.header

        self.send_car_command(0, 0)
        

    def send_car_command(self, v, omega):
        car_control_msg = Twist2DStamped()
        car_control_msg.header = self.header

        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0

        self.loginfo("Sending car command: v: %d omega: %d" % v, omega)
        self.pub_car_cmd.publish(car_control_msg)
        

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value)   # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." % self.node_name)

        # Stop listening
        self.sub_lane_reading.unregister()
        self.sub_obstacle_avoidance_pose.unregister()
        self.sub_obstacle_detected.unregister()
        self.sub_intersection_navigation_pose.unregister()
        # self.sub_parking_pose.unregister()
        # self.sub_fleet_planning_pose.unregister()
        # self.sub_fleet_planning_lane_following_override_active.unregister()
        # self.sub_implicit_coord_pose.unregister()
        # self.sub_implicit_coord_velocity_limit_active.unregister()
        self.sub_wheels_cmd_executed.unregister()
        self.sub_actuator_limits.unregister()
        self.sub_switch.unregister()
        self.sub_fsm_mode.unregister()

        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)

        rospy.sleep(0.5)    #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)

if __name__ == "__main__":
    rospy.init_node("pure_pursuit_controller_node", anonymous=False)  # adapted to sonjas default file

    lane_control_node = pure_pursuit_controller()
    rospy.spin()
