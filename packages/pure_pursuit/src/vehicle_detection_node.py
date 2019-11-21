#!/usr/bin/env python


import numpy as np
import rospy
from duckietown_msgs.msg import (BoolStamped, FSMState, LanePose,
                                 Segment, SegmentList, StopLineReading,
                                 Twist2DStamped, WheelsCmdStamped, Vector2D)
from sensor_msgs.msg import CompressedImage, Image
# from duckietown_msgs.msg import PointList

import cv2
import os
import datetime
# import test_pure_pursuit

class vehicle_detection_node(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.loginfo("Node Name: {}".format(self.node_name)
        self.last_stamp = rospy.Time.now()

        self.publish_freq = self.setupParam("~publish_freq", 2.0)


        # Publication
        # self.pub_car_cmd = rospy.Publisher("~detected_vehicle", Twist2DStamped, queue_size=1)
        # self.pub_follow_point = rospy.Publisher("~follow_point", Vector2D, queue_size=1)
        
        # Subscriptions
        self.sub_image = rospy.Subscriber("~image", CompressedImage,
                                          self.processImage, buff_size=921600, 
                                          queue_size=1)
        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        self.loginfo("Initialized")

    def processImage(self, img_msg):

        now = rospy.Time.now()
        if now - self.last_stamp < self.publish_duration:
            return
        else:
            self.last_stamp = now

        try:
            image_cv = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            print e

        params = cv2.SimpleBlobDetector_Params()
        params.minArea = self.blobdetector_min_area
        params.minDistBetweenBlobs = self.blobdetector_min_dist_between_blobs
        simple_blob_detector = cv2.SimpleBlobDetector_create(params)
        (detection, corners) = cv2.findCirclesGrid(image_cv,
                                                    self.circlepattern_dims, flags=cv2.CALIB_CB_SYMMETRIC_GRID,
                                                    blobDetector=simple_blob_detector)

        return
    
    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def custom_shutdown(self):
        self.loginfo("Shutting down...")

        # Stop listening
        self.sub_image.unregister()     

        # Send stop command
        self.loginfo("Stopping the robot")
        self.send_car_command(0.0,0.0)

        rospy.sleep(1.0)    #To make sure that it gets published.
        self.loginfo("Shutdown")

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
    rospy.init_node("vehicle_detection_node", anonymous=False, log_level=rospy.INFO)  # adapted to sonjas default file

    vehicle_detection_node = vehicle_detection_node()
    rospy.spin()
