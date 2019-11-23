#!/usr/bin/env python


import numpy as np
import rospy
from duckietown_msgs.msg import (BoolStamped, FSMState, LanePose,
                                 Segment, SegmentList, StopLineReading,
                                 Twist2DStamped, WheelsCmdStamped, Vector2D, VehicleCorners)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Point32


# from duckietown_msgs.msg import PointList

import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
import datetime
# import test_pure_pursuit

class vehicle_detection_node(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        self.loginfo("Node Name: {}".format(self.node_name))
        self.last_stamp = rospy.Time.now()

        # Setup parameters for circlegrid detection
        self.publish_freq = self.setupParam("~publish_freq", 2.0)
        self.circlepattern_dims = tuple(self.setupParam('~circlepattern_dims/data', [7, 3]))
        self.blobdetector_min_area = self.setupParam('~blobdetector_min_area', 10)
        self.blobdetector_min_dist_between_blobs = self.setupParam('~blobdetector_min_dist_between_blobs', 2)
        self.publish_circles = self.setupParam('~publish_circles', True)

        self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)


        # Publication
        # self.pub_car_cmd = rospy.Publisher("~detected_vehicle", Twist2DStamped, queue_size=1)
        # self.pub_follow_point = rospy.Publisher("~follow_point", Vector2D, queue_size=1)
        self.pub_detection = rospy.Publisher("~detection",
                                             BoolStamped, queue_size=1)
        self.pub_corners = rospy.Publisher("~corners",
                                           VehicleCorners, queue_size=1)
        self.pub_circlepattern_image = rospy.Publisher("~circlepattern_image",
                                                       Image, queue_size=1)
        self.pub_time_elapsed = rospy.Publisher("~detection_time",
                                                Float32, queue_size=1)

        # Subscriptions
        self.sub_image = rospy.Subscriber("~image", CompressedImage,
                                          self.processImage, buff_size=921600, 
                                          queue_size=1)
        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        self.loginfo("Initialized")

    def processImage(self, image_msg):

        now = rospy.Time.now()
        if now - self.last_stamp < self.publish_duration:
            return
        else:
            self.last_stamp = now


        vehicle_detected_msg_out = BoolStamped()
        vehicle_corners_msg_out = VehicleCorners()

        try:
            image_cv = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # TODO we are only finding a 3x1 circle grid
        start = rospy.Time.now()
        params = cv2.SimpleBlobDetector_Params()
        params.minArea = self.blobdetector_min_area
        params.minDistBetweenBlobs = self.blobdetector_min_dist_between_blobs
        simple_blob_detector = cv2.SimpleBlobDetector_create(params)
        (detection, corners) = cv2.findCirclesGrid(image_cv,
                                                    (3,1), flags=cv2.CALIB_CB_SYMMETRIC_GRID,
                                                    blobDetector=simple_blob_detector)
        

        vehicle_detected_msg_out.data = detection
        if(vehicle_detected_msg_out.data is not False):
            self.loginfo(">>>>>>>>>>>>>>>>>>>>>Published on detection")
        self.pub_detection.publish(vehicle_detected_msg_out)
        if detection:
            # print(corners)
            points_list = []
            for point in corners:
                corner = Point32()
                # print(point[0])
                corner.x = point[0, 0]
                # print(point[0,1])
                corner.y = point[0, 1]
                corner.z = 0
                points_list.append(corner)
            vehicle_corners_msg_out.header.stamp = rospy.Time.now()
            vehicle_corners_msg_out.corners = points_list
            vehicle_corners_msg_out.detection.data = detection
            vehicle_corners_msg_out.H = self.circlepattern_dims[1]
            vehicle_corners_msg_out.W = self.circlepattern_dims[0]
            self.pub_corners.publish(vehicle_corners_msg_out)
        elapsed_time = (rospy.Time.now() - start).to_sec()
        self.pub_time_elapsed.publish(elapsed_time)
        if self.publish_circles:
            cv2.drawChessboardCorners(image_cv,
                                        self.circlepattern_dims, corners, detection)
            image_msg_out = self.bridge.cv2_to_imgmsg(image_cv, "bgr8")
            self.pub_circlepattern_image.publish(image_msg_out)

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
