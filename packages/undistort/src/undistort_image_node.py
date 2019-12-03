#!/usr/bin/env python
from enum import Enum
import rospy
from std_msgs.msg import String, ColorRGBA #Imports msg
from duckietown_msgs.msg import Segment, SegmentList, Vector2D
from geometry_msgs.msg import Point, Point32, Polygon
from visualization_msgs.msg import Marker, MarkerArray


class UndistortImageNode(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Read parameters
        self.veh_name = self.setupParameter("~veh_name","default")
        
        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('undistort_image_node', anonymous=False)

    # Create the NodeName object
    node = UndistortImageNode()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()
