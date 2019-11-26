#!/usr/bin/env python
from enum import Enum
import rospy
from std_msgs.msg import String, ColorRGBA #Imports msg
from duckietown_msgs.msg import Segment, SegmentList, Vector2D
from geometry_msgs.msg import Point, Point32, Polygon
from visualization_msgs.msg import Marker, MarkerArray



class DuckieBotVisualizer(object):
    def __init__(self):
        # Save the name of the node
        # self.node_name = rospy.get_name()
        self.node_name = "duckiebot_visualizer_local"
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Read parameters
        self.veh_name = self.setupParameter("~veh_name","megaman")
        
        # Setup publishers
        # self.pub_timestep = self.setupParameter("~pub_timestep",1.0)
        self.pub_seg_list = rospy.Publisher("~segment_list_markers",MarkerArray,queue_size=1)
        self.pub_seg_list_filtered = rospy.Publisher("~filtered_segment_list_markers",MarkerArray,queue_size=1)
        
        self.pub_follow_point = rospy.Publisher("~follow_point_markers",Marker,queue_size=1)

        self.pub_filtered_yellow_points_markers = rospy.Publisher("~filtered_yellow_points_markers", MarkerArray, queue_size=1)
        self.pub_filtered_white_points_markers  = rospy.Publisher("~filtered_white_points_markers",  MarkerArray, queue_size=1)


        # Create a timer that calls the cbTimer function every 1.0 second
        # self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)

        self.seg_color_dict = dict()
        self.seg_color_dict[Segment.WHITE] = ColorRGBA(r=1.0,g=1.0,b=1.0,a=1.0)
        self.seg_color_dict[Segment.YELLOW] = ColorRGBA(r=1.0,g=1.0,b=0.0,a=1.0)
        self.seg_color_dict[Segment.RED] = ColorRGBA(r=1.0,g=0.0,b=0.0,a=1.0)

        # Setup subscriber
        self.sub_seg_list = rospy.Subscriber("~segment_list", SegmentList, self.cbSegList)
        self.sub_filtered_seg_list = rospy.Subscriber("~segment_list_filtered", SegmentList, self.cbSegListFiltered)
        self.sub_follow_point = rospy.Subscriber("~follow_point", Vector2D, self.viewFollowPoint )
        
        self.sub_filtered_yellow_points = rospy.Subscriber("~filtered_yellow_points", Polygon, self.view_filtered_yellow_points)
        self.sub_filtered_white_points  = rospy.Subscriber("~filtered_white_points",  Polygon, self.view_filtered_white_points)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def cbSegList(self,seg_list_msg):
        marker_array = MarkerArray()
        marker_array.markers.append(self.segList2Marker(seg_list_msg))
        # rospy.loginfo("[%s] publishing %s marker."%(self.node_name,len(marker_array.markers)))
        self.pub_seg_list.publish(marker_array)

    def cbSegListFiltered(self,seg_list_msg):
        marker_array = MarkerArray()
        marker_array.markers.append(self.segList2Marker(seg_list_msg))
        # rospy.loginfo("[%s] publishing %s marker."%(self.node_name,len(marker_array.markers)))
        self.pub_seg_list_filtered.publish(marker_array)
        
    def viewFollowPoint(self,follow_point_msg):
        """ Used to visualize the follow poing on pure-pursuit
        """
        marker = Marker()
        marker.header.frame_id = self.veh_name
        marker.ns = self.veh_name + "/follow_point"
        marker.id = 0 
        marker.action = Marker.ADD
        marker.type = Marker.SPHERE
        marker.lifetime = rospy.Duration.from_sec(5.0)
        marker.pose.position.z = 0 
        marker.pose.position.x = follow_point_msg.x
        marker.pose.position.y = follow_point_msg.y
        marker.color.a = 1.0 
        marker.scale.x = 0.2 
        marker.scale.y = 0.2 
        marker.scale.z = 0.1 
        marker.color.r = 0 
        marker.color.g = 1 
        marker.color.b = 0 
        self.pub_follow_point.publish(marker)

    def view_filtered_yellow_points(self, filtered_points_msg):
        # print("RECEIVED YELLOW POINTS.", len(filtered_points_msg.points))
        marker_array = MarkerArray()
        marker = self.pointList2Marker(filtered_points_msg, color="YELLOW")
        marker_array.markers.append(marker)
        self.pub_filtered_yellow_points_markers.publish(marker_array)

    def view_filtered_white_points(self, filtered_points_msg):
        # print("RECEIVED WHITE POINTS:", len(filtered_points_msg.points))
        marker_array = MarkerArray()
        marker = self.pointList2Marker(filtered_points_msg, color="WHITE")
        marker_array.markers.append(marker)
        self.pub_filtered_white_points_markers.publish(marker_array)

    def segList2Marker(self,seg_list_msg):
        marker = Marker()
        marker.header.frame_id = self.veh_name
        marker.header.stamp = seg_list_msg.header.stamp
        marker.ns = self.veh_name + "/line_seg"
        marker.id = 0
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration.from_sec(5.0)
        marker.type = Marker.LINE_LIST
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02
        for seg in seg_list_msg.segments:
            # point_start = seg.points[0]
            # point_end = seg.points[1]
            marker.points.append(seg.points[0])
            marker.points.append(seg.points[1])
            color = self.seg_color_dict[seg.color]
            marker.colors.append(color)
            marker.colors.append(color)

        # rospy.loginfo("[%s] Number of points %s" %(self.node_name,len(marker.points)))
        return marker


    def pointList2Marker(self, point_list_msg, color="WHITE"):
        marker = Marker()
        marker.header.frame_id = self.veh_name
        # marker.header.stamp = seg_list_msg.header.stamp
        marker.ns = self.veh_name + "/line_seg"
        marker.id = 0
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration.from_sec(5.0)
        marker.type = Marker.LINE_LIST
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02

        points = point_list_msg.points

        def path_segments(points):
            def distance_to_robot(point):
                return point.x ** 2 + point.y ** 2

            previous = None
            for point in sorted(points, key=distance_to_robot):
                if previous is None:
                    previous = point
                else:
                    seg = Segment()
                    seg.points.append(previous)
                    seg.points.append(point)
                    seg.color = Segment.WHITE if color == "WHITE" else Segment.YELLOW
                    yield seg
                    previous = point

        for seg in path_segments(points):
            # point_start = seg.points[0]
            # point_end = seg.points[1]
            marker.points.append(seg.points[0])
            marker.points.append(seg.points[1])
            color = self.seg_color_dict[seg.color]
            marker.colors.append(color)
            marker.colors.append(color)

        # rospy.loginfo("[%s] Number of points %s" %(self.node_name,len(marker.points)))
        return marker



    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('duckiebot_visualizer', anonymous=False)

    # Create the NodeName object
    node = DuckieBotVisualizer()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()
