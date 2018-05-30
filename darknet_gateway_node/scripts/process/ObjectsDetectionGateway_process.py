#! /usr/bin/env python
__author__ = 'Raphael LEBER'

import rospy

from darknet_gateway_srvs.srv import ObjectsDetectionGateway_Srv as ODG_Srv
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from robocup_msgs.msg import Entity2D, Entity2DList



# Node example class.
class ObjectsDetectionGateway_process():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        self.t = 0


    def BoundingBoxes_to_Entity2DList(self, bounding_boxes):
        el = Entity2DList()

        el.header = bounding_boxes.header
        # xxx = bounding_boxes.image_header

        for bounding_box in bounding_boxes :

            e = Entity2D()

            e.label =   bounding_box.Class
            #e.xxx =    bounding_box.probability
            e.pose.x =  (bounding_box.xmin + bounding_box.xmax) / 2
            e.pose.y =  (bounding_box.ymin + bounding_box.ymax) / 2
            e.bounding_box.x =      bounding_box.xmin 
            e.bounding_box.y =      bounding_box.ymin 
            e.bounding_box.width =  (bounding_box.xmax - bounding_box.xmin) 
            e.bounding_box.height = (bounding_box.ymax - bounding_box.ymin) 

            el.entity2DList.append( e )

        return el
        
    
    def CreateGoalFromImage(self):
        
        return 0

