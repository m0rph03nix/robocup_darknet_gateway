#!/usr/bin/env python
__author__ ='Raphael Leber'


import rospy
import actionlib

import cv2
from cv_bridge import CvBridge

from darknet_gateway_srvs.srv import ObjectsDetectionGateway_Srv as ODG_Srv
from darknet_gateway_srvs.srv import ObjectsDetectionGateway_distSorted_Srv as ODG_distSorted_Srv

from darknet_ros_msgs.msg import CheckForObjectsAction, CheckForObjectsGoal
from sensor_msgs.msg import Image

from process.ObjectsDetectionGateway_process import ObjectsDetectionGateway_process as ODG_process


class ObjectsDetectionGateway_node():
    """
    Gateway for object detection with darknet.
    """

    def __init__(self):
        """
        Create and start a ROS node ObjectsDetectionGateway_node
        """
        self.image_ready = 0
        self.image_width = 0
        self.image_height = 0
        #Node initialisation
        rospy.init_node('ObjectsDetectionGateway_node', anonymous=False)
        rospy.loginfo("ObjectsDetectionGateway_node init")
        #Declare ros service
        self.opg_Srv = rospy.Service('object_detection_gateway_srv', ODG_Srv, self.ODG_BB_SrvCallback)
        rospy.loginfo("ObjectsDetectionGateway_node object_detection_gateway_srv services init")
        self.opg_distSorted_Srv = rospy.Service('object_detection_gateway_distSorted_srv', ODG_distSorted_Srv, self.ODG_BB_distSorted_SrvCallback )
        rospy.loginfo("ObjectsDetectionGateway_node object_detection_gateway_distSorted_srv services init")
        self._actCheckForObjects = actionlib.SimpleActionClient("/darknet_ros/check_for_objects", CheckForObjectsAction)
        self._actCheckForObjects.wait_for_server()
        rospy.loginfo("ObjectsDetectionGateway_node actions init")
        #Topic subscribing
        self.image_topic_name = rospy.get_param("/darknet_ros/subscribers/camera_reading/topic","/videofile/image_raw")
        rospy.Subscriber(self.image_topic_name, Image, self.img_callback)
        rospy.loginfo("ObjectsDetectionGateway_node topics init")
        #Node online
        rospy.spin()

    def ODG_BB_SrvCallback(self, req):
        """
        Callback on object_detection_gateway_srv service

        Parameters
        ----------
        req : darknet_gateway_srvs.srv ObjectsDetectionGateway_Srv
            Service request
        """
        #Get image
        if req.img_file_path == "":
            #Load from topic
            while not self.image_ready :
                rospy.loginfo("wait image")
                rospy.sleep(0.05)
            msg_img = self.msg_img
        else:
            #Load from file
            img_loaded = cv2.imread(req.img_file_path)
            msg_img = CvBridge().cv2_to_imgmsg(img_loaded, encoding="bgr8")
        #Create Goal
        goal = CheckForObjectsGoal()
        goal.id = 1
        goal.image = msg_img
        #Send and wait for results
        self._actCheckForObjects.send_goal(goal)
        self._actCheckForObjects.wait_for_result()
        result = self._actCheckForObjects.get_result()
        #Get Entities from results
        odg_process = ODG_process()
        e2D = odg_process.BoundingBoxes_to_Entity2DList( result.bounding_boxes, req.labels )
        #Servie output
        return e2D

    def ODG_BB_distSorted_SrvCallback(self, req):
        """
        Callback on object_detection_gateway_distSorted_srv service

        Parameters
        ----------
        req : darknet_gateway_srvs.srv ObjectsDetectionGateway_distSorted_Srv
            Service request
        """
        # Get entity list
        e2D = self.ODG_BB_SrvCallback(req)
        # Sort the entity list
        odg_process = ODG_process()
        score_list, entityList, pitch_list, yaw_list = odg_process.get_e2D_distSorted_withAngles(e2D, self.image_width, self.image_height)
        # Service outputs
        e2D.entity2DList = entityList
        return  {   'entities': e2D, #{'header': e2D.header, 'entity2DList': entityList }
                    'pitch_list': pitch_list,
                    'yaw_list' : yaw_list,
                    'score_list' : score_list }

    def img_callback(self, msg_img):
        """
        Callback on image topic
        """
        self.image_ready = 1
        self.msg_img = msg_img
        self.image_width = msg_img.width
        self.image_height = msg_img.height


def main():
    """ main function
    """
    node = ObjectsDetectionGateway_node()

if __name__ == '__main__':
    main()
