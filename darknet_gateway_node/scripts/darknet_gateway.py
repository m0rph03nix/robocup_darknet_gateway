#!/usr/bin/env python  
__author__ ='Raphael Leber'


import rospy 
import actionlib

#from darknet_gateway_srvs.srv import OpenPoseGossip




from darknet_gateway_srvs.srv import ObjectsDetectionGateway_Srv as ODG_Srv

from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes, CheckForObjectsAction, CheckForObjectsGoal



from robocup_msgs.msg import Entity2D, Entity2DList
from sensor_msgs.msg import Image

from process.ObjectsDetectionGateway_process import ObjectsDetectionGateway_process as ODG_process




class ObjectsDetectionGateway_node():

    def __init__(self):

        self.ready = 0
        self.image_ready = 0

        rospy.init_node('ObjectsDetectionGateway_node', anonymous=False)
        rospy.loginfo("ObjectsDetectionGateway_node init")

        #rospy.Subscriber("/pepper_robot/naoqi_driver/camera/front/image_raw", Image, self.img_callback)
        self.IMAGE_TOPIC_NAME = rospy.get_param("/darknet_ros/subscribers/camera_reading/topic","/videofile/image_raw") 

        rospy.Subscriber(self.IMAGE_TOPIC_NAME, Image, self.img_callback)
        rospy.loginfo("ObjectsDetectionGateway_node topics init")

        #declare ros service 
        self.opg_Srv = rospy.Service('object_detection_gateway_srv', ODG_Srv, self.ODG_SrvCallback)
        rospy.loginfo("ObjectsDetectionGateway_node services init")

        self._actCheckForObjects = actionlib.SimpleActionClient("/darknet_ros/check_for_objects", CheckForObjectsAction)
        #self._actCheckForObjects.wait_for_server(rospy.Duration(5))
        self._actCheckForObjects.wait_for_server()
        rospy.loginfo("ObjectsDetectionGateway_node actions init")

        self.service_running = 0
        self.ready = 1

        rospy.spin()



    def ODG_SrvCallback(self,req):

        #self.service_running = 1
        #self.image_ready = 0

        odg_process = ODG_process()

        #goal = odg_process.CreateGoalFromImage( self.msg_img )

        while not self.image_ready :
            rospy.loginfo("wait image")
            rospy.sleep(0.05)
            
        goal = CheckForObjectsGoal()
        goal.id = 1
        goal.image = self.msg_img
        self._actCheckForObjects.send_goal(goal)
        self._actCheckForObjects.wait_for_result()

        result = self._actCheckForObjects.get_result()

        e2D = odg_process.BoundingBoxes_to_Entity2DList( result.bounding_boxes, req.labels )

        #self.service_running = 0

        return e2D

    def img_callback(self, msg_img):
        if self.ready == 1 :
            #if self.service_running == 0:
            self.image_ready = 1
            self.msg_img = msg_img       


def main():
    #""" main function
    #"""
    node = ObjectsDetectionGateway_node()

if __name__ == '__main__':
    main()