#!/usr/bin/env python
__author__ ='Raphael Leber'


import rospy
import actionlib


#from darknet_gateway_srvs.srv import OpenPoseGossip




from darknet_gateway_srvs.srv import ObjectsDetectionGateway_Srv as ODG_Srv
from darknet_gateway_srvs.srv import ObjectsDetectionGateway_distSorted_Srv as ODG_distSorted_Srv

from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes, CheckForObjectsAction, CheckForObjectsGoal



from robocup_msgs.msg import Entity2D, Entity2DList
from sensor_msgs.msg import Image

from process.ObjectsDetectionGateway_process import ObjectsDetectionGateway_process as ODG_process




class ObjectsDetectionGateway_node():

    def __init__(self):

        self.ready = 0
        self.image_ready = 0
        self.image_width = 0
        self.image_height = 0

        rospy.init_node('ObjectsDetectionGateway_node', anonymous=False)
        rospy.loginfo("ObjectsDetectionGateway_node init")

        #rospy.Subscriber("/pepper_robot/naoqi_driver/camera/front/image_raw", Image, self.img_callback)
        self.IMAGE_TOPIC_NAME = rospy.get_param("/darknet_ros/subscribers/camera_reading/topic","/videofile/image_raw")

        rospy.Subscriber(self.IMAGE_TOPIC_NAME, Image, self.img_callback)
        rospy.loginfo("ObjectsDetectionGateway_node topics init")

        #declare ros service
        self.opg_Srv = rospy.Service('object_detection_gateway_srv', ODG_Srv, self.ODG_BB_SrvCallback)
        rospy.loginfo("ObjectsDetectionGateway_node object_detection_gateway_srv services init")

        self.opg_distSorted_Srv = rospy.Service('object_detection_gateway_distSorted_srv', ODG_distSorted_Srv, self.ODG_BB_distSorted_SrvCallback )
        rospy.loginfo("ObjectsDetectionGateway_node object_detection_gateway_distSorted_srv services init")

        self._actCheckForObjects = actionlib.SimpleActionClient("/darknet_ros/check_for_objects", CheckForObjectsAction)
        #self._actCheckForObjects.wait_for_server(rospy.Duration(5))
        self._actCheckForObjects.wait_for_server()
        rospy.loginfo("ObjectsDetectionGateway_node actions init")

        self.service_running = 0
        self.ready = 1

        rospy.spin()



    def ODG_BB_SrvCallback(self,req):

        #self.service_running = 1
        #self.image_ready = 0

        odg_process = ODG_process()

        #Get image
        if req.img_file_path == "":
            while not self.image_ready :
                rospy.loginfo("wait image")
                rospy.sleep(0.05)
            msg_img = self.msg_img
        else:
            img_loaded = cv2.imread(req.img_file_path)
            msg_img = self._bridge.cv2_to_imgmsg(img_loaded, encoding="bgr8")

        #Create Goal
        goal = CheckForObjectsGoal()
        goal.id = 1
        goal.image = msg_img

        #Send and wait for results
        self._actCheckForObjects.send_goal(goal)
        self._actCheckForObjects.wait_for_result()
        result = self._actCheckForObjects.get_result()

        #Get Entities from results
        e2D = odg_process.BoundingBoxes_to_Entity2DList( result.bounding_boxes, req.labels )

        #self.service_running = 0

        return e2D


    def ODG_BB_distSorted_SrvCallback(self,req):
        odg_process = ODG_process()
        e2D = self.ODG_BB_SrvCallback(req)

        score_list, entityList, pitch_list, yaw_list = odg_process.get_e2D_distSorted_withAngles(e2D, self.image_width, self.image_height)

        e2D.entity2DList = entityList

        return  {   'entities': e2D, #{'header': e2D.header, 'entity2DList': entityList }
                    'pitch_list': pitch_list,
                    'yaw_list' : yaw_list,
                    'score_list' : score_list
                }



    def img_callback(self, msg_img):
        if self.ready == 1 :
            #if self.service_running == 0:
            self.image_ready = 1
            self.msg_img = msg_img
            self.image_width = msg_img.width
            self.image_height = msg_img.height


def main():
    #""" main function
    #"""
    node = ObjectsDetectionGateway_node()

if __name__ == '__main__':
    main()
