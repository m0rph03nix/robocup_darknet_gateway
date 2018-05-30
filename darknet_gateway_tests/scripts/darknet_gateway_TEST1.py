#!/usr/bin/env python  
__author__ ='Raphael Leber'


import rospy 
import actionlib

#from darknet_gateway_srvs.srv import OpenPoseGossip



from darknet_gateway_srvs.srv import ObjectsDetectionGateway_Srv as ODG_Srv

from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes, CheckForObjectsAction



from robocup_msgs.msg import Entity2D, Entity2DList
from sensor_msgs.msg import Image

from darknet_gateway_srvs.srv import ObjectsDetectionGateway_Srv as ODG_Srv




class ObjectsDetectionGateway_tests():

    def __init__(self):

        rospy.init_node('ObjectsDetectionGateway_tests', anonymous=False)
        rospy.loginfo("ObjectsDetectionGateway_tests init")

        self.test1()

        rospy.spin()



    def test1(self):

        try:
            labels = ["redbull"]
            test_darknet_gateway_srv = rospy.ServiceProxy('object_detection_gateway_srv', ODG_Srv)
            resp = test_darknet_gateway_srv(labels)
            rospy.loginfo("ObjectsDetectionGateway_tests services OK")
            print "service:" + str(resp.entities)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e



def main():
    #""" main function
    #"""
    node = ObjectsDetectionGateway_tests()

if __name__ == '__main__':
    main()