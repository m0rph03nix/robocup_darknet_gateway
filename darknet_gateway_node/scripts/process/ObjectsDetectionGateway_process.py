#! /usr/bin/env python
__author__ = 'Raphael LEBER'

import rospy
import rospkg
import csv

from copy import deepcopy
from math import pi

from darknet_gateway_srvs.srv import ObjectsDetectionGateway_Srv as ODG_Srv
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from robocup_msgs.msg import Entity2D, Entity2DList


class ObjectsDetectionGateway_process():
    """
    Sub-class for darknet gateway node.
    Process operations on detected objects' bounding box.
    """

    def __init__(self):
        """
        create ObjectsDetectionGateway_process object.
        """
        # Get the data folder path
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("darknet_gateway_node")
        self.data_folder = package_path + "/data/"
        # Get camera FoV
        self.hfov_deg = rospy.get_param("~/camera/hfov_deg", 57.2)
        self.vfov_deg = rospy.get_param("~/camera/vfov_deg", 44.3)

    def Trad(self):
        """
        Open and read the trad.csv file from data folder

        Returns
        -------
        dict
            Dictionnary of labels traduction {darknet class: custom word}
        """
        with open(self.data_folder + 'trad.csv', mode='r') as infile:
            reader = csv.reader(infile)
            words_by_classes = {rows[0]: rows[1] for rows in reader}
        return words_by_classes

    def BoundingBoxes_to_Entity2DList(self, bounding_boxes, asked_words):
        """
        Make the conversion from BoundingBox message to Entity2DList message

        Parameters
        ----------
        bounding_boxes : darknet_ros_msgs.msgs BoundingBoxes
            List of detected objects in darknet_ros format
        asked_words : string[]
            List of objects in which we are interested

        Returns
        -------
        robocup_msgs.msgs Entity2DList
            List of detected objects in custom format

        """
        #Initialise output
        entity2D_list = Entity2DList()
        entity2D_list.header = bounding_boxes.header
        #Get the dictionnary of labels
        words_by_classes = self.Trad()
        rospy.loginfo("classes-words traduction {0}".format(words_by_classes))
        #Through all detected objects
        for bounding_box in bounding_boxes.bounding_boxes :
            # Traduction of the labels found by darknet
            if bounding_box.Class in words_by_classes.keys() :
                word = words_by_classes[bounding_box.Class]
            else :
                word = 'There are no object found'
            rospy.loginfo("{0}".format(word))
            # Filter out label if needed
            if (len(asked_words)==0) or (word in asked_words) :
                entity2D = Entity2D()
                entity2D.label = word
                # Go from bounding box corners to center coordinates
                entity2D.pose.x = (bounding_box.xmin + bounding_box.xmax) / 2
                entity2D.pose.y = (bounding_box.ymin + bounding_box.ymax) / 2
                entity2D.bounding_box.x = bounding_box.xmin
                entity2D.bounding_box.y = bounding_box.ymin
                entity2D.bounding_box.width =  (bounding_box.xmax - bounding_box.xmin)
                entity2D.bounding_box.height = (bounding_box.ymax - bounding_box.ymin)
                # Append to the list
                entity2D_list.entity2DList.append(entity2D)
        #Output
        return entity2D_list


    def get_e2D_distSorted_withAngles(self, entity2D_list, image_width, image_height):
        """
        Get distances score, entities, pitchs and yaws sorted accordingly to the distance score
        (from the closest to the farthest distance)

        Pitch and yaw are computed relativly to the image center.

        Parameters
        ----------
        entity2D_list : robocup_msgs.msgs Entity2DList
            List of detected objects in Entity2D format
        image_width : int
            image width
        image_heigh : int
            image height

        Returns
        -------
        list[4][]
            distsorted_distscore_list, distsorted_entity2D_list, distsorted_pitch_list, distsorted_yaw_list

        """
        #Init
        outputs = []
        dist_score = 0
        #Process all detected objects
        for entity in entity2D_list.entity2DList:
            rospy.loginfo(entity.label)
            # Bounding box center
            x = entity.bounding_box.x
            y = entity.bounding_box.y
            x2 = x*x #sqrt(x*x, y*y)
            # Compute yaw and pitch from bounding box coordinates
            hfov = - self.hfov_deg * pi / 180.0  # Horizontal field of view of the front Pepper Camera
            yaw = (hfov / 2.0) *  (x - image_width / 2.0) / (image_width / 2.0) #Angle from the center of the camera to neck_x
            vfov = - self.vfov_deg * pi / 180.0  # Vertical field of view of the front Pepper Camera
            pitch = (vfov / 2.0) *  (y - image_height / 2.0) / (image_height / 2.0) #Angle from the center of the camera to neck_x
            # Bounding box distance score. Helps to sort detected object from closest to farthest
            distscore = float(x2) / float(entity.bounding_box.width) / float(entity.bounding_box.height)
            outputs.append( [distscore, entity, pitch, yaw] )
        # Sort the output from the closest object to the farthest
        distsorted_outputs = sorted(outputs, key=lambda x: x[0], reverse=True)
        # Outputs
        distsorted_distscore_list = [x[0] for x in distsorted_outputs]
        distsorted_entity2D_list = [x[1] for x in distsorted_outputs]
        distsorted_pitch_list = [x[2] for x in distsorted_outputs]
        distsorted_yaw_list = [x[3] for x in distsorted_outputs]
        return distsorted_distscore_list, distsorted_entity2D_list, distsorted_pitch_list, distsorted_yaw_list

if __name__ == '__main__':
    """
    main
    """
    try:
        o_p = ObjectsDetectionGateway_process()
        print o_p.Trad()['vienna']
    except :#rospy.ROSInterruptException:
        pass
