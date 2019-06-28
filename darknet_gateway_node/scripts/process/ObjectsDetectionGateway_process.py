#! /usr/bin/env python
__author__ = 'Raphael LEBER'

import rospy


from darknet_gateway_srvs.srv import ObjectsDetectionGateway_Srv as ODG_Srv
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

from robocup_msgs.msg import Entity2D, Entity2DList

from copy import deepcopy

from math import pi

import csv
import rospkg

# Node example class.
class ObjectsDetectionGateway_process():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        self.t = 0

        rospack = rospkg.RosPack()

        # get the file path for rospy_tutorials
        package_path=rospack.get_path('darknet_gateway_node')
        self.data_folder = package_path + "/data/"



    def Trad(self):

        with open(self.data_folder + 'trad.csv', mode='r') as infile:
            reader = csv.reader(infile)
            words = {rows[0]: rows[1] for rows in reader}

        return words


    def BoundingBoxes_to_Entity2DList(self, bounding_boxes, labels):
        el = Entity2DList()

        el.header = bounding_boxes.header
        words = self.Trad()

        print words
        print " "

        for bounding_box in bounding_boxes.bounding_boxes :

            if bounding_box.Class in words.keys() :
                word = words[bounding_box.Class]
            else :
                word = 'There are no object found'

            print word

            if len(labels)==0 or word in labels :

                e = Entity2D()

                e.label =   word
                #e.xxx =    bounding_box.probability
                e.pose.x =  (bounding_box.xmin + bounding_box.xmax) / 2
                e.pose.y =  (bounding_box.ymin + bounding_box.ymax) / 2
                e.bounding_box.x =      bounding_box.xmin
                e.bounding_box.y =      bounding_box.ymin
                e.bounding_box.width =  (bounding_box.xmax - bounding_box.xmin)
                e.bounding_box.height = (bounding_box.ymax - bounding_box.ymin)

                el.entity2DList.append( e )

        return el


    def get_e2D_distSorted_withAngles(self, e2D, image_width, image_height):
        """
        Get pitch and yaw of each detected object according to the center of the camera images
        max pitch =
        """
        scoreList = []
        score = 0

        for entity in e2D.entity2DList:
            rospy.loginfo(entity.label)
            x = entity.bounding_box.x
            y = entity.bounding_box.y
            x2 = x*x #sqrt(x*x, y*y)

            HFov = - 57.2 * pi / 180.0  # Horizontal field of view of the front Pepper Camera
            Yaw = (HFov / 2.0) *  (x - image_width / 2.0) / (image_width / 2.0) #Angle from the center of the camera to neck_x
            VFov = - 44.3 * pi / 180.0  # Vertical field of view of the front Pepper Camera
            Pitch = (VFov / 2.0) *  (y - image_height / 2.0) / (image_height / 2.0) #Angle from the center of the camera to neck_x

            score = float(x2) / float(entity.bounding_box.width) / float(entity.bounding_box.height)
            scoreList.append( [score, entity, Pitch, Yaw] )
            #find index du min de scoreList

        sortedScoreList = sorted(scoreList, key=lambda attributes: attributes, reverse=True)

        scoreList   = map(lambda x: x[0], sortedScoreList)
        entityList  = map(lambda x: x[1], sortedScoreList)
        PitchList   = map(lambda x: x[2], sortedScoreList)
        YawList     = map(lambda x: x[3], sortedScoreList)

        return scoreList, entityList, PitchList, YawList


    def BoundingBoxes_Filtered(self, bounding_boxes, labels):
        bbs = BoundingBoxes()

        bbs.header = bounding_boxes.header
        words = self.Trad()

        print words
        print " "

        for bounding_box in bounding_boxes.bounding_boxes :

            if bounding_box.Class in words.keys() :
                word = words[bounding_box.Class]
            else :
                word = 'There are no object found'

            print word

            if len(labels)==0 or word in labels :

                bbs.bounding_boxes.append( bounding_box )

        return bbs


    def CreateGoalFromImage(self):

        return 0


if __name__ == '__main__':
    try:
        o_p = ObjectsDetectionGateway_process()
        print o_p.Trad()['vienna']

    except :#rospy.ROSInterruptException:
        pass
