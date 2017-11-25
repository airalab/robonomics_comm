# -*- coding: utf-8 -*-
#
# Robonomics liability execution node.
#

from robonomics_liability.msg import Liability
import rospy, rosbag

class Executor:
    def __init__(self):
        '''
            Robonomics liability node initialisation.
        '''
        rospy.init_node('robonomics_liability_executor')


    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()
