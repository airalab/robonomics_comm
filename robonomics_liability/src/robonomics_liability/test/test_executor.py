#!/usr/bin/env python

import unittest, rostest, os, time, rospy, rosbag
from robonomics_lighthouse.msg import Result
from robonomics_liability.msg import Liability
from urllib.parse import urlparse
import ipfsapi
from tempfile import TemporaryDirectory
from std_msgs.msg import *

PKG = 'robonomics_liability'
NAME = 'test_executor'


class TestExecutor(unittest.TestCase):

    def __init__(self, *args):
        rospy.init_node(NAME)
        super(TestExecutor, self).__init__(*args)

        ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfs = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))
        self.test_start_time = time.time()

        self.success = False
        self.incoming_liability = None

    def incoming_liability_handler(self, msg):
        self.incoming_liability = msg
        rospy.loginfo("INCOMING HANDLER: address is %s", self.incoming_liability.address)

    def result_handler(self, result):
        rospy.loginfo("RESULT HANDLER: liability: %s result %s", result.liability, result.result)

        if self.incoming_liability is not None \
                and self.incoming_liability.address == result.liability \
                and self.check_rosbag_is_new_and_has_messages(result):
            self.success = True

    def check_rosbag_is_new_and_has_messages(self, result):
        with TemporaryDirectory() as tmpdir:
            os.chdir(tmpdir)
            self.ipfs.get(result.result)
            bag = rosbag.Bag(result.result, 'r')
            rospy.loginfo("ROSBAG: result has %s msgs", bag.get_message_count())
            rospy.loginfo("ROSBAG: result has start time %s", bag.get_start_time())
            return bag.get_message_count() != 0 and bag.get_start_time() > self.test_start_time

    def test_executor(self):
        rospy.Subscriber('liability/incoming', Liability, self.incoming_liability_handler)
        rospy.Subscriber('liability/result', Result, self.result_handler)
        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self.success)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestExecutor, sys.argv)
