#!/usr/bin/env python

import unittest, rostest, os, time, rospy, rosbag

from robonomics_msgs.msg import Result, Offer, Demand
from robonomics_liability.msg import Liability
from robonomics_liability.srv import FinishLiability
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

        self.test_token = rospy.get_param('~test_token')
        self.test_bid_publisher = rospy.Publisher('/liability/infochan/signing/offer', Offer, queue_size=10)
        self.test_ask_publisher = rospy.Publisher('/liability/infochan/signing/demand', Demand, queue_size=10)

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

        time.sleep(15)
        self.test_bid_publisher.publish(self.get_test_bid())
        self.test_ask_publisher.publish(self.get_test_ask())

        while self.incoming_liability is None:
            time.sleep(0.1)

        time.sleep(5)
        finish_service_proxy = rospy.ServiceProxy('/liability/finish', FinishLiability)
        finish_service_proxy(False)

        timeout_t = time.time() + 30.0
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self.success)


    def get_test_bid(self):
        bidDict = {
            "model": "QmaRmbJtyfMDBfkDETTPAxKUUcSqZKXWwFKKoZ318nrPku",
            "objective": "Qmb3H3tHZ1QutcrLq7WEtQWbEWjA11aPqVmeatMSrmFXvE",
            "token": self.test_token,
            "cost": 1,
            "validator": '0x0000000000000000000000000000000000000000',
            "lighthouseFee": 0,
            "deadline": 9999999
        }
        bid = Offer()
        bid.model = bidDict['model']
        bid.objective = bidDict['objective']
        bid.token = bidDict['token']
        bid.cost = bidDict['cost']
        bid.validator = bidDict['validator']
        bid.lighthouseFee = bidDict['lighthouseFee']
        bid.deadline = bidDict['deadline']
        return bid


    def get_test_ask(self):
        askDict = {
            "model": "QmaRmbJtyfMDBfkDETTPAxKUUcSqZKXWwFKKoZ318nrPku",
            "objective": "Qmb3H3tHZ1QutcrLq7WEtQWbEWjA11aPqVmeatMSrmFXvE",
            "token": self.test_token,
            "cost": 1,
            "validator": "0x0000000000000000000000000000000000000000",
            "validatorFee": 0,
            "deadline": 9999999
        }
        ask = Demand()
        ask.model = askDict['model']
        ask.objective = askDict['objective']
        ask.token = askDict['token']
        ask.cost = askDict['cost']
        ask.validator = askDict['validator']
        ask.validatorFee = askDict['validatorFee']
        ask.deadline = askDict['deadline']
        return ask


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestExecutor, sys.argv)
