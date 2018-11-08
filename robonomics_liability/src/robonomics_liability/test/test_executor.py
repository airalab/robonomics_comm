#!/usr/bin/env python

import unittest, rostest, os, time, rospy, rosbag

from robonomics_msgs.msg import Result, Offer, Demand, Multihash
from robonomics_liability.msg import Liability
from robonomics_liability.srv import FinishLiability, StartLiability
from urllib.parse import urlparse
import ipfsapi
from web3 import Web3, HTTPProvider
from ens import ENS
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
        self.test_bid_publisher = rospy.Publisher('/liability/infochan/eth/signing/offer', Offer, queue_size=10)
        self.test_ask_publisher = rospy.Publisher('/liability/infochan/eth/signing/demand', Demand, queue_size=10)

        web3_http_provider = rospy.get_param('~web3_http_provider')
        http_provider = HTTPProvider(web3_http_provider)
        ens_contract = rospy.get_param('~ens_contract', None)

        self.ens = ENS(http_provider, addr=ens_contract)
        self.web3 = Web3(http_provider, ens=self.ens)

        from web3.middleware import geth_poa_middleware
        # inject the poa compatibility middleware to the innermost layer
        self.web3.middleware_stack.inject(geth_poa_middleware, layer=0)
        self.ens.web3.middleware_stack.inject(geth_poa_middleware, layer=0)

        self.lighthouse_address = self.ens.address(rospy.get_param('~lighthouse_contract'))

        self.test_start_time = time.time()

        self.success = False
        self.ready_liability = None

    def ready_liability_handler(self, msg):
        self.ready_liability = msg
        rospy.loginfo("READY LIABILITY HANDLER: address is %s", self.ready_liability.address)

    def result_handler(self, result):
        rospy.loginfo("RESULT HANDLER: liability: %s result %s", result.liability, result.result)

        if self.ready_liability is not None \
                and self.ready_liability.address == result.liability \
                and self.check_rosbag_is_new_and_has_messages(result):
            self.success = True

    def check_rosbag_is_new_and_has_messages(self, result):
        with TemporaryDirectory() as tmpdir:
            os.chdir(tmpdir)
            self.ipfs.get(result.result.multihash)
            bag = rosbag.Bag(result.result.multihash, 'r')
            bag_topics = bag.get_type_and_topic_info()

            bag_topics_dict = {}
            for topic, topic_info in bag_topics[1].items():
                bag_topics_dict[topic] = topic_info[1]
                rospy.loginfo("rosbag contains %s messages of type %s in topic %s", topic_info[1], topic_info[0], topic)
            rospy.loginfo("ROSBAG: result has %s msgs", bag.get_message_count())
            rospy.loginfo("ROSBAG: result has start time %s", bag.get_start_time())
            return bag.get_message_count() == 2 and \
                   bag.get_start_time() > self.test_start_time and \
                   '/liability/eth_{0}/agent/objective/droneid'.format(self.ready_liability.address) in bag_topics_dict and \
                   '/liability/eth_{0}/agent/objective/email'.format(self.ready_liability.address) in bag_topics_dict

    def test_executor(self):
        rospy.Subscriber('liability/ready', Liability, self.ready_liability_handler)
        rospy.Subscriber('liability/result', Result, self.result_handler)

        time.sleep(15)
        rospy.logwarn("test_executor: publish test offer")
        self.test_bid_publisher.publish(self.get_test_bid())
        rospy.logwarn("test_executor: publish test demand")
        self.test_ask_publisher.publish(self.get_test_ask())

        while self.ready_liability is None:
            time.sleep(0.1)

        start_service_proxy = rospy.ServiceProxy('/liability/start', StartLiability)
        start_service_proxy(self.ready_liability.address)

        time.sleep(5)

        finish_service_proxy = rospy.ServiceProxy('/liability/finish', FinishLiability)
        finish_service_proxy(self.ready_liability.address, True)

        timeout_t = time.time() + 30.0
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self.success)

    def get_test_bid(self):
        bidDict = {
            "model": "QmaRmbJtyfMDBfkDETTPAxKUUcSqZKXWwFKKoZ318nrPku",
            "objective": "Qmb3H3tHZ1QutcrLq7WEtQWbEWjA11aPqVmeatMSrmFXvE",
            "token": self.test_token,
            "cost": 0,
            "validator": '0x0000000000000000000000000000000000000000',
            "lighthouse": self.lighthouse_address,
            "lighthouseFee": 0,
            "deadline": 9999999
        }
        bid = Offer()
        model_mh = Multihash()
        model_mh.multihash = bidDict['model']

        objective_mh = Multihash()
        objective_mh.multihash = bidDict['objective']

        bid.model = model_mh
        bid.objective = objective_mh
        bid.token = bidDict['token']
        bid.cost = bidDict['cost']
        bid.validator = bidDict['validator']
        bid.lighthouse = bidDict['lighthouse']
        bid.lighthouseFee = bidDict['lighthouseFee']
        bid.deadline = bidDict['deadline']
        return bid

    def get_test_ask(self):
        askDict = {
            "model": "QmaRmbJtyfMDBfkDETTPAxKUUcSqZKXWwFKKoZ318nrPku",
            "objective": "Qmb3H3tHZ1QutcrLq7WEtQWbEWjA11aPqVmeatMSrmFXvE",
            "token": self.test_token,
            "cost": 0,
            "lighthouse": self.lighthouse_address,
            "validator": "0x0000000000000000000000000000000000000000",
            "validatorFee": 0,
            "deadline": 9999999
        }
        ask = Demand()
        model_mh = Multihash()
        model_mh.multihash = askDict['model']

        objective_mh = Multihash()
        objective_mh.multihash = askDict['objective']

        ask.model = model_mh
        ask.objective = objective_mh

        ask.token = askDict['token']
        ask.cost = askDict['cost']
        ask.lighthouse = askDict['lighthouse']
        ask.validator = askDict['validator']
        ask.validatorFee = askDict['validatorFee']
        ask.deadline = askDict['deadline']
        return ask


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestExecutor, sys.argv)
