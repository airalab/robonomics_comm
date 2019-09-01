#!/usr/bin/env python

import unittest, rostest, os, time, rospy, rosbag

from robonomics_msgs.msg import Result, Offer, Demand
from robonomics_msgs import robonomicsMessageUtils
from robonomics_liability.msg import Liability
from robonomics_liability.srv import FinishLiability, StartLiability
from urllib.parse import urlparse
import ipfshttpclient
from tempfile import TemporaryDirectory
from std_msgs.msg import *
from web3 import Web3, HTTPProvider
from ens import ENS

PKG = 'robonomics_liability'
NAME = 'test_executor'


class TestExecutor(unittest.TestCase):

    def __init__(self, *args):
        rospy.init_node(NAME)
        super(TestExecutor, self).__init__(*args)

        ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfs_client = ipfshttpclient.connect("/dns/{0}/tcp/{1}/http".format(ipfs_provider[0], ipfs_provider[1]))

        self.test_token = rospy.get_param('~test_token')
        self.test_bid_publisher = rospy.Publisher('/liability/test_executor/o/eth/signing/offer', Offer, queue_size=10)
        self.test_ask_publisher = rospy.Publisher('/liability/test_executor/d/eth/signing/demand', Demand, queue_size=10)

        web3_http_provider = rospy.get_param('~web3_http_provider')
        http_provider = HTTPProvider(web3_http_provider)
        ens_contract = rospy.get_param('~ens_contract', None)

        self.ens = ENS(http_provider, addr=ens_contract)
        self.web3 = Web3(http_provider, ens=self.ens)

        from web3.middleware import geth_poa_middleware
        # inject the poa compatibility middleware to the innermost layer
        self.web3.middleware_stack.inject(geth_poa_middleware, layer=0)
        self.ens.web3.middleware_stack.inject(geth_poa_middleware, layer=0)
        self.lighthouse_address = rospy.get_param('~lighthouse_contract')

        self.test_start_time = time.time()

        self.success = False
        self.ready_liability = None

        self.test_objective = self.create_test_objective()
        rospy.logwarn("TEST EXECUTOR: CURRENT OBJECTIVE is %s", self.test_objective)

    def create_test_objective(self):
        with TemporaryDirectory() as tmpdir:
            os.chdir(tmpdir)
            test_objective_bag = rosbag.Bag('output.bag', 'w')
            email_data = String(data='test@mail')
            droneid_data = String(data='test_drone_000')

            test_objective_bag.write('/agent/objective/droneid', droneid_data, rospy.Time().now())
            test_objective_bag.write('/agent/objective/email', email_data, rospy.Time().now())

            test_objective_bag.close()

            ipfs_objective = self.ipfs_client.add(test_objective_bag.filename)
            rospy.logwarn("TEST_EXECUTOR: ADD TEST OBJECTIVE TO IPFS is %s", ipfs_objective)
            return ipfs_objective['Hash']

    def ready_liability_handler(self, msg):
        self.ready_liability = msg
        rospy.logwarn("EXECUTOR: READY LIABILITY address is %s", self.ready_liability.address.address)

    def result_handler(self, msg):
        rospy.logwarn("EXECUTOR: liability: %s result %s", msg.liability, msg.result)

        if self.ready_liability is not None \
                and self.ready_liability.address.address == msg.liability.address \
                and self.check_rosbag_is_new_and_has_messages(msg):
            self.success = True

    def check_rosbag_is_new_and_has_messages(self, msg):
        with TemporaryDirectory() as tmpdir:
            os.chdir(tmpdir)
            self.ipfs_client.get(msg.result.multihash)
            bag = rosbag.Bag(msg.result.multihash, 'r')
            bag_topics = bag.get_type_and_topic_info()

            bag_topics_dict = {}
            for topic, topic_info in bag_topics[1].items():
                bag_topics_dict[topic] = topic_info[1]
                rospy.loginfo("rosbag contains %s messages of type %s in topic %s", topic_info[1], topic_info[0], topic)
            rospy.loginfo("ROSBAG: result has %s msgs", bag.get_message_count())
            rospy.loginfo("ROSBAG: result has start time %s", bag.get_start_time())
            return bag.get_message_count() == 2 and \
                   bag.get_start_time() > self.test_start_time and \
                   '/liability/eth_{0}/agent/objective/droneid'.format(self.ready_liability.address.address) in bag_topics_dict and \
                   '/liability/eth_{0}/agent/objective/email'.format(self.ready_liability.address.address) in bag_topics_dict

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
        start_service_proxy(self.ready_liability.address.address)

        time.sleep(5)

        finish_service_proxy = rospy.ServiceProxy('/liability/finish', FinishLiability)
        finish_service_proxy(self.ready_liability.address.address, True)

        while not rospy.is_shutdown() and not self.success:
            time.sleep(0.1)
        self.assert_(self.success)

    def get_test_bid(self):
        bidDict = {
            "model": "QmaRmbJtyfMDBfkDETTPAxKUUcSqZKXWwFKKoZ318nrPku",
            "objective": self.test_objective,
            "token": self.test_token,
            "cost": 0,
            "validator": '0x0000000000000000000000000000000000000000',
            "lighthouse": self.lighthouse_address,
            "lighthouseFee": 0,
            "deadline": 9999999,
            "sender": "",
            "nonce": 0,
            "signature": ""
        }
        return robonomicsMessageUtils.dict2offer(bidDict)

    def get_test_ask(self):
        askDict = {
            "model": "QmaRmbJtyfMDBfkDETTPAxKUUcSqZKXWwFKKoZ318nrPku",
            "objective": self.test_objective,
            "token": self.test_token,
            "cost": "0",
            "lighthouse": self.lighthouse_address,
            "validator": "0x0000000000000000000000000000000000000000",
            "validatorFee": 0,
            "deadline": 9999999,
            "sender": "",
            "nonce": 0,
            "signature": ""
        }
        return robonomicsMessageUtils.dict2demand(askDict)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestExecutor, sys.argv)
