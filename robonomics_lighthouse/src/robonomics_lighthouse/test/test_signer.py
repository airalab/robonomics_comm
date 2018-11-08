#!/usr/bin/env python

import unittest, rostest, sys, rospy, time
from robonomics_msgs.msg import Result, Offer, Demand
from ethereum_common import signer
from robonomics_msgs import messageValidator
from robonomics_lighthouse.test import testMessages
from binascii import hexlify

PKG = 'robonomics_lighthouse'
NAME = 'test_signer'


class TestSigner(unittest.TestCase):

    def __init__(self, *args):
        rospy.init_node(NAME)
        super(TestSigner, self).__init__(*args)
        self._test_sign_ask_success = False
        self._test_sign_bid_success = False
        self._test_sign_res_success = False

    def test_demand_hash(self):
        self.assertEqual(bytearray.fromhex('70befa37eb23b512f1eec8d8cb4219ce720c5aafbb55ae043b66764870205afe'),
                         signer.demand_hash(messageValidator.dict2ask(testMessages.validAskDict)))

    def test_offer_hash(self):
        self.assertEqual(bytearray.fromhex('f371f7a55b0972dfb6d3a6d7e224f004593bb33230eab6e43a93d37ad2b5a6d5'),
                         signer.offer_hash(messageValidator.dict2bid(testMessages.validBidDict)))

    def test_result_hash(self):
        self.assertEqual(bytearray.fromhex('55c1a3f47762a8d67ed7c76bc140f85bdbf6eb41e70cc13aa41ed1791d939464'),
                         signer.result_hash(messageValidator.dict2res(testMessages.validResDict)))

    def signed_ask_handler(self, ask):
        self.assertEqual(testMessages.validAskDict['model'], ask.model.multihash)
        self.assertEqual(testMessages.validAskDict['objective'], ask.objective.multihash)
        self.assertEqual(testMessages.validAskDict['token'], ask.token)
        self.assertNotEqual(testMessages.validAskDict['signature'], hexlify(ask.signature).decode('utf-8'))
        self._test_sign_ask_success = True

    def test_sign_ask(self):
        rospy.Subscriber('/lighthouse/infochan/eth/sending/demand', Demand, self.signed_ask_handler)
        askPublisherTest = rospy.Publisher('/lighthouse/infochan/eth/signing/demand', Demand, queue_size=10)

        time.sleep(3) #because signer node subscribers may be not registered in master
        askPublisherTest.publish(testMessages.getValidAsk())
        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self._test_sign_ask_success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self._test_sign_ask_success)

    def signed_bid_handler(self, bid):
        self.assertEqual(testMessages.validBidDict['model'], bid.model.multihash)
        self.assertEqual(testMessages.validBidDict['objective'], bid.objective.multihash)
        self.assertEqual(testMessages.validBidDict['token'], bid.token)
        self.assertNotEqual(testMessages.validBidDict['signature'], hexlify(bid.signature).decode('utf-8'))
        self._test_sign_bid_success = True

    def test_sign_bid(self):
        rospy.Subscriber('/lighthouse/infochan/eth/sending/offer', Offer, self.signed_bid_handler)
        bidPublisherTest = rospy.Publisher('/lighthouse/infochan/eth/signing/offer', Offer, queue_size=10)

        time.sleep(3) #because signer node subscribers may be not registered in master
        bidPublisherTest.publish(testMessages.getValidBid())
        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self._test_sign_bid_success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self._test_sign_bid_success)

    def signed_res_handler(self, res):
        self.assertEqual(testMessages.validResDict['liability'], res.liability)
        self.assertEqual(testMessages.validResDict['result'], res.result.multihash)
        self.assertNotEqual(testMessages.validResDict['signature'], hexlify(res.signature).decode('utf-8'))
        self._test_sign_res_success = True

    def test_sign_res(self):
        rospy.Subscriber('/lighthouse/infochan/eth/sending/result', Result, self.signed_res_handler)
        resPublisherTest = rospy.Publisher('/lighthouse/infochan/eth/signing/result', Result, queue_size=10)

        time.sleep(3)  # because signer node subscribers may be not registered in master
        resPublisherTest.publish(testMessages.getValidRes())
        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self._test_sign_res_success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self._test_sign_res_success)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestSigner, sys.argv)
