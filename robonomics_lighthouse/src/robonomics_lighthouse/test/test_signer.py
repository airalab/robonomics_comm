#!/usr/bin/env python

import unittest, rostest, sys, rospy, time
from robonomics_msgs.msg import Result, Offer, Demand
from ethereum_common import signer
from robonomics_lighthouse import messageValidator
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
        self.assertEqual(bytearray.fromhex('4f7cae2f478f7052b61e7bbd52cc8b70f7b483004d236ea370acd75620fd1000'),
                         signer.demand_hash(messageValidator.dict2ask(testMessages.validAskDict)))

    def test_offer_hash(self):
        self.assertEqual(bytearray.fromhex('9b6864a449c21212a5588c2d81863a9f3d24abe2b99d90755b5fb0a9292e977e'),
                         signer.offer_hash(messageValidator.dict2bid(testMessages.validBidDict)))

    def test_result_hash(self):
        self.assertEqual(bytearray.fromhex('55c1a3f47762a8d67ed7c76bc140f85bdbf6eb41e70cc13aa41ed1791d939464'),
                         signer.result_hash(messageValidator.dict2res(testMessages.validResDict)))

    def signed_ask_handler(self, ask):
        self.assertEqual(testMessages.validAskDict['model'], ask.model)
        self.assertEqual(testMessages.validAskDict['objective'], ask.objective)
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
        self.assertEqual(testMessages.validBidDict['model'], bid.model)
        self.assertEqual(testMessages.validBidDict['objective'], bid.objective)
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
        self.assertEqual(testMessages.validResDict['result'], res.result)
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
