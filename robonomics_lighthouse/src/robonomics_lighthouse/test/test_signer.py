#!/usr/bin/env python

import unittest, rostest, sys, rospy, time
from robonomics_lighthouse import signer
from robonomics_lighthouse import messageValidator
from robonomics_lighthouse.msg import Ask, Bid, Result
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

    def test_askhash(self):
        self.assertEqual(bytearray.fromhex('4f7cae2f478f7052b61e7bbd52cc8b70f7b483004d236ea370acd75620fd1000'),
                         signer.askhash(messageValidator.dict2ask(testMessages.validAskDict)))

    def test_bidhash(self):
        self.assertEqual(bytearray.fromhex('1977a06ea783dfd99ee0efa85a59deadc5b80cedc15074d487e65e4bf3580828'),
                         signer.bidhash(messageValidator.dict2bid(testMessages.validBidDict)))

    def test_reshash(self):
        self.assertEqual(bytearray.fromhex('b79fade824c5785bc17347a3c6abe50de87aa4b0414281f9010c05ea3ea7c805'),
                         signer.reshash(messageValidator.dict2res(testMessages.validResDict)))

    def test_invalidRes1_reshash(self):
        self.assertNotEqual(bytearray.fromhex('b79fade824c5785bc17347a3c6abe50de87aa4b0414281f9010c05ea3ea7c666'),
                            signer.reshash(messageValidator.dict2res(testMessages.validResDict)))



    def signed_ask_handler(self, ask):
        self.assertEqual(testMessages.validAskDict['model'], ask.model)
        self.assertEqual(testMessages.validAskDict['objective'], ask.objective)
        self.assertEqual(testMessages.validAskDict['token'], ask.token)
        self.assertNotEqual(testMessages.validAskDict['signature'], hexlify(ask.signature).decode('utf-8'))
        self._test_sign_ask_success = True

    def test_sign_ask(self):
        rospy.Subscriber('/lighthouse/infochan/sending/ask', Ask, self.signed_ask_handler)
        askPublisherTest = rospy.Publisher('/lighthouse/infochan/signing/ask', Ask, queue_size=10)

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
        rospy.Subscriber('/lighthouse/infochan/sending/bid', Bid, self.signed_bid_handler)
        bidPublisherTest = rospy.Publisher('/lighthouse/infochan/signing/bid', Bid, queue_size=10)

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
        rospy.Subscriber('/lighthouse/infochan/sending/result', Result, self.signed_res_handler)
        resPublisherTest = rospy.Publisher('/lighthouse/infochan/signing/result', Result, queue_size=10)

        time.sleep(3)  # because signer node subscribers may be not registered in master
        resPublisherTest.publish(testMessages.getValidRes())
        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self._test_sign_res_success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self._test_sign_res_success)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestSigner, sys.argv)
