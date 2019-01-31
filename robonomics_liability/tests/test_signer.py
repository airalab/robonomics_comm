#!/usr/bin/env python

import unittest, rostest, sys, rospy, time
from robonomics_msgs.msg import Result, Offer, Demand
from robonomics_msgs import robonomicsMessageUtils
from ethereum_common import eth_keyfile_helper
from ethereum_common.msg import UInt256
from robonomics_msgs import messageValidator
import testMessages
from binascii import hexlify

PKG = 'robonomics_liability'
NAME = 'test_signer'


class TestSigner(unittest.TestCase):

    def __init__(self, *args):
        rospy.init_node(NAME)
        super(TestSigner, self).__init__(*args)
        self._test_sign_ask_success = False
        self._test_sign_bid_success = False
        self._test_sign_res_success = False

        __keyfile = rospy.get_param('~keyfile')
        __keyfile_password_file = rospy.get_param('~keyfile_password_file')

        __keyfile_helper = eth_keyfile_helper.KeyfileHelper(__keyfile, keyfile_password_file=__keyfile_password_file)
        self.__account = __keyfile_helper.get_local_account_from_keyfile()


    def test_demand_hash(self):
        test_nonce_value = UInt256("0")
        rospy.logwarn("test_demand_hash: %s", hexlify(robonomicsMessageUtils.demand_hash(messageValidator.dict2ask(testMessages.validAskDict), nonce=test_nonce_value)))
        self.assertEqual(bytearray.fromhex('50f82ac8c4905d14a65632915fa21cf3726c720fd9650d3b5ac134ba2ebb9da2'),
                         robonomicsMessageUtils.demand_hash(messageValidator.dict2ask(testMessages.validAskDict), nonce=test_nonce_value))

    def test_offer_hash(self):
        test_nonce_value = UInt256("0")
        rospy.logwarn("test_offer_hash: %s", hexlify(robonomicsMessageUtils.offer_hash(messageValidator.dict2bid(testMessages.validBidDict), nonce=test_nonce_value)))
        self.assertEqual(bytearray.fromhex('50f82ac8c4905d14a65632915fa21cf3726c720fd9650d3b5ac134ba2ebb9da2'),
                         robonomicsMessageUtils.offer_hash(messageValidator.dict2bid(testMessages.validBidDict), nonce=test_nonce_value))

    def test_result_hash(self):
        self.assertEqual(bytearray.fromhex('55c1a3f47762a8d67ed7c76bc140f85bdbf6eb41e70cc13aa41ed1791d939464'),
                         robonomicsMessageUtils.result_hash(messageValidator.dict2res(testMessages.validResDict)))

    def signed_ask_handler(self, ask):
        self.assertEqual(testMessages.validAskDict['model'], ask.model.multihash)
        self.assertEqual(testMessages.validAskDict['objective'], ask.objective.multihash)
        self.assertEqual(testMessages.validAskDict['token'], ask.token.address)
        self.assertNotEqual(testMessages.validAskDict['signature'], hexlify(ask.signature).decode('utf-8'))

        self.assertEqual(self.__account.address, ask.sender.address)
        self._test_sign_ask_success = True

    def test_sign_ask(self):
        rospy.Subscriber('/liability/infochan/eth/sending/demand', Demand, self.signed_ask_handler)
        askPublisherTest = rospy.Publisher('/liability/infochan/eth/signing/demand', Demand, queue_size=10)

        time.sleep(3) #because signer node subscribers may be not registered in master
        askPublisherTest.publish(testMessages.getValidAsk())
        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self._test_sign_ask_success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self._test_sign_ask_success)

    def signed_bid_handler(self, bid):
        self.assertEqual(testMessages.validBidDict['model'], bid.model.multihash)
        self.assertEqual(testMessages.validBidDict['objective'], bid.objective.multihash)
        self.assertEqual(testMessages.validBidDict['token'], bid.token.address)
        self.assertNotEqual(testMessages.validBidDict['signature'], hexlify(bid.signature).decode('utf-8'))

        self.assertEqual(self.__account.address, bid.sender.address)

        self._test_sign_bid_success = True

    def test_sign_bid(self):
        rospy.Subscriber('/liability/infochan/eth/sending/offer', Offer, self.signed_bid_handler)
        bidPublisherTest = rospy.Publisher('/liability/infochan/eth/signing/offer', Offer, queue_size=10)

        time.sleep(3) #because signer node subscribers may be not registered in master
        bidPublisherTest.publish(testMessages.getValidBid())
        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self._test_sign_bid_success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self._test_sign_bid_success)

    def signed_res_handler(self, res):
        self.assertEqual(testMessages.validResDict['liability'], res.liability.address)
        self.assertEqual(testMessages.validResDict['result'], res.result.multihash)
        self.assertNotEqual(testMessages.validResDict['signature'], hexlify(res.signature).decode('utf-8'))

        self.assertEqual(self.__account.address, robonomicsMessageUtils.get_result_msg_sender_address(res))

        self._test_sign_res_success = True

    def test_sign_res(self):
        rospy.Subscriber('/liability/infochan/eth/sending/result', Result, self.signed_res_handler)
        resPublisherTest = rospy.Publisher('/liability/infochan/eth/signing/result', Result, queue_size=10)

        time.sleep(3)  # because signer node subscribers may be not registered in master
        resPublisherTest.publish(testMessages.getValidRes())
        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self._test_sign_res_success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self._test_sign_res_success)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestSigner, sys.argv)
