#!/usr/bin/env python

import unittest, rostest, sys
from robonomics_msgs.msg import Result, Offer, Demand, AddedOrderFeedback, AddedPendingTransactionFeedback
from ethereum_common.msg import UInt256
from robonomics_msgs import messageValidator
import testMessages

PKG = 'robonomics_liability'
NAME = 'test_messageValidator'


class TestMessageValidator(unittest.TestCase):
    def setUp(self):
        self.multiplier = 2

    def test_ValidAsk(self):
        f = messageValidator.convertMessage(testMessages.validAskDict)
        assert isinstance(f, Demand)

    def test_ValidAskWithENS(self):
        f = messageValidator.convertMessage(testMessages.validAskWithENSNamesDict)
        assert isinstance(f, Demand)

    def test_ValidBid(self):
        f = messageValidator.convertMessage(testMessages.validBidDict)
        assert isinstance(f, Offer)

    def test_ValidRes(self):
        f = messageValidator.convertMessage(testMessages.validResDict)
        assert isinstance(f, Result)

    def test_Wrong(self):
        f = messageValidator.convertMessage(testMessages.someInvalidMsgDict)
        assert f is None

    def test_InvalidAsk1(self):
        f = messageValidator.convertMessage(testMessages.invalidAsk1Dict)
        assert f is None

    def test_InvalidBid1(self):
        f = messageValidator.convertMessage(testMessages.invalidBid1Dict)
        assert f is None

    def test_InvalidRes1(self):
        f = messageValidator.convertMessage(testMessages.invalidRes1Dict)
        assert f is None

    def test_InvalidAsk2(self):
        f = messageValidator.convertMessage(testMessages.invalidAsk2Dict)
        assert f is None

    def test_InvalidBid2(self):
        f = messageValidator.convertMessage(testMessages.invalidBid2Dict)
        assert f is None

    def test_InvalidRes2(self):
        f = messageValidator.convertMessage(testMessages.invalidRes2Dict)
        assert f is None

    def test_InvalidAsk3(self):
        f = messageValidator.convertMessage(testMessages.invalidAsk3Dict)
        assert f is None

    def test_InvalidBid3(self):
        f = messageValidator.convertMessage(testMessages.invalidBid3Dict)
        assert f is None

    def test_InvalidRes3(self):
        f = messageValidator.convertMessage(testMessages.invalidRes3Dict)
        assert f is None

    def test_InvalidAsk4(self):
        f = messageValidator.convertMessage(testMessages.invalidAsk4Dict)
        assert f is None

    def test_InvalidBid4(self):
        f = messageValidator.convertMessage(testMessages.invalidBid4Dict)
        assert f is None

    def test_ValidAddedOrderFeedback(self):
        f = messageValidator.convertMessage(testMessages.validAddedOrderFeedbackDict)
        assert isinstance(f, AddedOrderFeedback)

    def test_ValidAddedPendingTransactionFeedback(self):
        f = messageValidator.convertMessage(testMessages.validAddedPendingTransactionFeedbackDict)
        assert isinstance(f, AddedPendingTransactionFeedback)

    ###
    # test UInt256 messages
    ###
    def test_UInt256_comparison(self):
        u1 = UInt256("100")
        u2 = UInt256("101")
        u3 = UInt256("101")

        # test comparisons with UInt256
        self.assertGreater(u2, u1)
        self.assertLess(u1, u2)

        self.assertGreaterEqual(u2, u1)
        self.assertGreaterEqual(u3, u1)
        self.assertGreaterEqual(u3, u2)
        self.assertGreaterEqual(u2, u3)

        self.assertLessEqual(u1, u2)
        self.assertLessEqual(u1, u3)
        self.assertLessEqual(u3, u2)
        self.assertLessEqual(u2, u3)

        self.assertEqual(u2, u3)

        # comparison with int
        self.assertGreater(u2, 100)
        self.assertLess(u1, 101)

        self.assertGreaterEqual(u2, 100)
        self.assertGreaterEqual(u3, 100)
        self.assertGreaterEqual(u3, 101)

        self.assertLessEqual(u1, 101)
        self.assertLessEqual(u3, 101)

        self.assertEqual(u1, 100)

    def test_UInt256_default_value(self):
        u = UInt256()
        print(u)
        self.assertEqual(0, int(u.uint256))

    def test_UInt256_arithmetic(self):
        u1 = UInt256("100")
        u2 = UInt256("101")

        u1 += 1
        self.assertEqual(u1, u2)

        u1 = UInt256("100")
        u3 = u1 + 1
        self.assertEqual(u3, u2)

        u3 = u1 + UInt256("1")
        self.assertEqual(u3, u2)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestMessageValidator, sys.argv)
