#!/usr/bin/env python

import unittest, rostest, sys
from robonomics_msgs.msg import Result, Offer, Demand
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


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestMessageValidator, sys.argv)
