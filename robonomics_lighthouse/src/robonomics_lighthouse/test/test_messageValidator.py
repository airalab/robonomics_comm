#!/usr/bin/env python

import unittest, rostest, sys
from robonomics_lighthouse.msg import Ask, Bid, Result
from robonomics_lighthouse import messageValidator
from robonomics_lighthouse.test import testMessages

PKG = 'robonomics_lighthouse'
NAME = 'test_messageConverter'


class TestMessageConverter(unittest.TestCase):
    def setUp(self):
        self.multiplier = 2

    def test_ValidAsk(self):
        f = messageValidator.convertMessage(testMessages.validAskDict)
        assert isinstance(f, Ask)

    def test_ValidBid(self):
        f = messageValidator.convertMessage(testMessages.validBidDict)
        assert isinstance(f, Bid)

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

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestMessageConverter, sys.argv)