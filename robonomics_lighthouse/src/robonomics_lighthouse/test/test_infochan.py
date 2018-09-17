#!/usr/bin/env python

import rostest, sys, unittest
from robonomics_lighthouse.test import testMessages
from robonomics_lighthouse import infochan

PKG = 'robonomics_lighthouse'
NAME = 'test_infochan'


class TestInfochan(unittest.TestCase):
    def setUp(self):
        self.multiplier = 2

    def test_ask2dict(self):
        self.assertEqual(testMessages.validAskDict, infochan.ask2dict(testMessages.getValidAsk()))

    def test_bid2dict(self):
        self.assertEqual(testMessages.validBidDict, infochan.bid2dict(testMessages.getValidBid()))

    def test_res2dict(self):
        self.assertEqual(testMessages.validResDict, infochan.res2dict(testMessages.getValidRes()))


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestInfochan, sys.argv)